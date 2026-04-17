// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml6_ros2_plugin/tf_buffer.hpp"
#include "logging.hpp"
#include "qml6_ros2_plugin/conversion/message_conversions.hpp"
#include "qml6_ros2_plugin/conversion/qml_ros_conversion.hpp"
#include "qml6_ros2_plugin/ros2.hpp"
#include "qml6_ros2_plugin/tf_frame_info.hpp"

#include <algorithm>
#include <cstring>
#include <rcl/validate_topic_name.h>
#include <tf2_ros/qos.hpp>

using namespace qml6_ros2_plugin::conversion;

namespace qml6_ros2_plugin
{

namespace
{
constexpr std::size_t kMaxCacheEntries = 30;
constexpr std::chrono::seconds kMaxCacheAge{ 60 };
} // namespace

TfBuffer::TfBuffer( QObject *parent ) : QObjectRos2( parent ) { }

TfBuffer::~TfBuffer() { unsubscribeTopics(); }

TfBuffer::CallbackActivityGuard::CallbackActivityGuard( TfBuffer *buffer )
    : buffer_( buffer ), active_( false )
{
  if ( buffer_ == nullptr )
    return;
  std::lock_guard<std::mutex> lock( buffer_->callback_state_mutex_ );
  if ( !buffer_->accepting_callbacks_ )
    return;
  ++buffer_->active_callback_count_;
  active_ = true;
}

TfBuffer::CallbackActivityGuard::~CallbackActivityGuard()
{
  if ( !active_ || buffer_ == nullptr )
    return;
  std::lock_guard<std::mutex> lock( buffer_->callback_state_mutex_ );
  --buffer_->active_callback_count_;
  buffer_->callback_state_cv_.notify_all();
}

bool TfBuffer::CallbackActivityGuard::active() const { return active_; }

QString TfBuffer::ns() const { return namespace_; }

void TfBuffer::setNs( const QString &ns )
{
  QString clean_ns = ns;
  if ( clean_ns.endsWith( '/' ) )
    clean_ns.chop( 1 );
  if ( clean_ns == namespace_ )
    return;
  int validation_result = 0;
  size_t invalid_index = 0;
  rcl_ret_t ret = rcl_validate_topic_name( ( clean_ns.toStdString() + "/tf" ).c_str(),
                                           &validation_result, &invalid_index );
  if ( ret != RCL_RET_OK || validation_result != RCL_TOPIC_NAME_VALID ) {
    QML_ROS2_PLUGIN_ERROR( "Invalid namespace '%s': %s (at index %zu)",
                           clean_ns.toStdString().c_str(), rcl_get_error_string().str,
                           invalid_index );
    rcl_reset_error();
    return;
  }
  namespace_ = clean_ns;
  if ( isRosInitialized() ) {
    unsubscribeTopics();
    if ( buffer_ != nullptr )
      buffer_->clear();
    subscribeTopics();
  }
  emit nsChanged();
}

void TfBuffer::clear()
{
  unsubscribeTopics();
  if ( buffer_ != nullptr )
    buffer_->clear();
  subscribeTopics();
}

tf2_ros::Buffer *TfBuffer::buffer() { return buffer_.get(); }

void TfBuffer::onRos2Initialized()
{
  auto node = Ros2Qml::getInstance().node();
  if ( node == nullptr )
    return;
  buffer_ = std::make_unique<tf2_ros::Buffer>( node->get_clock() );
  buffer_->setUsingDedicatedThread( true );
  subscribeTopics();
}

void TfBuffer::onRos2Shutdown()
{
  unsubscribeTopics();
  buffer_.reset();
}

void TfBuffer::subscribeTopics()
{
  auto node = Ros2Qml::getInstance().node();
  if ( node == nullptr )
    return;
  tf_topic_ = namespace_.toStdString() + "/tf";
  tf_static_topic_ = namespace_.toStdString() + "/tf_static";

  // Match the QoS that tf2_ros::TransformListener uses internally so behavior is consistent with
  // the global listener path.
  const rclcpp::QoS tf_qos = tf2_ros::DynamicListenerQoS();
  const rclcpp::QoS tf_static_qos = tf2_ros::StaticListenerQoS();

  rclcpp::SubscriptionOptions opts;
  QPointer<TfBuffer> self( this );
  {
    std::lock_guard<std::mutex> lock( callback_state_mutex_ );
    accepting_callbacks_ = true;
  }
  tf_sub_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
      tf_topic_, tf_qos,
      [self]( tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo &info ) {
        if ( self.isNull() )
          return;
        self->tfCallback( msg, info, false );
      },
      opts );
  tf_static_sub_ = node->create_subscription<tf2_msgs::msg::TFMessage>(
      tf_static_topic_, tf_static_qos,
      [self]( tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo &info ) {
        if ( self.isNull() )
          return;
        self->tfCallback( msg, info, true );
      },
      opts );
}

void TfBuffer::unsubscribeTopics()
{
  {
    std::unique_lock<std::mutex> lock( callback_state_mutex_ );
    accepting_callbacks_ = false;
  }
  tf_sub_.reset();
  tf_static_sub_.reset();
  {
    std::unique_lock<std::mutex> lock( callback_state_mutex_ );
    callback_state_cv_.wait( lock, [this]() { return active_callback_count_ == 0; } );
  }
  std::lock_guard<std::mutex> lock( gid_cache_mutex_ );
  std::lock_guard<std::mutex> lock_auth( authority_mutex_ );
  gid_cache_.clear();
  frame_states_.clear();
}

void TfBuffer::tfCallback( tf2_msgs::msg::TFMessage::ConstSharedPtr msg,
                           const rclcpp::MessageInfo &info, bool is_static )
{
  CallbackActivityGuard callback_guard( this );
  if ( !callback_guard.active() )
    return;
  if ( !buffer_ )
    return;
  std::array<uint8_t, RMW_GID_STORAGE_SIZE> gid{};
  std::memcpy( gid.data(), info.get_rmw_message_info().publisher_gid.data, RMW_GID_STORAGE_SIZE );
  const std::string authority = resolveAuthority( gid, is_static ? tf_static_topic_ : tf_topic_ );
  {
    std::lock_guard<std::mutex> lock( authority_mutex_ );
    for ( const auto &t : msg->transforms ) {
      const std::string child_frame_id = t.child_frame_id;
      const std::string parent_frame_id = t.header.frame_id;

      auto [state_it, inserted] = frame_states_.try_emplace( child_frame_id );
      if ( inserted || state_it->second.frame_id.empty() )
        state_it->second.frame_id = child_frame_id;
      // Update parent and children bookkeeping.
      if ( state_it->second.parent_id != parent_frame_id ) {
        // Remove from old parent's children.
        if ( !state_it->second.parent_id.empty() ) {
          auto old_it = frame_states_.find( state_it->second.parent_id );
          if ( old_it != frame_states_.end() ) {
            auto &oc = old_it->second.children;
            oc.erase( std::remove( oc.begin(), oc.end(), child_frame_id ), oc.end() );
          }
        }
        auto [parent_it, parent_inserted] = frame_states_.try_emplace( parent_frame_id );
        if ( parent_inserted || parent_it->second.frame_id.empty() )
          parent_it->second.frame_id = parent_frame_id;

        state_it = frame_states_.find( child_frame_id );
        state_it->second.parent_id = parent_frame_id;
        // Add to new parent's children.
        auto &pc = parent_it->second.children;
        if ( std::find( pc.begin(), pc.end(), child_frame_id ) == pc.end() )
          pc.push_back( child_frame_id );
      }
      state_it->second.authority = authority;
      state_it->second.is_static = is_static;
      state_it->second.transform = t.transform;
      state_it->second.last_stamp = rclcpp::Time( t.header.stamp );
      state_it->second.frequency_tracker.addSample( std::chrono::steady_clock::now() );
    }
  }
  for ( const auto &t : msg->transforms ) {
    try {
      buffer_->setTransform( t, authority, is_static );
    } catch ( const tf2::TransformException &ex ) {
      QML_ROS2_PLUGIN_WARN( "setTransform failed: %s", ex.what() );
    }
  }
}

std::string TfBuffer::resolveAuthority( const std::array<uint8_t, RMW_GID_STORAGE_SIZE> &gid,
                                        const std::string &topic )
{
  const auto now = std::chrono::steady_clock::now();
  {
    std::lock_guard<std::mutex> lock( gid_cache_mutex_ );
    for ( auto &entry : gid_cache_ ) {
      if ( entry.gid == gid ) {
        entry.last_seen = now;
        return entry.authority;
      }
    }
  }
  std::string authority = "unknown_publisher";
  auto node = Ros2Qml::getInstance().node();
  if ( node != nullptr ) {
    const auto infos = node->get_publishers_info_by_topic( topic );
    for ( const auto &info : infos ) {
      const auto &endpoint_gid = info.endpoint_gid();
      if ( std::equal( gid.begin(), gid.end(), endpoint_gid.begin() ) ) {
        std::string ns = info.node_namespace();
        const std::string &name = info.node_name();
        if ( ns.empty() || ns == "/" ) {
          authority = "/" + name;
        } else {
          if ( ns.back() == '/' )
            ns.pop_back();
          authority = ns + "/" + name;
        }
        break;
      }
    }
  }
  std::lock_guard<std::mutex> lock( gid_cache_mutex_ );
  gid_cache_.push_back( GidCacheEntry{ gid, authority, now } );
  evictCacheIfNeeded();
  return authority;
}

void TfBuffer::evictCacheIfNeeded()
{
  // When called we already have a mutex locked
  if ( gid_cache_.size() <= kMaxCacheEntries )
    return;
  const auto cutoff = std::chrono::steady_clock::now() - kMaxCacheAge;
  gid_cache_.erase(
      std::remove_if( gid_cache_.begin(), gid_cache_.end(),
                      [cutoff]( const GidCacheEntry &e ) { return e.last_seen < cutoff; } ),
      gid_cache_.end() );
  if ( gid_cache_.size() > kMaxCacheEntries ) {
    std::sort( gid_cache_.begin(), gid_cache_.end(),
               []( const GidCacheEntry &a, const GidCacheEntry &b ) {
                 return a.last_seen < b.last_seen;
               } );
    gid_cache_.erase( gid_cache_.begin(),
                      gid_cache_.begin() +
                          static_cast<std::ptrdiff_t>( gid_cache_.size() - kMaxCacheEntries ) );
  }
}

// ---- Native (rclcpp::Time) canTransform / lookUpTransform ----

QVariant TfBuffer::canTransform( const QString &target_frame, const QString &source_frame,
                                 const rclcpp::Time &time, double timeout ) const
{
  if ( buffer_ == nullptr )
    return QString( "Uninitialized" );
  std::string error;
  bool result;
  if ( timeout <= 0.0000001 ) {
    using namespace std::chrono_literals;
    result = buffer_->canTransform( target_frame.toStdString(), source_frame.toStdString(), time,
                                    rclcpp::Duration( 0ns ), &error );
  } else {
    result = buffer_->canTransform( target_frame.toStdString(), source_frame.toStdString(), time,
                                    qmlToRos2Duration( timeout ), &error );
  }
  if ( result )
    return true;
  if ( error.empty() )
    return false;
  return QString::fromStdString( error );
}

QVariant TfBuffer::canTransform( const QString &target_frame, const rclcpp::Time &target_time,
                                 const QString &source_frame, const rclcpp::Time &source_time,
                                 const QString &fixed_frame, double timeout ) const
{
  if ( buffer_ == nullptr )
    return QString( "Uninitialized" );
  std::string error;
  bool result;
  if ( timeout <= 0.0000001 ) {
    using namespace std::chrono_literals;
    result = buffer_->canTransform( target_frame.toStdString(), target_time,
                                    source_frame.toStdString(), source_time,
                                    fixed_frame.toStdString(), rclcpp::Duration( 0ns ), &error );
  } else {
    result = buffer_->canTransform( target_frame.toStdString(), target_time,
                                    source_frame.toStdString(), source_time,
                                    fixed_frame.toStdString(), qmlToRos2Duration( timeout ), &error );
  }
  if ( result )
    return true;
  if ( error.empty() )
    return false;
  return QString::fromStdString( error );
}

QVariantMap TfBuffer::lookUpTransform( const QString &target_frame, const QString &source_frame,
                                       const rclcpp::Time &time, double timeout ) const
{
  geometry_msgs::msg::TransformStamped transform;
  if ( buffer_ == nullptr ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "Uninitialized" );
    result.insert( "message", "TfBuffer is not yet initialized!" );
    return result;
  }
  try {
    if ( timeout <= 1E-6 ) {
      transform =
          buffer_->lookupTransform( target_frame.toStdString(), source_frame.toStdString(), time );
    } else {
      transform = buffer_->lookupTransform( target_frame.toStdString(), source_frame.toStdString(),
                                            time, qmlToRos2Duration( timeout ) );
    }
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", true );
    return result;
  } catch ( tf2::LookupException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "LookupException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::ConnectivityException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ConnectivityException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::ExtrapolationException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ExtrapolationException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::InvalidArgumentException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "InvalidArgumentException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  }
}

QVariantMap TfBuffer::lookUpTransform( const QString &target_frame, const rclcpp::Time &target_time,
                                       const QString &source_frame, const rclcpp::Time &source_time,
                                       const QString &fixed_frame, double timeout ) const
{
  geometry_msgs::msg::TransformStamped transform;
  if ( buffer_ == nullptr ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "Uninitialized" );
    result.insert( "message", "TfBuffer is not yet initialized!" );
    return result;
  }
  try {
    if ( timeout <= 0.0000001 ) {
      transform = buffer_->lookupTransform( target_frame.toStdString(), target_time,
                                            source_frame.toStdString(), source_time,
                                            fixed_frame.toStdString() );
    } else {
      transform = buffer_->lookupTransform( target_frame.toStdString(), target_time,
                                            source_frame.toStdString(), source_time,
                                            fixed_frame.toStdString(), qmlToRos2Duration( timeout ) );
    }
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", true );
    return result;
  } catch ( tf2::LookupException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "LookupException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::ConnectivityException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ConnectivityException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::ExtrapolationException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "ExtrapolationException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  } catch ( tf2::InvalidArgumentException &ex ) {
    QVariantMap result = msgToMap( transform );
    result.insert( "valid", false );
    result.insert( "exception", "InvalidArgumentException" );
    result.insert( "message", QString( ex.what() ) );
    return result;
  }
}

// ---- Q_INVOKABLE overloads: one-liners into the rclcpp::Time versions ----

QVariant TfBuffer::canTransform( const QString &target_frame, const QString &source_frame,
                                 const QDateTime &time, double timeout ) const
{
  return canTransform( target_frame, source_frame, qmlToRos2Time( time ), timeout );
}

QVariant TfBuffer::canTransform( const QString &target_frame, const QString &source_frame,
                                 const qml6_ros2_plugin::Time &time, double timeout ) const
{
  return canTransform( target_frame, source_frame, time.getTime(), timeout );
}

QVariant TfBuffer::canTransform( const QString &target_frame, const QDateTime &target_time,
                                 const QString &source_frame, const QDateTime &source_time,
                                 const QString &fixed_frame, double timeout ) const
{
  return canTransform( target_frame, qmlToRos2Time( target_time ), source_frame,
                       qmlToRos2Time( source_time ), fixed_frame, timeout );
}

QVariant TfBuffer::canTransform( const QString &target_frame,
                                 const qml6_ros2_plugin::Time &target_time,
                                 const QString &source_frame,
                                 const qml6_ros2_plugin::Time &source_time,
                                 const QString &fixed_frame, double timeout ) const
{
  return canTransform( target_frame, target_time.getTime(), source_frame, source_time.getTime(),
                       fixed_frame, timeout );
}

QVariantMap TfBuffer::lookUpTransform( const QString &target_frame, const QString &source_frame,
                                       const QDateTime &time, double timeout ) const
{
  return lookUpTransform( target_frame, source_frame, qmlToRos2Time( time ), timeout );
}

QVariantMap TfBuffer::lookUpTransform( const QString &target_frame, const QString &source_frame,
                                       const qml6_ros2_plugin::Time &time, double timeout ) const
{
  return lookUpTransform( target_frame, source_frame, time.getTime(), timeout );
}

QVariantMap TfBuffer::lookUpTransform( const QString &target_frame, const QDateTime &target_time,
                                       const QString &source_frame, const QDateTime &source_time,
                                       const QString &fixed_frame, double timeout ) const
{
  return lookUpTransform( target_frame, qmlToRos2Time( target_time ), source_frame,
                          qmlToRos2Time( source_time ), fixed_frame, timeout );
}

QVariantMap TfBuffer::lookUpTransform( const QString &target_frame,
                                       const qml6_ros2_plugin::Time &target_time,
                                       const QString &source_frame,
                                       const qml6_ros2_plugin::Time &source_time,
                                       const QString &fixed_frame, double timeout ) const
{
  return lookUpTransform( target_frame, target_time.getTime(), source_frame, source_time.getTime(),
                          fixed_frame, timeout );
}

QString TfBuffer::getFrameAuthority( const QString &frame_id ) const
{
  std::lock_guard<std::mutex> lock( authority_mutex_ );
  auto it = frame_states_.find( frame_id.toStdString() );
  if ( it == frame_states_.end() )
    return {};
  return QString::fromStdString( it->second.authority );
}

TfFrameInfo TfBuffer::frameStateToInfo( const FrameState &state,
                                        std::chrono::steady_clock::time_point now )
{
  QVariantMap translation;
  translation["x"] = state.transform.translation.x;
  translation["y"] = state.transform.translation.y;
  translation["z"] = state.transform.translation.z;
  QVariantMap rotation;
  rotation["w"] = state.transform.rotation.w;
  rotation["x"] = state.transform.rotation.x;
  rotation["y"] = state.transform.rotation.y;
  rotation["z"] = state.transform.rotation.z;
  QStringList children;
  children.reserve( static_cast<int>( state.children.size() ) );
  for ( const auto &c : state.children ) children.append( QString::fromStdString( c ) );
  const auto rates = state.frequency_tracker.rates( now );
  return TfFrameInfo(
      QString::fromStdString( state.frame_id ), QString::fromStdString( state.parent_id ),
      QString::fromStdString( state.authority ), std::move( translation ), std::move( rotation ),
      state.is_static, std::move( children ), rates.frequency_hz );
}

QVariant TfBuffer::getFrame( const QString &frame_id ) const
{
  std::lock_guard<std::mutex> lock( authority_mutex_ );
  const auto now = std::chrono::steady_clock::now();
  auto it = frame_states_.find( frame_id.toStdString() );
  if ( it == frame_states_.end() )
    return {};
  return QVariant::fromValue( frameStateToInfo( it->second, now ) );
}

QVariantList TfBuffer::getAllFrames() const
{
  std::lock_guard<std::mutex> lock( authority_mutex_ );
  const auto now = std::chrono::steady_clock::now();
  QVariantList result;
  result.reserve( static_cast<int>( frame_states_.size() ) );
  for ( const auto &pair : frame_states_ )
    result.append( QVariant::fromValue( frameStateToInfo( pair.second, now ) ) );
  return result;
}

double TfBuffer::getTransformAge( const QString &frame_id ) const
{
  rclcpp::Time stamp;
  {
    std::lock_guard<std::mutex> lock( authority_mutex_ );
    auto it = frame_states_.find( frame_id.toStdString() );
    if ( it == frame_states_.end() || it->second.parent_id.empty() )
      return -1.0;
    stamp = it->second.last_stamp;
  }
  auto node = Ros2Qml::getInstance().node();
  if ( node == nullptr )
    return -1.0;
  try {
    return ( node->now() - stamp ).seconds();
  } catch ( ... ) {
    return -1.0;
  }
}

} // namespace qml6_ros2_plugin
