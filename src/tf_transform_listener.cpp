// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml6_ros2_plugin/tf_transform_listener.hpp"
#include "logging.hpp"
#include "qml6_ros2_plugin/conversion/message_conversions.hpp"
#include "qml6_ros2_plugin/conversion/qml_ros_conversion.hpp"
#include "qml6_ros2_plugin/ros2.hpp"
#include "qml6_ros2_plugin/tf_buffer.hpp"

#include <QVariantMap>
#include <memory>

using namespace qml6_ros2_plugin::conversion;

namespace qml6_ros2_plugin
{

struct TfTransformListener::State {
  State() : buffer( std::make_unique<TfBuffer>() ) { }

  std::unique_ptr<TfBuffer> buffer;
};

TfTransformListener &TfTransformListener::getInstance()
{
  static TfTransformListener instance;
  return instance;
}

TfTransformListener::TfTransformListener() : wrapper_count_( 0 ) { state_.reset(); }

TfTransformListener::~TfTransformListener() = default;

bool TfTransformListener::isInitialized() const { return state_ != nullptr; }

bool TfTransformListener::initialize()
{
  if ( state_ != nullptr )
    return true;
  if ( wrapper_count_ == 0 )
    return false;
  if ( !Ros2Qml::getInstance().isInitialized() ) {
    connect( &Ros2Qml::getInstance(), &Ros2Qml::initialized, this, &TfTransformListener::initialize );
    return true;
  }
  // The inner TfBuffer is a QObjectRos2 and registers itself as a Ros2Qml dependant for the
  // executor lifetime.
  state_ = std::make_unique<State>();
  return true;
}

QVariant TfTransformListener::canTransform( const QString &target_frame, const QString &source_frame,
                                            const rclcpp::Time &time, double timeout ) const
{
  if ( state_ == nullptr )
    return QString( "Uninitialized" );
  return state_->buffer->canTransform( target_frame, source_frame, time, timeout );
}

QVariant TfTransformListener::canTransform( const QString &target_frame,
                                            const rclcpp::Time &target_time,
                                            const QString &source_frame,
                                            const rclcpp::Time &source_time,
                                            const QString &fixed_frame, double timeout ) const
{
  if ( state_ == nullptr )
    return QString( "Uninitialized" );
  return state_->buffer->canTransform( target_frame, target_time, source_frame, source_time,
                                       fixed_frame, timeout );
}

QVariantMap TfTransformListener::lookUpTransform( const QString &target_frame,
                                                  const QString &source_frame,
                                                  const rclcpp::Time &time, double timeout ) const
{
  if ( state_ == nullptr ) {
    QVariantMap result = msgToMap( geometry_msgs::msg::TransformStamped{} );
    result.insert( "valid", false );
    result.insert( "exception", "Uninitialized" );
    result.insert( "message", "ROS node is not yet initialized!" );
    return result;
  }
  return state_->buffer->lookUpTransform( target_frame, source_frame, time, timeout );
}

QVariantMap TfTransformListener::lookUpTransform( const QString &target_frame,
                                                  const rclcpp::Time &target_time,
                                                  const QString &source_frame,
                                                  const rclcpp::Time &source_time,
                                                  const QString &fixed_frame, double timeout ) const
{
  if ( state_ == nullptr ) {
    QVariantMap result = msgToMap( geometry_msgs::msg::TransformStamped{} );
    result.insert( "valid", false );
    result.insert( "exception", "Uninitialized" );
    result.insert( "message", "ROS node is not yet initialized!" );
    return result;
  }
  return state_->buffer->lookUpTransform( target_frame, target_time, source_frame, source_time,
                                          fixed_frame, timeout );
}

QString TfTransformListener::getFrameAuthority( const QString &frame_id ) const
{
  if ( state_ == nullptr )
    return {};
  return state_->buffer->getFrameAuthority( frame_id );
}

QVariant TfTransformListener::getFrame( const QString &frame_id ) const
{
  if ( state_ == nullptr )
    return {};
  return state_->buffer->getFrame( frame_id );
}

QVariantList TfTransformListener::getAllFrames() const
{
  if ( state_ == nullptr )
    return {};
  return state_->buffer->getAllFrames();
}

double TfTransformListener::getTransformAge( const QString &frame_id ) const
{
  if ( state_ == nullptr )
    return -1.0;
  return state_->buffer->getTransformAge( frame_id );
}

void TfTransformListener::registerWrapper()
{
  if ( wrapper_count_++ == 0 )
    initialize();
}

void TfTransformListener::unregisterWrapper()
{
  int count = --wrapper_count_;
  if ( count == 0 ) {
    state_.reset();
  } else if ( count < 0 ) {
    QML_ROS2_PLUGIN_ERROR( "Unregister wrapper was called more often than registerWrapper for "
                           "TfTransformListener! This is a bug!" );
    wrapper_count_ += -count;
  }
}

tf2_ros::Buffer *TfTransformListener::buffer()
{
  return state_ == nullptr ? nullptr : state_->buffer->buffer();
}

TfTransformListenerWrapper::TfTransformListenerWrapper()
{
  TfTransformListener::getInstance().registerWrapper();
}

TfTransformListenerWrapper::~TfTransformListenerWrapper()
{
  TfTransformListener::getInstance().unregisterWrapper();
}

void TfTransformListenerWrapper::initialize()
{ /* only serves to create wrapper which will init anyway. */ }

QVariantMap TfTransformListenerWrapper::lookUpTransform( const QString &target_frame,
                                                         const QString &source_frame,
                                                         const QDateTime &time, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform( target_frame, source_frame,
                                                             qmlToRos2Time( time ), timeout );
}

QVariantMap TfTransformListenerWrapper::lookUpTransform( const QString &target_frame,
                                                         const QString &source_frame,
                                                         const Time &time, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform( target_frame, source_frame,
                                                             time.getTime(), timeout );
}

QVariantMap TfTransformListenerWrapper::lookUpTransform( const QString &target_frame,
                                                         const QDateTime &target_time,
                                                         const QString &source_frame,
                                                         const QDateTime &source_time,
                                                         const QString &fixed_frame, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform(
      target_frame, qmlToRos2Time( target_time ), source_frame, qmlToRos2Time( source_time ),
      fixed_frame, timeout );
}

QVariantMap TfTransformListenerWrapper::lookUpTransform( const QString &target_frame,
                                                         const Time &target_time,
                                                         const QString &source_frame,
                                                         const Time &source_time,
                                                         const QString &fixed_frame, double timeout )
{
  return TfTransformListener::getInstance().lookUpTransform( target_frame, target_time.getTime(),
                                                             source_frame, source_time.getTime(),
                                                             fixed_frame, timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame,
                                                   const QString &source_frame,
                                                   const QDateTime &time, double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, source_frame,
                                                          qmlToRos2Time( time ), timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame,
                                                   const QString &source_frame, const Time &time,
                                                   double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, source_frame,
                                                          time.getTime(), timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame,
                                                   const QDateTime &target_time,
                                                   const QString &source_frame,
                                                   const QDateTime &source_time,
                                                   const QString &fixed_frame, double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, qmlToRos2Time( target_time ),
                                                          source_frame, qmlToRos2Time( source_time ),
                                                          fixed_frame, timeout );
}

QVariant TfTransformListenerWrapper::canTransform( const QString &target_frame,
                                                   const Time &target_time,
                                                   const QString &source_frame,
                                                   const Time &source_time,
                                                   const QString &fixed_frame, double timeout ) const
{
  return TfTransformListener::getInstance().canTransform( target_frame, target_time.getTime(),
                                                          source_frame, source_time.getTime(),
                                                          fixed_frame, timeout );
}

QString TfTransformListenerWrapper::getFrameAuthority( const QString &frame_id ) const
{
  return TfTransformListener::getInstance().getFrameAuthority( frame_id );
}

QVariant TfTransformListenerWrapper::getFrame( const QString &frame_id ) const
{
  return TfTransformListener::getInstance().getFrame( frame_id );
}

QVariantList TfTransformListenerWrapper::getAllFrames() const
{
  return TfTransformListener::getInstance().getAllFrames();
}

double TfTransformListenerWrapper::getTransformAge( const QString &frame_id ) const
{
  return TfTransformListener::getInstance().getTransformAge( frame_id );
}
} // namespace qml6_ros2_plugin
