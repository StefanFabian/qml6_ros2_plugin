// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml6_ros2_plugin/publisher.hpp"
#include "logging.hpp"
#include "qml6_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml6_ros2_plugin/conversion/message_conversions.hpp"
#include "qml6_ros2_plugin/ros2.hpp"

#include <rclcpp/exceptions.hpp>

using namespace ros_babel_fish;
using namespace qml6_ros2_plugin::conversion;

namespace qml6_ros2_plugin
{

Publisher::Publisher( QString topic, QString type, const QoSWrapper &qos )
    : qos_( qos ), type_( std::move( type ) ), topic_( std::move( topic ) ), is_advertised_( false )
{
  std_type_ = type_.toStdString();
  babel_fish_ = BabelFishDispenser::getBabelFish();
  advertise_timer_.setInterval( 50 );
  connect( &advertise_timer_, &QTimer::timeout, this, &Publisher::advertise );

  if ( isRosInitialized() )
    advertise();
}

Publisher::~Publisher() = default;

QString Publisher::topic() const
{
  if ( !publisher_ ) {
    return topic_;
  }
  return QString::fromStdString( publisher_->get_topic_name() );
}

const QString &Publisher::type() const { return type_; }

quint32 Publisher::queueSize() const { return qos_.depth(); }

const QoSWrapper &Publisher::qos() const { return qos_; }

bool Publisher::isAdvertised() const { return is_advertised_; }

unsigned int Publisher::getSubscriptionCount()
{
  return publisher_ == nullptr ? 0 : publisher_->get_subscription_count();
}

bool Publisher::publish( const QVariantMap &msg )
{
  if ( !is_advertised_ )
    return false;
  try {
    ros_babel_fish::CompoundMessage message = babel_fish_.create_message( type_.toStdString() );
    if ( !fillMessage( message, msg ) )
      return false;
    publisher_->publish( message );
    return true;
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to publish message: %s", ex.what() );
  }
  return false;
}

void Publisher::shutdown()
{
  is_shutdown_ = true;
  advertise_timer_.stop();
  publisher_.reset();
  is_advertised_ = false;
}

void Publisher::onRos2Initialized()
{
  if ( !is_advertised_ )
    advertise();
}

void Publisher::onRos2Shutdown() { shutdown(); }

void Publisher::advertise()
{
  if ( is_shutdown_ )
    return;
  if ( is_advertised_ )
    publisher_.reset();
  if ( type_.isEmpty() )
    return;
  if ( topic_.isEmpty() )
    return;

  std::shared_ptr<rclcpp::Node> node = Ros2Qml::getInstance().node();
  if ( node == nullptr )
    return;
  try {
    publisher_ =
        babel_fish_.create_publisher( *node, topic_.toStdString(), std_type_, qos_.rclcppQoS(), {} );
    advertise_timer_.stop();
    is_advertised_ = true;
    emit advertised();
    if ( topic_ != topic() )
      emit topicChanged();
  } catch ( BabelFishException &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to create publisher: %s", ex.what() );
  } catch ( rclcpp::exceptions::RCLError &ex ) {
    // Usually the topic is still used with a different type by a publisher in this application that
    // was not destroyed yet. Retry until it is released.
    QML_ROS2_PLUGIN_WARN_THROTTLE( 5000, "Failed to create publisher on '%s', retrying: %s",
                                   topic_.toStdString().c_str(), ex.what() );
    advertise_timer_.start();
  } catch ( std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to create publisher on '%s': %s", topic_.toStdString().c_str(),
                           ex.what() );
  }
}
} // namespace qml6_ros2_plugin
