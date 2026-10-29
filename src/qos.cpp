// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml6_ros2_plugin/qos.hpp"

#include <QMetaEnum>

namespace qml6_ros2_plugin
{
QoSWrapper::QoSWrapper() : qos_( 1 )
{
  qos_.best_effort();
  qos_.durability_volatile();
}

QoSWrapper::QoSWrapper( rclcpp::QoS qos ) : qos_( qos ) { }

QoSWrapper QoSWrapper::reliable()
{
  qos_.reliable();
  return *this;
}

QoSWrapper QoSWrapper::best_effort()
{
  qos_.best_effort();
  return *this;
}

QoSWrapper QoSWrapper::durability_volatile()
{
  qos_.durability_volatile();
  return *this;
}

QoSWrapper QoSWrapper::transient_local()
{
  qos_.transient_local();
  return *this;
}

QoSWrapper QoSWrapper::keep_all()
{
  qos_.keep_all();
  return *this;
}

qml6_ros2_plugin::QoSWrapper QoSWrapper::keep_last( int depth )
{
  qos_.keep_last( depth );
  return *this;
}

qos_reliability_policy::ReliabilityPolicy QoSWrapper::reliability() const
{
  return static_cast<qos_reliability_policy::ReliabilityPolicy>( qos_.reliability() );
}

qos_durability_policy::DurabilityPolicy QoSWrapper::durability() const
{
  return static_cast<qos_durability_policy::DurabilityPolicy>( qos_.durability() );
}

qos_history_policy::HistoryPolicy QoSWrapper::history() const
{
  return static_cast<qos_history_policy::HistoryPolicy>( qos_.history() );
}

int QoSWrapper::depth() const { return qos_.depth(); }

namespace
{
template<typename Enum>
std::string to_string( Enum value )
{
  const char *key = QMetaEnum::fromType<Enum>().valueToKey( value );
  return key != nullptr ? key : "Invalid";
}
} // namespace

std::string QoSWrapper::toString() const
{
  std::string result = "QoS(";
  result += "reliability: " + to_string( reliability() ) + ", ";
  result += "durability: " + to_string( durability() ) + ", ";
  result += "history: " + to_string( history() ) + ", ";
  result += "depth: " + std::to_string( depth() ) + ")";
  return result;
}
} // namespace qml6_ros2_plugin
