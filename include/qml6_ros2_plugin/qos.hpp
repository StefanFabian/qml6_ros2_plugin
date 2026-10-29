// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_QOS_HPP
#define QML_ROS2_PLUGIN_QOS_HPP

#include <QObject>
#include <QString>

#include <rclcpp/qos.hpp>

namespace qml6_ros2_plugin
{

namespace qos_reliability_policy
{
Q_NAMESPACE
//! Mirrors rclcpp::ReliabilityPolicy.
enum ReliabilityPolicy {
  BestEffort = RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
  Reliable = RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  SystemDefault = RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
  BestAvailable = RMW_QOS_POLICY_RELIABILITY_BEST_AVAILABLE,
  Unknown = RMW_QOS_POLICY_RELIABILITY_UNKNOWN
};
Q_ENUM_NS( ReliabilityPolicy )
} // namespace qos_reliability_policy

namespace qos_durability_policy
{
Q_NAMESPACE
//! Mirrors rclcpp::DurabilityPolicy.
enum DurabilityPolicy {
  Volatile = RMW_QOS_POLICY_DURABILITY_VOLATILE,
  TransientLocal = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
  SystemDefault = RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
  BestAvailable = RMW_QOS_POLICY_DURABILITY_BEST_AVAILABLE,
  Unknown = RMW_QOS_POLICY_DURABILITY_UNKNOWN
};
Q_ENUM_NS( DurabilityPolicy )
} // namespace qos_durability_policy

namespace qos_history_policy
{
Q_NAMESPACE
//! Mirrors rclcpp::HistoryPolicy.
enum HistoryPolicy {
  KeepLast = RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  KeepAll = RMW_QOS_POLICY_HISTORY_KEEP_ALL,
  SystemDefault = RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
  Unknown = RMW_QOS_POLICY_HISTORY_UNKNOWN
};
Q_ENUM_NS( HistoryPolicy )
} // namespace qos_history_policy

//! @brief Wrapper to enable setting and reading QoS settings in QML.
//! Defaults to 1 depth, best effort reliability and volatile durability.
class QoSWrapper
{
  Q_GADGET
  //! The reliability policy. See ReliabilityPolicy. (readonly)
  Q_PROPERTY( qml6_ros2_plugin::qos_reliability_policy::ReliabilityPolicy reliability READ reliability )
  //! The durability policy. See DurabilityPolicy. (readonly)
  Q_PROPERTY( qml6_ros2_plugin::qos_durability_policy::DurabilityPolicy durability READ durability )
  //! The history policy. See HistoryPolicy. (readonly)
  Q_PROPERTY( qml6_ros2_plugin::qos_history_policy::HistoryPolicy history READ history )
  //! The history depth.
  //! If the history policy is KeepLast, this is the number of messages to keep. (readonly)
  Q_PROPERTY( int depth READ depth )
public:
  QoSWrapper();
  explicit QoSWrapper( rclcpp::QoS qos );

  //! @brief Sets the reliability policy to reliable. Returns the QoSWrapper for chaining.
  Q_INVOKABLE qml6_ros2_plugin::QoSWrapper reliable();

  //! @brief Sets the reliability policy to best effort. Returns the QoSWrapper for chaining.
  Q_INVOKABLE qml6_ros2_plugin::QoSWrapper best_effort();

  //! @brief Sets the durability policy to volatile. Returns the QoSWrapper for chaining.
  Q_INVOKABLE qml6_ros2_plugin::QoSWrapper durability_volatile();

  //! @brief Sets the durability policy to transient local. Returns the QoSWrapper for chaining.
  Q_INVOKABLE qml6_ros2_plugin::QoSWrapper transient_local();

  //! @brief Sets the history policy to keep all messages. Returns the QoSWrapper for chaining.
  Q_INVOKABLE qml6_ros2_plugin::QoSWrapper keep_all();

  //! @brief Sets the history policy to keep last messages. Returns the QoSWrapper for chaining.
  //! @param depth The number of messages to keep.
  Q_INVOKABLE qml6_ros2_plugin::QoSWrapper keep_last( int depth );

  qos_reliability_policy::ReliabilityPolicy reliability() const;

  qos_durability_policy::DurabilityPolicy durability() const;

  qos_history_policy::HistoryPolicy history() const;

  int depth() const;

  const rclcpp::QoS &rclcppQoS() const { return qos_; }

  std::string toString() const;

private:
  rclcpp::QoS qos_;
};

} // namespace qml6_ros2_plugin

Q_DECLARE_METATYPE( qml6_ros2_plugin::QoSWrapper )

#endif // QML_ROS2_PLUGIN_QOS_HPP
