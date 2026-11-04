// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_PUBLISHER_HPP
#define QML_ROS2_PLUGIN_PUBLISHER_HPP

#include "qml6_ros2_plugin/qobject_ros2.hpp"
#include "qml6_ros2_plugin/qos.hpp"
#include <QMap>
#include <QTimer>
#include <QVariant>

#include <ros_babel_fish/babel_fish.hpp>

namespace qml6_ros2_plugin
{

class Publisher : public QObjectRos2
{
  Q_OBJECT
  //! The type of the published messages, e.g., geometry_msgs/Pose. (readonly)
  Q_PROPERTY( QString type READ type )
  //! The topic this Publisher publishes messages on. This property is only valid if the publisher is already advertised! (readonly)
  Q_PROPERTY( QString topic READ topic NOTIFY topicChanged )
  //! The queue size of this Publisher. This is the depth if the history policy of the QoS is set to keep_last. (readonly)
  Q_PROPERTY( quint32 queueSize READ queueSize )
  //! The Quality of Service settings of this publisher.
  Q_PROPERTY( QoSWrapper qos READ qos )
  //! Whether or not this publisher has advertised its existence on its topic.
  //! Reasons for not being advertised include ROS not being initialized yet. (readonly)
  Q_PROPERTY( bool isAdvertised READ isAdvertised NOTIFY advertised )
public:
  Publisher( QString topic, QString type, const QoSWrapper &qos );

  ~Publisher() override;

  QString topic() const;

  const QString &type() const;

  quint32 queueSize() const;

  const QoSWrapper &qos() const;

  bool isAdvertised() const;

  //! @return The number of subscribers currently connected to this Publisher.
  Q_INVOKABLE unsigned int getSubscriptionCount();

  /*!
   * Sends a message to subscribers currently connected to this Publisher.
   * @param msg The message that is published.
   * @return True if the message was sent successfully, false otherwise.
   */
  Q_INVOKABLE bool publish( const QVariantMap &msg );

  /*!
   * Immediately releases the underlying ROS publisher. The Publisher can not be used afterwards.
   * Use this when replacing a Publisher, e.g., to change the message type on the same topic, since
   * a Publisher otherwise lives until it is garbage collected and blocks other message types on
   * its topic until then.
   */
  Q_INVOKABLE void shutdown();

signals:

  /*!
   * Fired once this Publisher was advertised.
   * This is either done at construction, immediately after ROS is initialized or, if the topic is
   * still blocked by a publisher with a different type in this application, once it was released.
   * Since this is only fired once, you should check if the Publisher is already advertised using the isAdvertised property.
   */
  void advertised();

  //! Fired if the topic changed after creating the publisher.
  void topicChanged();

protected:
  void onRos2Initialized() override;

  void onRos2Shutdown() override;

  void advertise();

  QTimer advertise_timer_;
  ros_babel_fish::BabelFish babel_fish_;
  ros_babel_fish::BabelFishPublisher::SharedPtr publisher_;
  QoSWrapper qos_;

  QString type_;
  std::string std_type_;
  QString topic_;
  bool is_advertised_;
  bool is_shutdown_ = false;
};
} // namespace qml6_ros2_plugin

#endif // QML_ROS2_PLUGIN_PUBLISHER_HPP
