// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_TF_TRANSFORM_HPP
#define QML_ROS2_PLUGIN_TF_TRANSFORM_HPP

#include <QObject>
#include <QPointer>
#include <QTimer>
#include <QVariantMap>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace qml6_ros2_plugin
{
class TfBuffer;

/*!
 * Represents a tf transform between source and target frame.
 */
class TfTransform : public QObject
{
  Q_OBJECT
  //! The source frame of the tf transform, i.e., the frame where the data originated.
  Q_PROPERTY( QString sourceFrame READ sourceFrame WRITE setSourceFrame NOTIFY sourceFrameChanged )

  //! The target frame of the tf transform, i.e., the frame to which the data should be transformed.
  Q_PROPERTY( QString targetFrame READ targetFrame WRITE setTargetFrame NOTIFY targetFrameChanged )

  //! Whether this tf transform is enabled, i.e., receiving transform updates.
  Q_PROPERTY( bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged )

  //! The last received transform as a geometry_msgs/msg/TransformStamped with an added boolean
  //! valid field and optional error fields. See TfTransformListener::lookUpTransform
  Q_PROPERTY( QVariantMap transform READ message NOTIFY messageChanged )
  //! An alias for transform.
  Q_PROPERTY( QVariantMap message READ message NOTIFY messageChanged )

  //! The translation part of the tf transform as a vector with x, y, z fields. Zero if no valid transform available (yet).
  Q_PROPERTY( QVariant translation READ translation NOTIFY messageChanged )

  //! The rotation part of the tf transform as a quaternion with w, x, y, z fields. Identity if no valid transform available (yet).
  Q_PROPERTY( QVariant rotation READ rotation NOTIFY messageChanged )

  //! The maximum rate in Hz at which tf updates are processed and emitted as changed signals.
  //! Default: 60 Note: The rate can not exceed 1000. Setting to 0 will disable updates.
  Q_PROPERTY( qreal rate READ rate WRITE setRate NOTIFY rateChanged )

  //! Whether the current transform, i.e., the fields message, translation and rotation are valid.
  Q_PROPERTY( bool valid READ valid NOTIFY validChanged )

  //! Optional TfBuffer instance providing the tf buffer. If unset (default), the global
  //! TfTransformListener singleton is used. When set, transforms are sourced from the given
  //! buffer, allowing TfTransform to follow a namespaced tf tree.
  Q_PROPERTY( qml6_ros2_plugin::TfBuffer *buffer READ buffer WRITE setBuffer NOTIFY bufferChanged )
public:
  TfTransform();

  ~TfTransform() override;

  const QString &sourceFrame() const;

  void setSourceFrame( const QString &value );

  const QString &targetFrame() const;

  void setTargetFrame( const QString &targetFrame );

  bool enabled() const;

  void setEnabled( bool value );

  qreal rate() const;

  void setRate( qreal value );

  const QVariantMap &message();

  QVariant translation();

  QVariant rotation();

  bool valid();

  TfBuffer *buffer() const;

  void setBuffer( TfBuffer *value );

signals:

  void sourceFrameChanged();

  void targetFrameChanged();

  void enabledChanged();

  void rateChanged();

  void messageChanged();

  void validChanged();

  void bufferChanged();

protected slots:

  void updateMessage();

protected:
  void subscribe();

  void shutdown();

  QTimer update_timer_;
  QVariantMap message_;
  QString source_frame_;
  QString target_frame_;
  geometry_msgs::msg::TransformStamped last_transform_;
  std::chrono::milliseconds update_interval_;
  QPointer<TfBuffer> buffer_;
  bool subscribed_ = false;
  bool enabled_ = true;
  bool using_singleton_ = false;
};
} // namespace qml6_ros2_plugin

#endif // QML_ROS2_PLUGIN_TF_TRANSFORM_HPP
