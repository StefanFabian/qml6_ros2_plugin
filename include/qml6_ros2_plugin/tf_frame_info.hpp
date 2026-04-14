// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_TF_FRAME_INFO_HPP
#define QML_ROS2_PLUGIN_TF_FRAME_INFO_HPP

#include <QMetaType>
#include <QString>
#include <QStringList>
#include <QVariantMap>
#include <QtQmlIntegration/qqmlintegration.h>

namespace qml6_ros2_plugin
{

class TfFrameInfo
{
  Q_GADGET
  QML_VALUE_TYPE( tfframeinfo )
  QML_UNCREATABLE( "TfFrameInfo is a value type that can only be created by the TfBuffer class." )

  //! The frame ID of this frame.
  Q_PROPERTY( QString frameId READ frameId )
  //! The parent frame ID, or empty string if this is a root frame.
  Q_PROPERTY( QString parentId READ parentId )
  //! The authority (node name) that provided the latest transform for this frame.
  Q_PROPERTY( QString authority READ authority )
  //! The translation from parent to this frame as a map with x, y, z fields.
  Q_PROPERTY( QVariantMap translation READ translation )
  //! The rotation from parent to this frame as a quaternion map with w, x, y, z fields.
  Q_PROPERTY( QVariantMap rotation READ rotation )
  //! Whether the transform was received on the static topic (/tf_static).
  Q_PROPERTY( bool isStatic READ isStatic )
  //! The list of direct child frame IDs.
  Q_PROPERTY( QStringList children READ children )
  //! The publishing frequency in Hz, computed from a sliding window of recent messages.
  Q_PROPERTY( double frequency READ frequency )
public:
  TfFrameInfo() = default;

  TfFrameInfo( QString frame_id, QString parent_id, QString authority, QVariantMap translation,
               QVariantMap rotation, bool is_static, QStringList children, double frequency )
      : frame_id_( std::move( frame_id ) ), parent_id_( std::move( parent_id ) ),
        authority_( std::move( authority ) ), translation_( std::move( translation ) ),
        rotation_( std::move( rotation ) ), is_static_( is_static ),
        children_( std::move( children ) ), frequency_( frequency )
  {
  }

  const QString &frameId() const { return frame_id_; }

  const QString &parentId() const { return parent_id_; }

  const QString &authority() const { return authority_; }

  const QVariantMap &translation() const { return translation_; }

  const QVariantMap &rotation() const { return rotation_; }

  bool isStatic() const { return is_static_; }

  const QStringList &children() const { return children_; }

  double frequency() const { return frequency_; }

private:
  QString frame_id_;
  QString parent_id_;
  QString authority_;
  QVariantMap translation_;
  QVariantMap rotation_;
  bool is_static_ = false;
  QStringList children_;
  double frequency_ = 0.0;
};
} // namespace qml6_ros2_plugin

Q_DECLARE_METATYPE( qml6_ros2_plugin::TfFrameInfo )

#endif // QML_ROS2_PLUGIN_TF_FRAME_INFO_HPP
