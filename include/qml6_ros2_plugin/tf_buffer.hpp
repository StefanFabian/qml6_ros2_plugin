// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_TF_BUFFER_HPP
#define QML_ROS2_PLUGIN_TF_BUFFER_HPP

#include "qml6_ros2_plugin/internal/window_rate_tracker.hpp"
#include "qml6_ros2_plugin/qobject_ros2.hpp"
#include "qml6_ros2_plugin/time.hpp"

#include <QDateTime>
#include <QPointer>
#include <QVariant>
#include <QVariantMap>
#include <QtQmlIntegration/qqmlintegration.h>
#include <array>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/transform.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.hpp>

namespace qml6_ros2_plugin
{

class TfFrameInfo;

/*!
 * A tf buffer that subscribes to a (optionally namespaced) tf topic pair.
 *
 * Unlike the global TfTransformListener (which is backed by tf2_ros::TransformListener and can
 * therefore only listen on /tf and /tf_static), this element subscribes directly to
 * <namespace>/tf and <namespace>/tf_static on the shared plugin node and feeds the received
 * transforms into its own tf2_ros::Buffer. This enables multi-robot setups where each robot
 * publishes its own tf tree under a namespace.
 *
 * Provides the same canTransform/lookUpTransform QML API as the global Ros2.TfTransformListener.
 * Instances can be passed to TfTransform via its buffer property.
 */
class TfBuffer : public QObjectRos2
{
  Q_OBJECT
  QML_ELEMENT
  //! The ROS2 namespace whose /tf and /tf_static topics this buffer subscribes to.
  //! An empty namespace (default) subscribes to /tf and /tf_static, equivalent to the global
  //! TfTransformListener.
  Q_PROPERTY( QString ns READ ns WRITE setNs NOTIFY nsChanged )
public:
  explicit TfBuffer( QObject *parent = nullptr );

  ~TfBuffer() override;

  QString ns() const;

  void setNs( const QString &ns );

  Q_INVOKABLE void clear();

  //! Non-QML accessor used by TfTransform. May return nullptr until ROS2 is initialized.
  tf2_ros::Buffer *buffer();

  //! @copydoc TfTransformListener::canTransform(const QString &, const QString &, const rclcpp::Time &, double) const
  QVariant canTransform( const QString &target_frame, const QString &source_frame,
                         const rclcpp::Time &time, double timeout = 0 ) const;

  //! @copydoc TfTransformListener::canTransform(const QString &, const rclcpp::Time &, const QString &, const rclcpp::Time &, const QString &, double) const
  QVariant canTransform( const QString &target_frame, const rclcpp::Time &target_time,
                         const QString &source_frame, const rclcpp::Time &source_time,
                         const QString &fixed_frame, double timeout = 0 ) const;

  //! @copydoc TfTransformListener::lookUpTransform(const QString &, const QString &, const rclcpp::Time &, double) const
  QVariantMap lookUpTransform( const QString &target_frame, const QString &source_frame,
                               const rclcpp::Time &time, double timeout = 0 ) const;

  //! @copydoc TfTransformListener::lookUpTransform(const QString &, const rclcpp::Time &, const QString &, const rclcpp::Time &, const QString &, double) const
  QVariantMap lookUpTransform( const QString &target_frame, const rclcpp::Time &target_time,
                               const QString &source_frame, const rclcpp::Time &source_time,
                               const QString &fixed_frame, double timeout = 0 ) const;

  // Q_INVOKABLE overloads mirror TfTransformListenerWrapper one-for-one.
  Q_INVOKABLE QVariant canTransform( const QString &target_frame, const QString &source_frame,
                                     const QDateTime &time, double timeout = 0 ) const;

  Q_INVOKABLE QVariant
  canTransform( const QString &target_frame, const QString &source_frame,
                const qml6_ros2_plugin::Time &time = qml6_ros2_plugin::Time( rclcpp::Time( 0 ) ),
                double timeout = 0 ) const;

  Q_INVOKABLE QVariant canTransform( const QString &target_frame, const QDateTime &target_time,
                                     const QString &source_frame, const QDateTime &source_time,
                                     const QString &fixed_frame, double timeout = 0 ) const;

  Q_INVOKABLE QVariant canTransform( const QString &target_frame,
                                     const qml6_ros2_plugin::Time &target_time,
                                     const QString &source_frame,
                                     const qml6_ros2_plugin::Time &source_time,
                                     const QString &fixed_frame, double timeout = 0 ) const;

  Q_INVOKABLE QVariantMap lookUpTransform( const QString &target_frame, const QString &source_frame,
                                           const QDateTime &time, double timeout = 0 ) const;

  Q_INVOKABLE QVariantMap
  lookUpTransform( const QString &target_frame, const QString &source_frame,
                   const qml6_ros2_plugin::Time &time = qml6_ros2_plugin::Time( rclcpp::Time( 0 ) ),
                   double timeout = 0 ) const;

  Q_INVOKABLE QVariantMap lookUpTransform( const QString &target_frame, const QDateTime &target_time,
                                           const QString &source_frame, const QDateTime &source_time,
                                           const QString &fixed_frame, double timeout = 0 ) const;

  Q_INVOKABLE QVariantMap lookUpTransform( const QString &target_frame,
                                           const qml6_ros2_plugin::Time &target_time,
                                           const QString &source_frame,
                                           const qml6_ros2_plugin::Time &source_time,
                                           const QString &fixed_frame, double timeout = 0 ) const;

  /**
   * @brief Returns the authority (node name) that provided the latest transform for the given frame.
   *
   * In TF, the authority for a frame is defined as the node that publishes the transform where
   * the frame is the child_frame_id.
   *
   * @param frame_id The frame ID to look up.
   * @return The node name of the authority, or an empty string if not found.
   */
  Q_INVOKABLE QString getFrameAuthority( const QString &frame_id ) const;

  /**
   * @brief Returns information about a single frame.
   *
   * @param frame_id The frame ID to look up.
   * @return A TfFrameInfo wrapped in QVariant, or an invalid QVariant if the frame is not known.
   */
  Q_INVOKABLE QVariant getFrame( const QString &frame_id ) const;

  /**
   * @brief Returns information about all known frames.
   *
   * @return A list of TfFrameInfo objects for every frame in the buffer.
   */
  Q_INVOKABLE QVariantList getAllFrames() const;

  /**
   * @brief Returns the age of the latest transform for the given frame.
   *
   * The age is computed as the difference between the current ROS time and the timestamp of the
   * most recent transform received for the frame.
   *
   * @param frame_id The frame ID to look up.
   * @return The age in seconds, or -1.0 if the frame is not known or has no parent.
   */
  Q_INVOKABLE double getTransformAge( const QString &frame_id ) const;

signals:

  void nsChanged();

protected:
  void onRos2Initialized() override;

  void onRos2Shutdown() override;

private:
  static constexpr std::size_t kFrequencyRingSize = 30;

  class CallbackActivityGuard
  {
  public:
    explicit CallbackActivityGuard( TfBuffer *buffer );

    ~CallbackActivityGuard();

    bool active() const;

  private:
    TfBuffer *buffer_;
    bool active_;
  };

  struct GidCacheEntry {
    std::array<uint8_t, RMW_GID_STORAGE_SIZE> gid{};
    std::string authority;
    std::chrono::steady_clock::time_point last_seen;
  };

  struct FrameState {
    std::string frame_id;
    std::string parent_id;
    std::string authority;
    bool is_static = false;
    geometry_msgs::msg::Transform transform;
    std::vector<std::string> children;
    internal::WindowRateTracker<kFrequencyRingSize> frequency_tracker;
    rclcpp::Time last_stamp{ 0, 0, RCL_ROS_TIME };
  };

  void subscribeTopics();

  void unsubscribeTopics();

  void tfCallback( tf2_msgs::msg::TFMessage::ConstSharedPtr msg, const rclcpp::MessageInfo &info,
                   bool is_static );

  std::string resolveAuthority( const std::array<uint8_t, RMW_GID_STORAGE_SIZE> &gid,
                                const std::string &topic );

  void evictCacheIfNeeded();

  static TfFrameInfo frameStateToInfo( const FrameState &state,
                                       std::chrono::steady_clock::time_point now );

  QString namespace_;
  std::unique_ptr<tf2_ros::Buffer> buffer_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_static_sub_;
  std::vector<GidCacheEntry> gid_cache_;
  mutable std::mutex gid_cache_mutex_;
  std::string tf_topic_;
  std::string tf_static_topic_;
  std::unordered_map<std::string, FrameState> frame_states_;
  mutable std::mutex authority_mutex_;
  mutable std::mutex callback_state_mutex_;
  std::condition_variable callback_state_cv_;
  bool accepting_callbacks_ = false;
  std::size_t active_callback_count_ = 0;

  friend class TfBufferTest;
};
} // namespace qml6_ros2_plugin

#endif // QML_ROS2_PLUGIN_TF_BUFFER_HPP
