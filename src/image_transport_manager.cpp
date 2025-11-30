// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml6_ros2_plugin/image_transport_manager.hpp"

#include "./image_conversion.hpp"
#include "./logging.hpp"
#include "./ring_buffer.hpp"
#include "./rolling_average.hpp"
#include "qml6_ros2_plugin/qobject_ros2.hpp"

#include <QTimer>
#include <QVideoFrame>
#include <mutex>
#include <set>
#include <thread>

using namespace std::chrono_literals;

namespace qml6_ros2_plugin
{

struct ImageTransportManager::SubscriptionManager {
  explicit SubscriptionManager( const rclcpp::Node::SharedPtr &node )
  {
    transport = std::make_unique<image_transport::ImageTransport>( node );
  }

  std::vector<std::shared_ptr<Subscription>> subscriptions;
  std::unique_ptr<image_transport::ImageTransport> transport;
};

class ImageTransportManager::Subscription final : public QObject
{
  Q_OBJECT
public:
  image_transport::TransportHints hints;
  std::shared_ptr<SubscriptionManager> subscription_manager;
  std::string topic;
  ImageTransportManager *manager = nullptr;
  quint32 queue_size = 0;
  std::thread subscribe_thread;
  std::future<void> subscribe_future_;
  // Use a timer to unsubscribe to avoid unsubscribing and resubscribing in quick succession
  QTimer unsubscribe_timer;

  explicit Subscription( image_transport::TransportHints hints ) : hints( std::move( hints ) )
  {
    unsubscribe_timer.setInterval( 1000 );
    unsubscribe_timer.setSingleShot( true );
    QObject::connect( &unsubscribe_timer, &QTimer::timeout, [this]() {
      std::lock_guard subscriptions_lock( subscriptions_mutex_ );
      if ( subscriptions_.empty() && subscriber_ ) {
        QML_ROS2_PLUGIN_DEBUG(
            "No more subscriptions left for topic '%s'. Shutting down subscriber.", topic.c_str() );
        subscriber_.shutdown();
        std::lock_guard image_lock( image_mutex_ );
        last_image_ = nullptr;
        last_frame_ = QVideoFrame();
      }
    } );
  }

  ~Subscription() override = default;

  void subscribe()
  {
    if ( subscriptions_.empty() )
      return;
    // Make sure we don't subscribe twice in a row due to a race condition
    std::unique_lock lock( subscribe_mutex_ );
    if ( subscribe_future_.valid() &&
         subscribe_future_.wait_for( 0ms ) == std::future_status::timeout ) {
      // Already subscribing
      return;
    }
    // Subscribing on background thread to reduce load on UI thread
    subscribe_future_ = std::async( std::launch::async, [this]() {
      if ( subscriber_ )
        return;
      try {
        auto subscriber = subscription_manager->transport->subscribe(
            topic, queue_size, &Subscription::imageCallback, this, &hints );
        std::lock_guard subscriptions_lock( subscriptions_mutex_ );
        if ( subscriptions_.empty() )
          subscriber.shutdown();
        else
          subscriber_ = std::move( subscriber );
      } catch ( std::exception &ex ) {
        QML_ROS2_PLUGIN_ERROR( "Failed to subscribe to topic '%s' with transport '%s': %s",
                               topic.c_str(), hints.getTransport().c_str(), ex.what() );
      } catch ( ... ) {
        QML_ROS2_PLUGIN_ERROR(
            "Failed to subscribe to topic '%s' with transport '%s': Unknown error", topic.c_str(),
            hints.getTransport().c_str() );
      }
    } );
  }

  void addSubscription( const std::shared_ptr<ImageTransportSubscriptionHandle> &sub )
  {
    std::lock_guard subscriptions_lock( subscriptions_mutex_ );
    subscriptions_.push_back( sub.get() );
    subscription_handles_.push_back( sub );
    // If this was the first subscription, subscribe
    if ( !subscriber_ )
      subscribe();
  }

  void removeSubscription( const ImageTransportSubscriptionHandle *sub )
  {
    std::lock_guard subscriptions_lock( subscriptions_mutex_ );
    auto it = std::find_if(
        subscriptions_.begin(), subscriptions_.end(),
        [sub]( const ImageTransportSubscriptionHandle *handle ) { return handle == sub; } );
    if ( it == subscriptions_.end() ) {
      QML_ROS2_PLUGIN_ERROR(
          "Tried to remove a subscription that was not found! Please file a bug report!" );
      return;
    }
    size_t index = it - subscriptions_.begin();
    subscriptions_.erase( it );
    subscription_handles_.erase( subscription_handles_.begin() + index );
    bool subscriber_active = subscriber_;
    if ( subscriptions_.empty() && subscriber_active ) {
      unsubscribe_timer.start();
    }
  }

  std::string getTopic() const
  {
    const std::string &topic = subscriber_.getTopic();
    const std::string &transport = subscriber_.getTransport();
    if ( topic.size() < transport.size() + 1 ||
         0 != topic.compare( topic.size() - transport.size() - 1, transport.size() + 1,
                             "/" + transport ) )
      return topic;
    return topic.substr( 0, topic.size() - transport.size() - 1 );
  }

private:
  void imageCallback( const sensor_msgs::msg::Image::ConstSharedPtr &image )
  {
    rclcpp::Time received_stamp = clock_.now();

    QVideoFrameFormat::PixelFormat format = getVideoFramePixelFormat( image->encoding );
    QVideoFrame frame;
    if ( format != QVideoFrameFormat::Format_Invalid ) {
      frame = QVideoFrame( QVideoFrameFormat( QSize( image->width, image->height ), format ) );
      if ( !writeImageToVideoFrame( image, frame ) ) {
        frame = QVideoFrame();
      }
    }
    {
      std::lock_guard image_lock( image_mutex_ );
      last_received_stamp_ = received_stamp;
      last_image_ = image;
      last_frame_.swap( frame );
    }
    // Deliver frames on UI thread
    QMetaObject::invokeMethod( this, "imageDelivery", Qt::AutoConnection );
  }

  Q_INVOKABLE void imageDelivery()
  {
    sensor_msgs::msg::Image::ConstSharedPtr image;
    rclcpp::Time received;
    QVideoFrame frame;
    {
      std::lock_guard image_lock( image_mutex_ );
      if ( !last_frame_.isValid() || last_image_ == nullptr )
        return;
      last_frame_.swap( frame );
      image = last_image_;
      received = last_received_stamp_;
    }
    std::vector<std::shared_ptr<ImageTransportSubscriptionHandle>> subscribers;
    {
      // This nested lock makes sure that our destruction of the subscription pointer will not lead to a deadlock
      std::lock_guard subscriptions_lock( subscriptions_mutex_ );
      for ( const auto &sub_weak : subscription_handles_ ) {
        if ( sub_weak.expired() )
          continue;
        subscribers.push_back( sub_weak.lock() );
      }
    }
    const rclcpp::Time &image_stamp = image->header.stamp;
    int network_latency = image_stamp.nanoseconds() != 0
                              ? static_cast<int>( ( received - image_stamp ).seconds() * 1000 )
                              : -1;
    network_latency_average_.add( network_latency );
    auto processing_latency = static_cast<int>( ( clock_.now() - received ).seconds() * 1000 );
    processing_latency_average_.add( processing_latency );
    if ( image_stamp.nanoseconds() != 0 ) {
      image_interval_average_.push_back( image_stamp );
    } else {
      image_interval_average_.push_back( last_received_stamp_ );
    }
    const auto image_stamp_diff =
        ( image_interval_average_.back() - image_interval_average_.front() ).nanoseconds();
    const float framerate =
        image_stamp_diff > 0 ? ( image_interval_average_.size() - 1 ) * 1E9f / image_stamp_diff : 0;
    for ( const auto &sub : subscribers ) {
      if ( sub == nullptr || !sub->callback )
        continue;
      sub->network_latency = network_latency_average_;
      sub->processing_latency = processing_latency_average_;
      sub->framerate_ = std::round( framerate * 10 ) / 10;
      sub->callback( frame );
    }
  }

  std::mutex subscriptions_mutex_;
  std::mutex subscribe_mutex_;
  std::mutex image_mutex_;
  image_transport::Subscriber subscriber_;
  std::vector<ImageTransportSubscriptionHandle *> subscriptions_;
  std::vector<std::weak_ptr<ImageTransportSubscriptionHandle>> subscription_handles_;
  RingBuffer<rclcpp::Time, 10> image_interval_average_;
  RollingAverage<int, 10> network_latency_average_;
  RollingAverage<int, 10> processing_latency_average_;
  rclcpp::Clock clock_ = rclcpp::Clock( RCL_ROS_TIME );
  rclcpp::Time last_received_stamp_;
  sensor_msgs::msg::Image::ConstSharedPtr last_image_;
  QVideoFrame last_frame_;
};

ImageTransportSubscriptionHandle::~ImageTransportSubscriptionHandle()
{
  try {
    subscription->removeSubscription( this );
  } catch ( std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Error while removing subscription: %s", ex.what() );
  }
}

std::string ImageTransportSubscriptionHandle::getTopic() const { return subscription->getTopic(); }

int ImageTransportSubscriptionHandle::latency() const
{
  return network_latency + processing_latency;
}

int ImageTransportSubscriptionHandle::networkLatency() const { return network_latency; }

int ImageTransportSubscriptionHandle::processingLatency() const { return processing_latency; }

double ImageTransportSubscriptionHandle::framerate() const { return framerate_; }

ImageTransportManager::ImageTransportManager() = default;

ImageTransportManager &ImageTransportManager::getInstance()
{
  static ImageTransportManager manager;
  return manager;
}

std::shared_ptr<ImageTransportSubscriptionHandle>
ImageTransportManager::subscribe( const rclcpp::Node::SharedPtr &node, const QString &qtopic,
                                  quint32 queue_size,
                                  const image_transport::TransportHints &transport_hints,
                                  const std::function<void( const QVideoFrame & )> &callback )
{
  if ( subscription_manager_ == nullptr ) {
    subscription_manager_ = std::make_shared<SubscriptionManager>( node );
  }
  std::string topic = qtopic.toStdString();
  std::vector<std::shared_ptr<Subscription>> &subscriptions = subscription_manager_->subscriptions;
  size_t i = 0;
  for ( ; i < subscriptions.size(); ++i ) {
    if ( subscriptions[i]->topic == topic && subscriptions[i]->queue_size == queue_size &&
         subscriptions[i]->hints.getTransport() == transport_hints.getTransport() )
      break; // We could also compare transport type and hints
  }
  auto handle = std::make_shared<ImageTransportSubscriptionHandle>();
  handle->callback = callback;
  if ( i == subscriptions.size() ) {
    auto sub = std::make_shared<Subscription>( transport_hints );
    sub->manager = this;
    sub->subscription_manager = subscription_manager_;
    sub->topic = topic;
    sub->queue_size = queue_size;
    subscriptions.emplace_back( sub );
    QML_ROS2_PLUGIN_DEBUG( "Subscribed to '%s' with transport '%s'.", topic.c_str(),
                           transport_hints.getTransport().c_str() );
  }
  handle->subscription = subscriptions[i];
  subscriptions[i]->addSubscription( handle );
  return handle;
}
} // namespace qml6_ros2_plugin

#include "image_transport_manager.moc"
