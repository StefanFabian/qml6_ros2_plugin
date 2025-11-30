// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml6_ros2_plugin/image_transport_subscription.hpp"
#include "./logging.hpp"
#include "qml6_ros2_plugin/image_transport_manager.hpp"
#include "qml6_ros2_plugin/ros2.hpp"

namespace qml6_ros2_plugin
{

ImageTransportSubscription::ImageTransportSubscription( QString topic, quint32 queue_size )
    : topic_( std::move( topic ) ), default_transport_( "compressed" ), clock_( RCL_ROS_TIME ),
      queue_size_( queue_size )
{
  no_image_timer_.setSingleShot( true );
  connect( &no_image_timer_, &QTimer::timeout, this, &ImageTransportSubscription::onNoImageTimeout,
           Qt::AutoConnection );
  initSubscriber();
}

ImageTransportSubscription::ImageTransportSubscription()
    : default_transport_( "compressed" ), clock_( RCL_ROS_TIME ), queue_size_( 1 )
{
  no_image_timer_.setSingleShot( true );
  connect( &no_image_timer_, &QTimer::timeout, this, &ImageTransportSubscription::onNoImageTimeout,
           Qt::AutoConnection );
}

QVideoSink *ImageTransportSubscription::videoSink() const { return sink_; }

void ImageTransportSubscription::setVideoSink( QVideoSink *sink )
{
  if ( sink == sink_ )
    return;
  sink_ = sink;
  if ( sink_ == nullptr && subscribed_ ) {
    shutdownSubscriber();
    return;
  }
  if ( !subscribed_ )
    initSubscriber();
  if ( last_frame_.isValid() )
    presentFrame( last_frame_ );
}

void ImageTransportSubscription::onRos2Initialized() { initSubscriber(); }

void ImageTransportSubscription::onRos2Shutdown() { shutdownSubscriber(); }

void ImageTransportSubscription::initSubscriber()
{
  // This makes sure we lazy subscribe and only subscribe if there is a surface to write to
  if ( sink_ == nullptr || !enabled_ || topic_.isEmpty() )
    return;
  if ( !Ros2Qml::getInstance().isInitialized() )
    return;
  bool was_subscribed = subscribed_;
  if ( subscribed_ ) {
    blockSignals( true );
    shutdownSubscriber();
    blockSignals( false );
  }
  // TODO Transport hints
  const rclcpp::Node::SharedPtr &node = Ros2Qml::getInstance().node();
  image_transport::TransportHints transport_hints( node.get(), default_transport_.toStdString() );
  subscription_ = ImageTransportManager::getInstance().subscribe(
      node, topic_, queue_size_, transport_hints,
      [this]( const QVideoFrame &frame ) { presentFrame( frame ); } );
  subscribed_ = subscription_ != nullptr;
  if ( !was_subscribed )
    emit subscribedChanged();
}

void ImageTransportSubscription::shutdownSubscriber()
{
  if ( !subscribed_ )
    return;
  subscription_.reset();
  subscribed_ = false;
  emit subscribedChanged();
}

void ImageTransportSubscription::onNoImageTimeout()
{
  if ( sink_ == nullptr )
    return;
  int elapsed_time_milliseconds =
      static_cast<int>( ( clock_.now() - last_frame_timestamp_ ).nanoseconds() / 1000000 );

  if ( timeout_ == 0 )
    return;
  if ( elapsed_time_milliseconds < timeout_ ) {
    no_image_timer_.start( timeout_ - elapsed_time_milliseconds );
    return;
  }
  sink_->setVideoFrame( QVideoFrame() );
}

void ImageTransportSubscription::presentFrame( const QVideoFrame &frame )
{
  if ( sink_ == nullptr )
    return;
  last_frame_ = frame;
  sink_->setVideoFrame( frame );
  // Return if this is the last frame of our subscription.
  if ( subscription_ == nullptr )
    return;

  bool network_latency_changed = last_network_latency_ != subscription_->networkLatency();
  bool processing_latency_changed = last_processing_latency_ != subscription_->processingLatency();
  if ( network_latency_changed )
    emit networkLatencyChanged();
  if ( processing_latency_changed )
    emit processingLatencyChanged();
  if ( network_latency_changed || processing_latency_changed )
    emit latencyChanged();
  if ( std::abs( last_framerate_ - subscription_->framerate() ) > 0.1 )
    emit framerateChanged();
  last_framerate_ = subscription_->framerate();
  last_frame_timestamp_ = clock_.now();
  last_network_latency_ = subscription_->networkLatency();
  last_processing_latency_ = subscription_->processingLatency();
  if ( timeout_ != 0 ) {
    no_image_timer_.start( throttle_interval_ + timeout_ );
  }
}

QString ImageTransportSubscription::topic() const
{
  if ( subscription_ )
    return QString::fromStdString( subscription_->getTopic() );
  return topic_;
}

void ImageTransportSubscription::setTopic( const QString &value )
{
  if ( topic_ == value )
    return;
  topic_ = value;
  initSubscriber();
  emit topicChanged();
}

const QString &ImageTransportSubscription::defaultTransport() const { return default_transport_; }

void ImageTransportSubscription::setDefaultTransport( const QString &value )
{
  if ( default_transport_ == value )
    return;
  default_transport_ = value;
  initSubscriber();
  emit defaultTransportChanged();
}

bool ImageTransportSubscription::subscribed() const { return subscribed_; }

int ImageTransportSubscription::timeout() const { return timeout_; }

void ImageTransportSubscription::setTimeout( int value )
{
  timeout_ = value;
  emit timeoutChanged();
}

bool ImageTransportSubscription::enabled() const { return enabled_; }

void ImageTransportSubscription::setEnabled( bool value )
{
  if ( enabled_ == value )
    return;
  enabled_ = value;
  if ( enabled_ )
    initSubscriber();
  else
    shutdownSubscriber();
  emit enabledChanged();
}

double ImageTransportSubscription::framerate() const
{
  return subscription_ == nullptr ? 0 : subscription_->framerate();
}

int ImageTransportSubscription::latency() const
{
  return subscription_ == nullptr ? -1 : subscription_->latency();
}

int ImageTransportSubscription::networkLatency() const
{
  return subscription_ == nullptr ? -1 : subscription_->networkLatency();
}

int ImageTransportSubscription::processingLatency() const
{
  return subscription_ == nullptr ? -1 : subscription_->processingLatency();
}
} // namespace qml6_ros2_plugin
