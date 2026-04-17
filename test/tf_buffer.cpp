// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "common.hpp"
#include <QCoreApplication>
#include <QSignalSpy>
#include <qml6_ros2_plugin/ros2.hpp>
#include <qml6_ros2_plugin/tf_buffer.hpp>
#include <qml6_ros2_plugin/tf_frame_info.hpp>
#include <qml6_ros2_plugin/tf_transform.hpp>
#include <qml6_ros2_plugin/tf_transform_listener.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <tf2_ros/transform_broadcaster.hpp>

using namespace qml6_ros2_plugin;
using namespace std::chrono_literals;

namespace qml6_ros2_plugin
{
class TfBufferTest
{
public:
  static size_t getCacheSize( const TfBuffer &buffer )
  {
    std::lock_guard<std::mutex> lock( buffer.gid_cache_mutex_ );
    return buffer.gid_cache_.size();
  }

  static void fillCachePastLimit( TfBuffer &buffer, size_t count )
  {
    std::lock_guard<std::mutex> lock( buffer.gid_cache_mutex_ );
    const auto now = std::chrono::steady_clock::now();
    buffer.gid_cache_.clear();
    for ( size_t i = 0; i < count; ++i ) {
      TfBuffer::GidCacheEntry entry;
      entry.gid.fill( 0 );
      entry.gid[i % RMW_GID_STORAGE_SIZE] = static_cast<uint8_t>( i + 1 );
      entry.authority = "/fake_authority_" + std::to_string( i );
      entry.last_seen = now + std::chrono::milliseconds( static_cast<int>( i ) );
      buffer.gid_cache_.push_back( std::move( entry ) );
    }
    buffer.evictCacheIfNeeded();
  }
};
} // namespace qml6_ros2_plugin

static rclcpp::Node::SharedPtr main_node;
static rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;

static void processEvents()
{
  QCoreApplication::processEvents();
  if ( executor )
    executor->spin_some();
}

static bool waitFor( const std::function<bool()> &pred, std::chrono::milliseconds timeout = 2s )
{
  auto start = std::chrono::steady_clock::now();
  while ( ( std::chrono::steady_clock::now() - start ) < timeout ) {
    processEvents();
    if ( pred() )
      return true;
    std::this_thread::sleep_for( 1ms );
  }
  return false;
}

TEST( TfBuffer, namespaceValidation )
{
  TfBuffer buffer;
  QSignalSpy spy( &buffer, SIGNAL( nsChanged() ) );

  buffer.setNs( "/robot1" );
  EXPECT_EQ( buffer.ns(), "/robot1" );
  EXPECT_EQ( spy.count(), 1 );

  buffer.setNs( "/robot2/" );
  EXPECT_EQ( buffer.ns(), "/robot2" );
  EXPECT_EQ( spy.count(), 2 );

  buffer.setNs( "" );
  EXPECT_EQ( buffer.ns(), "" );
  EXPECT_EQ( spy.count(), 3 );

  buffer.setNs( "!!" );
  EXPECT_EQ( buffer.ns(), "" );
  EXPECT_EQ( spy.count(), 3 );
}

TEST( TfBuffer, namespacedSubscription )
{
  TfBuffer buffer;
  buffer.setNs( "/robot1" );

  ASSERT_TRUE( waitFor( [&]() { return buffer.isRosInitialized(); } ) );

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "world";
  t.child_frame_id = "robot1_base";
  t.transform.translation.x = 1.0;
  t.transform.rotation.w = 1.0;

  auto node = Ros2Qml::getInstance().node();
  auto pub = node->create_publisher<tf2_msgs::msg::TFMessage>( "/robot1/tf", 10 );

  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back( t );

  ASSERT_TRUE( waitFor(
      [&]() {
        for ( auto &tr : msg.transforms ) tr.header.stamp = node->now();
        pub->publish( msg );
        auto res = buffer.canTransform( "world", "robot1_base", rclcpp::Time( 0 ) );
        return res.typeId() == QMetaType::Bool && res.toBool();
      },
      10s ) )
      << "Failed to receive namespaced transform";

  QVariantMap result = buffer.lookUpTransform( "world", "robot1_base", rclcpp::Time( 0 ) );
  EXPECT_TRUE( result["valid"].toBool() );
}

TEST( TfBuffer, isolation )
{
  TfBuffer buffer1;
  buffer1.setNs( "/robot1" );
  TfBuffer buffer2;
  buffer2.setNs( "/robot2" );

  ASSERT_TRUE( waitFor( [&]() { return buffer1.isRosInitialized() && buffer2.isRosInitialized(); } ) );

  auto node = Ros2Qml::getInstance().node();
  auto pub1 = node->create_publisher<tf2_msgs::msg::TFMessage>( "/robot1/tf", 10 );
  auto pub2 = node->create_publisher<tf2_msgs::msg::TFMessage>( "/robot2/tf", 10 );

  geometry_msgs::msg::TransformStamped t1, t2;
  t1.header.frame_id = "world1";
  t1.child_frame_id = "base1";
  t1.transform.translation.x = 1.0;
  t1.transform.rotation.w = 1.0;

  t2.header.frame_id = "world2";
  t2.child_frame_id = "base2";
  t2.transform.translation.x = 2.0;
  t2.transform.rotation.w = 1.0;

  tf2_msgs::msg::TFMessage msg1, msg2;
  msg1.transforms.push_back( t1 );
  msg2.transforms.push_back( t2 );

  ASSERT_TRUE( waitFor(
      [&]() {
        for ( auto &tr : msg1.transforms ) tr.header.stamp = node->now();
        for ( auto &tr : msg2.transforms ) tr.header.stamp = node->now();
        pub1->publish( msg1 );
        pub2->publish( msg2 );
        auto res1 = buffer1.canTransform( "world1", "base1", rclcpp::Time( 0 ) );
        auto res2 = buffer2.canTransform( "world2", "base2", rclcpp::Time( 0 ) );
        return res1.typeId() == QMetaType::Bool && res1.toBool() &&
               res2.typeId() == QMetaType::Bool && res2.toBool();
      },
      10s ) )
      << "Failed to receive isolated transforms";

  QVariantMap res1_info = buffer1.lookUpTransform( "world1", "base1", rclcpp::Time( 0 ) );
  EXPECT_TRUE( res1_info["valid"].toBool() )
      << "Buffer1 transform invalid: "
      << buffer1.canTransform( "world1", "base1", rclcpp::Time( 0 ) ).toString().toStdString();
  EXPECT_DOUBLE_EQ( res1_info["transform"].toMap()["translation"].toMap()["x"].toDouble(), 1.0 );

  QVariantMap res2_info = buffer2.lookUpTransform( "world2", "base2", rclcpp::Time( 0 ) );
  EXPECT_TRUE( res2_info["valid"].toBool() )
      << "Buffer2 transform invalid: "
      << buffer2.canTransform( "world2", "base2", rclcpp::Time( 0 ) ).toString().toStdString();
  EXPECT_DOUBLE_EQ( res2_info["transform"].toMap()["translation"].toMap()["x"].toDouble(), 2.0 );
}

TEST( TfBuffer, tfTransformWithBuffer )
{
  TfBuffer buffer;
  buffer.setNs( "/robot1" );

  TfTransform transform;
  transform.setSourceFrame( "base" );
  transform.setTargetFrame( "world" );
  transform.setBuffer( &buffer );

  ASSERT_TRUE( waitFor( [&]() { return buffer.isRosInitialized(); } ) );

  auto node = Ros2Qml::getInstance().node();
  auto pub = node->create_publisher<tf2_msgs::msg::TFMessage>( "/robot1/tf", 10 );

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "world";
  t.child_frame_id = "base";
  t.transform.translation.x = 42.0;
  t.transform.rotation.w = 1.0;

  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back( t );

  ASSERT_TRUE( waitFor(
      [&]() {
        for ( auto &tr : msg.transforms ) tr.header.stamp = node->now();
        pub->publish( msg );
        return transform.valid();
      },
      10s ) )
      << "TfTransform failed to become valid";

  EXPECT_DOUBLE_EQ( transform.translation().toMap()["x"].toDouble(), 42.0 );
}

TEST( TfBuffer, authorityAndCache )
{
  TfBuffer buffer;
  ASSERT_TRUE( waitFor( [&]() { return buffer.isRosInitialized(); } ) );

  auto node = Ros2Qml::getInstance().node();
  auto pub = node->create_publisher<tf2_msgs::msg::TFMessage>( "/tf", 10 );

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "world";
  t.child_frame_id = "base";
  t.transform.translation.x = 1.0;
  t.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back( t );

  ASSERT_TRUE( waitFor(
      [&]() {
        for ( auto &tr : msg.transforms ) tr.header.stamp = node->now();
        pub->publish( msg );
        return TfBufferTest::getCacheSize( buffer ) >= 1;
      },
      10s ) )
      << "Failed to receive message for authority test";

  // 2. Cache Eviction test
  TfBufferTest::fillCachePastLimit( buffer, 40 );
  EXPECT_LE( TfBufferTest::getCacheSize( buffer ), 30u );
}

TEST( TfBuffer, getFrameAuthority )
{
  TfBuffer buffer;
  ASSERT_TRUE( waitFor( [&]() { return buffer.isRosInitialized(); } ) );

  auto node = Ros2Qml::getInstance().node();
  auto pub = node->create_publisher<tf2_msgs::msg::TFMessage>( "/tf", 10 );

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "world";
  t.child_frame_id = "test_frame_auth";
  t.transform.translation.x = 1.0;
  t.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back( t );

  ASSERT_TRUE( waitFor(
      [&]() {
        for ( auto &tr : msg.transforms ) tr.header.stamp = node->now();
        pub->publish( msg );
        return !buffer.getFrameAuthority( "test_frame_auth" ).isEmpty();
      },
      10s ) )
      << "Failed to receive authority for test_frame_auth";

  QString authority = buffer.getFrameAuthority( "test_frame_auth" );
  EXPECT_EQ( authority, "/test_tf_buffer_qml" );
}

TEST( TfBuffer, singletonAuthority )
{
  TfTransformListenerWrapper listener;
  listener.initialize();
  processEvents();

  auto node = Ros2Qml::getInstance().node();
  auto pub = node->create_publisher<tf2_msgs::msg::TFMessage>( "/tf", 10 );

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "world";
  t.child_frame_id = "test_frame_singleton";
  t.transform.translation.x = 1.0;
  t.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back( t );

  ASSERT_TRUE( waitFor(
      [&]() {
        for ( auto &tr : msg.transforms ) tr.header.stamp = node->now();
        pub->publish( msg );
        return !listener.getFrameAuthority( "test_frame_singleton" ).isEmpty();
      },
      10s ) )
      << "Failed to receive authority for test_frame_singleton";

  QString authority = listener.getFrameAuthority( "test_frame_singleton" );
  EXPECT_EQ( authority, "/test_tf_buffer_qml" );
}

TEST( TfBuffer, getFrame )
{
  TfBuffer buffer;
  ASSERT_TRUE( waitFor( [&]() { return buffer.isRosInitialized(); } ) );

  // Unknown frame returns invalid QVariant.
  QVariant unknown = buffer.getFrame( "does_not_exist" );
  EXPECT_FALSE( unknown.isValid() );

  auto node = Ros2Qml::getInstance().node();
  auto pub = node->create_publisher<tf2_msgs::msg::TFMessage>( "/tf", 10 );

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "map";
  t.child_frame_id = "gf_base";
  t.transform.translation.x = 1.5;
  t.transform.translation.y = 2.5;
  t.transform.translation.z = 3.5;
  t.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back( t );

  ASSERT_TRUE( waitFor(
      [&]() {
        for ( auto &tr : msg.transforms ) tr.header.stamp = node->now();
        pub->publish( msg );
        return buffer.getFrame( "gf_base" ).isValid();
      },
      10s ) )
      << "Failed to receive frame info for gf_base";

  QVariant v = buffer.getFrame( "gf_base" );
  ASSERT_TRUE( v.isValid() );
  auto info = v.value<TfFrameInfo>();
  EXPECT_EQ( info.frameId(), "gf_base" );
  EXPECT_EQ( info.parentId(), "map" );
  EXPECT_DOUBLE_EQ( info.translation()["x"].toDouble(), 1.5 );
  EXPECT_DOUBLE_EQ( info.translation()["y"].toDouble(), 2.5 );
  EXPECT_DOUBLE_EQ( info.translation()["z"].toDouble(), 3.5 );
  EXPECT_DOUBLE_EQ( info.rotation()["w"].toDouble(), 1.0 );
  EXPECT_FALSE( info.isStatic() );

  // Parent frame should list gf_base as a child.
  QVariant parent_v = buffer.getFrame( "map" );
  ASSERT_TRUE( parent_v.isValid() );
  auto parent_info = parent_v.value<TfFrameInfo>();
  EXPECT_TRUE( parent_info.children().contains( "gf_base" ) );
}

TEST( TfBuffer, getAllFrames )
{
  TfBuffer buffer;
  buffer.setNs( "/gaf_robot" );
  ASSERT_TRUE( waitFor( [&]() { return buffer.isRosInitialized(); } ) );

  auto node = Ros2Qml::getInstance().node();
  auto pub = node->create_publisher<tf2_msgs::msg::TFMessage>( "/gaf_robot/tf", 10 );

  geometry_msgs::msg::TransformStamped t1, t2;
  t1.header.frame_id = "gaf_world";
  t1.child_frame_id = "gaf_a";
  t1.transform.rotation.w = 1.0;
  t2.header.frame_id = "gaf_a";
  t2.child_frame_id = "gaf_b";
  t2.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back( t1 );
  msg.transforms.push_back( t2 );

  ASSERT_TRUE( waitFor(
      [&]() {
        for ( auto &tr : msg.transforms ) tr.header.stamp = node->now();
        pub->publish( msg );
        return buffer.getAllFrames().size() >= 3;
      },
      10s ) )
      << "Failed to receive all frames";

  QVariantList frames = buffer.getAllFrames();
  // Should have gaf_world, gaf_a, gaf_b.
  QStringList ids;
  for ( const auto &f : frames ) ids.append( f.value<TfFrameInfo>().frameId() );
  EXPECT_TRUE( ids.contains( "gaf_world" ) );
  EXPECT_TRUE( ids.contains( "gaf_a" ) );
  EXPECT_TRUE( ids.contains( "gaf_b" ) );
}

TEST( TfBuffer, getTransformAge )
{
  TfBuffer buffer;
  ASSERT_TRUE( waitFor( [&]() { return buffer.isRosInitialized(); } ) );

  // Unknown frame returns -1.
  EXPECT_DOUBLE_EQ( buffer.getTransformAge( "nonexistent" ), -1.0 );

  auto node = Ros2Qml::getInstance().node();
  auto pub = node->create_publisher<tf2_msgs::msg::TFMessage>( "/tf", 10 );

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "age_parent";
  t.child_frame_id = "age_child";
  t.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back( t );

  ASSERT_TRUE( waitFor(
      [&]() {
        for ( auto &tr : msg.transforms ) tr.header.stamp = node->now();
        pub->publish( msg );
        return buffer.getTransformAge( "age_child" ) >= 0.0;
      },
      10s ) )
      << "Failed to receive transform for age test";

  double age = buffer.getTransformAge( "age_child" );
  EXPECT_GE( age, 0.0 );
  EXPECT_LT( age, 5.0 );

  // Root frame (no parent) returns -1.
  EXPECT_DOUBLE_EQ( buffer.getTransformAge( "age_parent" ), -1.0 );
}

TEST( TfBuffer, staticFrameInfo )
{
  TfBuffer buffer;
  ASSERT_TRUE( waitFor( [&]() { return buffer.isRosInitialized(); } ) );

  auto node = Ros2Qml::getInstance().node();
  auto pub_static = node->create_publisher<tf2_msgs::msg::TFMessage>(
      "/tf_static", rclcpp::QoS( 10 ).transient_local() );

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "sf_parent";
  t.child_frame_id = "sf_child";
  t.header.stamp = node->now();
  t.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back( t );

  ASSERT_TRUE( waitFor(
      [&]() {
        pub_static->publish( msg );
        return buffer.getFrame( "sf_child" ).isValid();
      },
      10s ) )
      << "Failed to receive static transform";

  auto info = buffer.getFrame( "sf_child" ).value<TfFrameInfo>();
  EXPECT_TRUE( info.isStatic() );
}

TEST( TfBuffer, frequency )
{
  TfBuffer buffer;
  buffer.setNs( "/freq_robot" );
  ASSERT_TRUE( waitFor( [&]() { return buffer.isRosInitialized(); } ) );

  auto node = Ros2Qml::getInstance().node();
  auto pub = node->create_publisher<tf2_msgs::msg::TFMessage>( "/freq_robot/tf", 10 );

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "freq_world";
  t.child_frame_id = "freq_base";
  t.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back( t );

  // Publish several messages to build up frequency.
  for ( int i = 0; i < 20; ++i ) {
    for ( auto &tr : msg.transforms ) tr.header.stamp = node->now();
    pub->publish( msg );
    processEvents();
    std::this_thread::sleep_for( 10ms );
  }

  ASSERT_TRUE( waitFor( [&]() { return buffer.getFrame( "freq_base" ).isValid(); }, 5s ) );

  auto info = buffer.getFrame( "freq_base" ).value<TfFrameInfo>();
  // Published at ~100 Hz, expect frequency > 90.
  EXPECT_GT( info.frequency(), 90.0 );

  // After a moment of silence, frequency should drop but not immediately to zero due to decay.
  std::this_thread::sleep_for( 40ms );
  processEvents();
  info = buffer.getFrame( "freq_base" ).value<TfFrameInfo>();
  EXPECT_GT( info.frequency(), 50.0 );

  ASSERT_TRUE( waitFor(
      [&]() {
        if ( !buffer.getFrame( "freq_base" ).isValid() )
          return false;
        return buffer.getFrame( "freq_base" ).value<TfFrameInfo>().frequency() < 0.1;
      },
      4s ) )
      << "Frequency should decay to zero when no transforms are received.";
}

TEST( TfBuffer, singletonGetFrame )
{
  TfTransformListenerWrapper listener;
  listener.initialize();
  processEvents();

  auto node = Ros2Qml::getInstance().node();
  auto pub = node->create_publisher<tf2_msgs::msg::TFMessage>( "/tf", 10 );

  geometry_msgs::msg::TransformStamped t;
  t.header.frame_id = "sgf_world";
  t.child_frame_id = "sgf_base";
  t.transform.translation.x = 7.0;
  t.transform.rotation.w = 1.0;
  tf2_msgs::msg::TFMessage msg;
  msg.transforms.push_back( t );

  ASSERT_TRUE( waitFor(
      [&]() {
        for ( auto &tr : msg.transforms ) tr.header.stamp = node->now();
        pub->publish( msg );
        return listener.getFrame( "sgf_base" ).isValid();
      },
      10s ) )
      << "Singleton failed to receive frame";

  auto info = listener.getFrame( "sgf_base" ).value<TfFrameInfo>();
  EXPECT_EQ( info.frameId(), "sgf_base" );
  EXPECT_EQ( info.parentId(), "sgf_world" );
  EXPECT_DOUBLE_EQ( info.translation()["x"].toDouble(), 7.0 );

  // getAllFrames should contain sgf_base.
  QVariantList all = listener.getAllFrames();
  bool found = false;
  for ( const auto &f : all ) {
    if ( f.value<TfFrameInfo>().frameId() == "sgf_base" ) {
      found = true;
      break;
    }
  }
  EXPECT_TRUE( found );

  // getTransformAge should work through singleton.
  double age = listener.getTransformAge( "sgf_base" );
  EXPECT_GE( age, 0.0 );
  EXPECT_LT( age, 5.0 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  rclcpp::init( argc, argv );
  main_node = rclcpp::Node::make_shared( "test_tf_buffer_node" );
  executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  executor->add_node( main_node );

  Ros2QmlSingletonWrapper wrapper;
  wrapper.init( "test_tf_buffer_qml" );

  int result = RUN_ALL_TESTS();

  wrapper.shutdown();
  if ( executor && main_node )
    executor->remove_node( main_node );
  executor.reset();
  main_node.reset();
  rclcpp::shutdown();
  return result;
}

#include "tf_buffer.moc"
