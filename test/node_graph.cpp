// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <qml6_ros2_plugin/ros2.hpp>

#include <ros_babel_fish_test_msgs/action/simple_test.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>

#include <QCoreApplication>
#include <QQmlComponent>
#include <QQmlContext>
#include <QQmlEngine>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace qml6_ros2_plugin;
using namespace std::chrono_literals;

rclcpp::executors::SingleThreadedExecutor::UniquePtr executor;
rclcpp::Node::SharedPtr node;

void processEvents()
{
  QCoreApplication::processEvents();
  executor->spin_some( 1ms );
}

bool waitFor( const std::function<bool()> &pred, std::chrono::milliseconds timeout = 3s )
{
  auto start = std::chrono::steady_clock::now();
  while ( ( std::chrono::steady_clock::now() - start ) < timeout ) {
    processEvents();
    if ( pred() )
      return true;
  }
  return false;
}

namespace
{
//! Finds the entry whose key equals value in a list of maps, or an empty map.
QVariantMap findEntry( const QVariant &list, const QString &value, const QString &key = "name" )
{
  for ( const QVariant &entry : list.toList() ) {
    QVariantMap map = entry.toMap();
    if ( map.value( key ).toString() == value )
      return map;
  }
  return {};
}
} // namespace

TEST( NodeGraph, getNodeNamesAndNamespaces )
{
  Ros2QmlSingletonWrapper wrapper;
  QVariantMap entry;
  ASSERT_TRUE( waitFor( [&]() {
    entry = findEntry( wrapper.getNodeNamesAndNamespaces(), "/node_graph", "fullName" );
    return !entry.isEmpty();
  } ) )
      << "Test node /node_graph not found in node list.";

  EXPECT_EQ( entry.value( "name" ).toString(), "node_graph" );
  EXPECT_EQ( entry.value( "namespace" ).toString(), "/" );
}

TEST( NodeGraph, getNodeInfoPublisherAndService )
{
  auto pub = node->create_publisher<std_msgs::msg::Int32>( "~/info_pub", 10 );
  auto service = node->create_service<std_srvs::srv::Empty>(
      "~/info_srv", []( const std_srvs::srv::Empty::Request::SharedPtr,
                        std_srvs::srv::Empty::Response::SharedPtr ) {} );
  Ros2QmlSingletonWrapper wrapper;

  // Capture the snapshot that satisfies the wait so the assertions below see the same graph state.
  QVariantMap info;
  ASSERT_TRUE( waitFor( [&]() {
    info = wrapper.getNodeInfo( "node_graph", "/" );
    return !findEntry( info.value( "publishers" ), "/node_graph/info_pub" ).isEmpty() &&
           !findEntry( info.value( "serviceServers" ), "/node_graph/info_srv" ).isEmpty();
  } ) )
      << "Publisher /node_graph/info_pub and service /node_graph/info_srv not found.";

  EXPECT_EQ( info.value( "name" ).toString(), "node_graph" );
  EXPECT_EQ( info.value( "namespace" ).toString(), "/" );
  EXPECT_EQ( info.value( "fullName" ).toString(), "/node_graph" );
  EXPECT_EQ(
      findEntry( info.value( "publishers" ), "/node_graph/info_pub" ).value( "datatypes" ).toStringList(),
      QStringList{ "std_msgs/msg/Int32" } );
  EXPECT_EQ( findEntry( info.value( "serviceServers" ), "/node_graph/info_srv" )
                 .value( "datatypes" )
                 .toStringList(),
             QStringList{ "std_srvs/srv/Empty" } );

  EXPECT_TRUE( wrapper.getNodeInfo( "does_not_exist", "/" ).isEmpty() );
  // Invalid names and namespaces are reported and yield an empty map.
  EXPECT_TRUE( wrapper.getNodeInfo( "/node_graph", "/" ).isEmpty() );
  EXPECT_TRUE( wrapper.getNodeInfo( "node_graph", "no_leading_slash" ).isEmpty() );
}

TEST( NodeGraph, getEndpointsInfoByTopic )
{
  // Non-default policies, so the assertions below prove the QoS is read from the graph.
  auto pub = node->create_publisher<std_msgs::msg::Int32>( "~/endpoint_topic",
                                                           rclcpp::QoS( 10 ).transient_local() );
  auto sub = node->create_subscription<std_msgs::msg::Int32>(
      "~/endpoint_topic", rclcpp::QoS( 10 ).best_effort(),
      []( std_msgs::msg::Int32::ConstSharedPtr ) {} );
  Ros2QmlSingletonWrapper wrapper;

  QVariantList publishers;
  QVariantList subscriptions;
  ASSERT_TRUE( waitFor( [&]() {
    publishers = wrapper.getPublishersInfoByTopic( "/node_graph/endpoint_topic" );
    subscriptions = wrapper.getSubscriptionsInfoByTopic( "/node_graph/endpoint_topic" );
    return publishers.size() == 1 && subscriptions.size() == 1;
  } ) )
      << "Publisher and subscription on /node_graph/endpoint_topic not found.";

  const QVariantMap publisher = publishers.first().toMap();
  EXPECT_EQ( publisher.value( "nodeName" ).toString(), "node_graph" );
  EXPECT_EQ( publisher.value( "nodeNamespace" ).toString(), "/" );
  EXPECT_EQ( publisher.value( "datatype" ).toString(), "std_msgs/msg/Int32" );
  const QoSWrapper publisher_qos = publisher.value( "qos" ).value<QoSWrapper>();
  EXPECT_EQ( publisher_qos.reliability(), qos_reliability_policy::Reliable );
  EXPECT_EQ( publisher_qos.durability(), qos_durability_policy::TransientLocal );
  // Endpoint history and depth are RMW dependent, e.g., Fast DDS reports Unknown and 0.
  if ( publisher_qos.history() != qos_history_policy::Unknown ) {
    EXPECT_EQ( publisher_qos.history(), qos_history_policy::KeepLast );
    EXPECT_EQ( publisher_qos.depth(), 10 );
  }
  EXPECT_EQ( subscriptions.first().toMap().value( "qos" ).value<QoSWrapper>().reliability(),
             qos_reliability_policy::BestEffort );

  EXPECT_TRUE( wrapper.getPublishersInfoByTopic( "/node_graph/no_such_topic" ).isEmpty() );
  // Invalid names are reported and yield an empty list instead of throwing into QML.
  EXPECT_TRUE( wrapper.getSubscriptionsInfoByTopic( "invalid topic" ).isEmpty() );
  // Relative names are rejected even though the topic exists.
  EXPECT_TRUE( wrapper.getPublishersInfoByTopic( "node_graph/endpoint_topic" ).isEmpty() );
  EXPECT_TRUE( wrapper.getSubscriptionsInfoByTopic( "~/endpoint_topic" ).isEmpty() );
}

TEST( NodeGraph, getNodeInfoActionServer )
{
  using SimpleTest = ros_babel_fish_test_msgs::action::SimpleTest;
  auto server = rclcpp_action::create_server<SimpleTest>(
      node, "~/info_action",
      []( const rclcpp_action::GoalUUID &, SimpleTest::Goal::ConstSharedPtr ) {
        return rclcpp_action::GoalResponse::REJECT;
      },
      []( const std::shared_ptr<rclcpp_action::ServerGoalHandle<SimpleTest>> ) {
        return rclcpp_action::CancelResponse::REJECT;
      },
      []( const std::shared_ptr<rclcpp_action::ServerGoalHandle<SimpleTest>> ) {} );
  // A plain service so the filtered service list is provably non-empty once the graph converges.
  auto service = node->create_service<std_srvs::srv::Empty>(
      "~/plain_srv", []( const std_srvs::srv::Empty::Request::SharedPtr,
                         std_srvs::srv::Empty::Response::SharedPtr ) {} );
  Ros2QmlSingletonWrapper wrapper;

  // The action's internal services show up as plain services in the unfiltered graph.
  auto graphHasActionInternalService = []() {
    for ( const auto &[name, types] :
          node->get_node_graph_interface()->get_service_names_and_types_by_node( "node_graph", "/" ) ) {
      if ( name.find( "/_action/" ) != std::string::npos )
        return true;
    }
    return false;
  };

  // Wait until the action server, its internal services and the plain service are all in the graph,
  // so the filter assertion below exercises a populated list with internal services to drop.
  QVariantMap info;
  ASSERT_TRUE( waitFor( [&]() {
    if ( !graphHasActionInternalService() )
      return false;
    info = wrapper.getNodeInfo( "node_graph", "/" );
    return !findEntry( info.value( "actionServers" ), "/node_graph/info_action" ).isEmpty() &&
           !findEntry( info.value( "serviceServers" ), "/node_graph/plain_srv" ).isEmpty();
  } ) )
      << "Action server, its internal services and the plain service did not all appear.";

  // The action's internal topics and services exist in the graph but must not leak into any list.
  for ( const char *key : { "serviceServers", "publishers", "subscriptions" } ) {
    for ( const QVariant &entry : info.value( key ).toList() ) {
      EXPECT_FALSE( entry.toMap().value( "name" ).toString().contains( "/_action/" ) )
          << "Action-internal entry leaked into " << key;
    }
  }
}

TEST( NodeGraph, getAllNodeInfo )
{
  using SimpleTest = ros_babel_fish_test_msgs::action::SimpleTest;
  auto other = rclcpp::Node::make_shared( "interfaces", "/ns" );
  auto sub = other->create_subscription<std_msgs::msg::Int32>(
      "~/sub", 10, []( std_msgs::msg::Int32::ConstSharedPtr ) {} );
  auto client = other->create_client<std_srvs::srv::Empty>( "~/client" );
  auto action_client = rclcpp_action::create_client<SimpleTest>( other, "~/action" );
  Ros2QmlSingletonWrapper wrapper;

  QVariantMap entry;
  ASSERT_TRUE( waitFor( [&]() {
    entry = findEntry( wrapper.getAllNodeInfo(), "/ns/interfaces", "fullName" );
    return !findEntry( entry.value( "subscriptions" ), "/ns/interfaces/sub" ).isEmpty() &&
           !findEntry( entry.value( "serviceClients" ), "/ns/interfaces/client" ).isEmpty() &&
           !findEntry( entry.value( "actionClients" ), "/ns/interfaces/action" ).isEmpty();
  } ) )
      << "Interfaces of /ns/interfaces not found.";

  // Same entries as getNodeInfo() for every node.
  EXPECT_TRUE( entry == wrapper.getNodeInfo( "interfaces", "/ns" ) );
  EXPECT_EQ( findEntry( entry.value( "subscriptions" ), "/ns/interfaces/sub" )
                 .value( "datatypes" )
                 .toStringList(),
             QStringList{ "std_msgs/msg/Int32" } );
  for ( const char *key : { "publishers", "subscriptions", "serviceClients" } ) {
    for ( const QVariant &item : entry.value( key ).toList() )
      EXPECT_FALSE( item.toMap().value( "name" ).toString().contains( "/_action/" ) )
          << "Action-internal entry leaked into " << key;
  }
}

TEST( NodeGraph, graphChanged )
{
  Ros2QmlSingletonWrapper wrapper;
  int count = 0;
  QObject::connect( &wrapper, &Ros2QmlSingletonWrapper::graphChanged, [&count]() { ++count; } );
  // Let the changes of the previous tests pass, so the count below is caused by the new publisher.
  waitFor( []() { return false; }, 500ms );
  count = 0;
  auto pub = node->create_publisher<std_msgs::msg::Int32>( "~/graph_changed", 10 );
  EXPECT_TRUE( waitFor( [&count]() { return count > 0; } ) );
}

TEST( NodeGraph, graphChangedQml )
{
  Ros2QmlSingletonWrapper wrapper;
  QQmlEngine engine;
  engine.rootContext()->setContextProperty( "Ros2", &wrapper );
  QQmlComponent component( &engine );
  component.setData( R"(
import QtQml

QtObject {
  property int count: 0
  property Connections connections: Connections {
    target: Ros2
    function onGraphChanged() { ++count }
  }
}
)",
                     QUrl() );
  std::unique_ptr<QObject> obj( component.create() );
  ASSERT_NE( obj, nullptr ) << component.errorString().toStdString();
  // Let the changes of the previous tests pass, so the count below is caused by the new publisher.
  waitFor( []() { return false; }, 500ms );
  obj->setProperty( "count", 0 );
  auto pub = node->create_publisher<std_msgs::msg::Int32>( "~/graph_changed_qml", 10 );
  EXPECT_TRUE( waitFor( [&obj]() { return obj->property( "count" ).toInt() > 0; } ) );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  rclcpp::init( argc, argv );
  node = rclcpp::Node::make_shared( "node_graph",
                                    rclcpp::NodeOptions().use_intra_process_comms( false ) );
  executor = rclcpp::executors::SingleThreadedExecutor::make_unique();
  executor->add_node( node );
  Ros2QmlSingletonWrapper wrapper;
  wrapper.init( "node_graph_qml" );
  int result = RUN_ALL_TESTS();
  executor.reset();
  node.reset();
  wrapper.shutdown();
  rclcpp::shutdown();
  return result;
}
