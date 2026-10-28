// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "logging.hpp"
#include "qml6_ros2_plugin/ros2.hpp"

#include <rcl_action/graph.h>
#include <rclcpp/exceptions/exceptions.hpp>
#include <rmw/validate_full_topic_name.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>

namespace qml6_ros2_plugin
{

namespace
{
//! Adds /msg/ (or the given type) to datatype if it is missing, e.g., std_msgs/Int32 -> std_msgs/msg/Int32.
std::string toFullyQualifiedDatatype( const std::string &datatype, const std::string &type = "msg" )
{
  std::string result = datatype;
  std::string::size_type sep = result.find( '/' );
  if ( sep == std::string::npos )
    return result;
  if ( sep + type.size() + 1 < result.size() &&
       result.substr( sep + 1, type.size() + 1 ) == type + "/" )
    return result;
  return result.substr( 0, sep ) + "/" + type + result.substr( sep );
}

//! True for the internal topics and services ROS creates to implement an action.
bool isActionInternal( const std::string &name )
{
  return name.find( "/_action/" ) != std::string::npos;
}

QStringList toQStringList( const std::vector<std::string> &values )
{
  QStringList result;
  result.reserve( static_cast<int>( values.size() ) );
  for ( const auto &value : values ) result.append( QString::fromStdString( value ) );
  return result;
}

//! The topics and their types in the graph, or an empty map with a logged error if the query fails.
std::map<std::string, std::vector<std::string>>
topicNamesAndTypes( const std::shared_ptr<rclcpp::Node> &node )
{
  try {
    return node->get_topic_names_and_types();
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to get topic names and types: %s", ex.what() );
    return {};
  }
}

//! The services and their types in the graph, or an empty map with a logged error if the query fails.
std::map<std::string, std::vector<std::string>>
serviceNamesAndTypes( const std::shared_ptr<rclcpp::Node> &node )
{
  try {
    return node->get_service_names_and_types();
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to get service names and types: %s", ex.what() );
    return {};
  }
}

//! The names and namespaces of the nodes in the graph, or an empty list with a logged error if the
//! query fails.
std::vector<std::pair<std::string, std::string>>
nodeNamesAndNamespaces( const std::shared_ptr<rclcpp::Node> &node )
{
  try {
    return node->get_node_graph_interface()->get_node_names_and_namespaces();
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to get node names and namespaces: %s", ex.what() );
    return {};
  }
}

//! Runs an rcl_action names-and-types query and returns the result, freeing the rcl memory.
//! @param query Called with the allocator and the rcl_names_and_types_t to fill.
//! @throws rclcpp::exceptions::RCLError if the query fails.
template<typename Query>
std::map<std::string, std::vector<std::string>> actionNamesAndTypes( Query query )
{
  std::map<std::string, std::vector<std::string>> result;
  rcl_names_and_types_t names_and_types = rcl_get_zero_initialized_names_and_types();
  rcl_allocator_t allocator = rcutils_get_default_allocator();
  if ( rcl_ret_t ret = query( &allocator, &names_and_types ); ret != RCL_RET_OK )
    rclcpp::exceptions::throw_from_rcl_error( ret );
  for ( size_t i = 0; i < names_and_types.names.size; ++i ) {
    const auto &types = names_and_types.types[i];
    std::vector<std::string> result_types;
    result_types.reserve( types.size );
    for ( size_t j = 0; j < types.size; ++j ) result_types.emplace_back( types.data[j] );
    result.emplace( names_and_types.names.data[i], std::move( result_types ) );
  }
  if ( rcl_names_and_types_fini( &names_and_types ) != RCL_RET_OK ) {
    QML_ROS2_PLUGIN_ERROR(
        "Failed to free action names and types: '%s'. Might have lost some memory.",
        rcl_get_error_string().str );
    rcl_reset_error();
  }
  return result;
}

QVariantList
namesAndTypesToList( const std::map<std::string, std::vector<std::string>> &names_and_types )
{
  QVariantList result;
  for ( const auto &[name, types] : names_and_types ) {
    if ( isActionInternal( name ) )
      continue;
    result.append( QVariantMap{ { "name", QString::fromStdString( name ) },
                                { "datatypes", toQStringList( types ) } } );
  }
  return result;
}

QVariantMap nodeEntry( const std::string &name, const std::string &ns )
{
  const std::string full_name = ( ns == "/" ? ns : ns + "/" ) + name;
  return { { "name", QString::fromStdString( name ) },
           { "namespace", QString::fromStdString( ns ) },
           { "fullName", QString::fromStdString( full_name ) } };
}

//! The entry of a node with its interfaces, or an empty map if the node is not in the graph.
QVariantMap nodeInfo( const std::shared_ptr<rclcpp::Node> &node, const std::string &name,
                      const std::string &ns )
{
  // The by-node queries throw if the node is not (or no longer) in the graph, which is a benign
  // race when callers act on a name from getNodeNamesAndNamespaces().
  try {
    const auto graph = node->get_node_graph_interface();
    const rcl_node_t *node_handle = node->get_node_base_interface()->get_rcl_node_handle();
    const auto actionsByNode = [&]( auto query ) {
      return namesAndTypesToList( actionNamesAndTypes( [&]( auto *allocator, auto *output ) {
        return query( node_handle, allocator, name.c_str(), ns.c_str(), output );
      } ) );
    };
    QVariantMap result = nodeEntry( name, ns );
    result.insert( "publishers",
                   namesAndTypesToList( graph->get_publisher_names_and_types_by_node( name, ns ) ) );
    result.insert( "subscriptions",
                   namesAndTypesToList( graph->get_subscriber_names_and_types_by_node( name, ns ) ) );
    result.insert( "serviceServers",
                   namesAndTypesToList( graph->get_service_names_and_types_by_node( name, ns ) ) );
    result.insert( "serviceClients",
                   namesAndTypesToList( graph->get_client_names_and_types_by_node( name, ns ) ) );
    result.insert( "actionServers", actionsByNode( rcl_action_get_server_names_and_types_by_node ) );
    result.insert( "actionClients", actionsByNode( rcl_action_get_client_names_and_types_by_node ) );
    return result;
  } catch ( const std::exception &ex ) {
    const auto *rcl_error = dynamic_cast<const rclcpp::exceptions::RCLError *>( &ex );
    if ( rcl_error == nullptr || rcl_error->ret != RCL_RET_NODE_NAME_NON_EXISTENT ) {
      QML_ROS2_PLUGIN_ERROR( "Failed to get node info for '%s' in namespace '%s': %s", name.c_str(),
                             ns.c_str(), ex.what() );
    }
    return {};
  }
}

//! Logs a warning and returns false if name is not a valid node name or ns not a valid namespace.
bool isValidNodeName( const std::string &name, const std::string &ns )
{
  int result;
  size_t invalid_index;
  if ( rmw_validate_node_name( name.c_str(), &result, &invalid_index ) == RMW_RET_OK &&
       result != RMW_NODE_NAME_VALID ) {
    QML_ROS2_PLUGIN_WARN( "Invalid node name '%s': %s", name.c_str(),
                          rmw_node_name_validation_result_string( result ) );
    return false;
  }
  if ( rmw_validate_namespace( ns.c_str(), &result, &invalid_index ) == RMW_RET_OK &&
       result != RMW_NAMESPACE_VALID ) {
    QML_ROS2_PLUGIN_WARN( "Invalid node namespace '%s': %s", ns.c_str(),
                          rmw_namespace_validation_result_string( result ) );
    return false;
  }
  return true;
}

//! Logs a warning and returns false if topic is not a valid fully qualified topic name.
bool isValidFullTopicName( const std::string &topic )
{
  int result;
  size_t invalid_index;
  if ( rmw_validate_full_topic_name( topic.c_str(), &result, &invalid_index ) == RMW_RET_OK &&
       result != RMW_TOPIC_VALID ) {
    QML_ROS2_PLUGIN_WARN( "Invalid topic name '%s': %s", topic.c_str(),
                          rmw_full_topic_name_validation_result_string( result ) );
    return false;
  }
  return true;
}

QVariantList endpointsInfo( const std::vector<rclcpp::TopicEndpointInfo> &endpoints )
{
  QVariantList result;
  for ( const auto &endpoint : endpoints ) {
    result.append(
        QVariantMap{ { "nodeName", QString::fromStdString( endpoint.node_name() ) },
                     { "nodeNamespace", QString::fromStdString( endpoint.node_namespace() ) },
                     { "datatype", QString::fromStdString( endpoint.topic_type() ) },
                     { "qos", QVariant::fromValue( QoSWrapper( endpoint.qos_profile() ) ) } } );
  }
  return result;
}
} // namespace

QStringList Ros2Qml::queryTopics( const QString &datatype ) const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to query topics before node was initialized!" );
    return {};
  }
  auto topics_and_types = topicNamesAndTypes( node_ );
  QStringList result;
  std::string std_datatype = toFullyQualifiedDatatype( datatype.toStdString() );
  for ( const auto &[topic, types] : topics_and_types ) {
    if ( isActionInternal( topic ) )
      continue;
    if ( !std_datatype.empty() &&
         std::find( types.begin(), types.end(), std_datatype ) == types.end() )
      continue;
    result.append( QString::fromStdString( topic ) );
  }
  return result;
}

QList<TopicInfo> Ros2Qml::queryTopicInfo() const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to query topic info before node was initialized!" );
    return {};
  }
  auto topics_and_types = topicNamesAndTypes( node_ );
  QList<TopicInfo> result;
  for ( const auto &[topic, types] : topics_and_types ) {
    if ( isActionInternal( topic ) )
      continue;
    result.append( { QString::fromStdString( topic ), toQStringList( types ) } );
  }
  return result;
}

QStringList Ros2Qml::queryTopicTypes( const QString &name ) const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to query topic types before node was initialized!" );
    return {};
  }
  if ( name.isEmpty() )
    return {};
  auto topics_and_types = topicNamesAndTypes( node_ );
  std::string std_name = name.toStdString();
  for ( const auto &[topic, types] : topics_and_types ) {
    if ( std_name != topic )
      continue;
    return toQStringList( types );
  }
  return {};
}

QMap<QString, QStringList> Ros2Qml::getTopicNamesAndTypes() const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to get topic names and types before node was initialized!" );
    return {};
  }
  auto topics_and_types = topicNamesAndTypes( node_ );
  QMap<QString, QStringList> result;
  for ( const auto &[topic, types] : topics_and_types ) {
    if ( isActionInternal( topic ) )
      continue;
    result.insert( QString::fromStdString( topic ), toQStringList( types ) );
  }
  return result;
}

QStringList Ros2Qml::queryServices( const QString &datatype ) const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to query services before node was initialized!" );
    return {};
  }
  auto service_names_and_types = serviceNamesAndTypes( node_ );
  QStringList result;
  std::string std_datatype = toFullyQualifiedDatatype( datatype.toStdString(), "srv" );
  for ( const auto &[topic, types] : service_names_and_types ) {
    if ( !std_datatype.empty() &&
         std::find( types.begin(), types.end(), std_datatype ) == types.end() )
      continue;
    result.append( QString::fromStdString( topic ) );
  }
  return result;
}

QMap<QString, QStringList> Ros2Qml::getServiceNamesAndTypes() const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to get service names and types before node was initialized!" );
    return {};
  }
  auto service_names_and_types = serviceNamesAndTypes( node_ );
  QMap<QString, QStringList> result;
  for ( const auto &[service_name, types] : service_names_and_types ) {
    if ( isActionInternal( service_name ) )
      continue;
    result.insert( QString::fromStdString( service_name ), toQStringList( types ) );
  }
  return result;
}

QStringList Ros2Qml::queryActions( const QString &datatype ) const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to query actions before node was initialized!" );
    return {};
  }
  // Get all actions
  QStringList result;
  auto full_datatype =
      QString::fromStdString( toFullyQualifiedDatatype( datatype.toStdString(), "action" ) );
  QMap<QString, QStringList> action_names_and_types = getActionNamesAndTypes();
  for ( auto it = action_names_and_types.begin(); it != action_names_and_types.end(); ++it ) {
    if ( !full_datatype.isEmpty() &&
         std::find( it.value().begin(), it.value().end(), full_datatype ) == it.value().end() )
      continue;
    result.append( it.key() );
  }
  return result;
}

QMap<QString, QStringList> Ros2Qml::getActionNamesAndTypes() const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to get action names and types before node was initialized!" );
    return {};
  }
  const rcl_node_t *node_handle = node_->get_node_base_interface()->get_rcl_node_handle();
  std::map<std::string, std::vector<std::string>> names_and_types;
  try {
    names_and_types = actionNamesAndTypes( [node_handle]( auto *allocator, auto *output ) {
      return rcl_action_get_names_and_types( node_handle, allocator, output );
    } );
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to get action names and types: %s", ex.what() );
    return {};
  }
  QMap<QString, QStringList> result;
  for ( const auto &[name, types] : names_and_types )
    result.insert( QString::fromStdString( name ), toQStringList( types ) );
  return result;
}

QVariantList Ros2Qml::getNodeNamesAndNamespaces() const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to get node names and namespaces before node was initialized!" );
    return {};
  }
  QVariantList result;
  for ( const auto &[name, ns] : nodeNamesAndNamespaces( node_ ) )
    result.append( nodeEntry( name, ns ) );
  return result;
}

QVariantMap Ros2Qml::getNodeInfo( const QString &name, const QString &ns ) const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to get node info before node was initialized!" );
    return {};
  }
  const std::string std_name = name.toStdString();
  const std::string std_ns = ns.toStdString();
  if ( !isValidNodeName( std_name, std_ns ) )
    return {};
  return nodeInfo( node_, std_name, std_ns );
}

QVariantList Ros2Qml::getAllNodeInfo() const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to get all node info before node was initialized!" );
    return {};
  }
  QVariantList result;
  for ( const auto &[name, ns] : nodeNamesAndNamespaces( node_ ) ) {
    // A node that left the graph in the meantime is left out.
    if ( QVariantMap info = nodeInfo( node_, name, ns ); !info.isEmpty() )
      result.append( info );
  }
  return result;
}

QVariantList Ros2Qml::getPublishersInfoByTopic( const QString &topic ) const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to get publishers info before node was initialized!" );
    return {};
  }
  const std::string std_topic = topic.toStdString();
  if ( !isValidFullTopicName( std_topic ) )
    return {};
  try {
    return endpointsInfo( node_->get_publishers_info_by_topic( std_topic ) );
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to get publishers info for '%s': %s", std_topic.c_str(),
                           ex.what() );
    return {};
  }
}

QVariantList Ros2Qml::getSubscriptionsInfoByTopic( const QString &topic ) const
{
  if ( node_ == nullptr ) {
    QML_ROS2_PLUGIN_DEBUG( "Tried to get subscriptions info before node was initialized!" );
    return {};
  }
  const std::string std_topic = topic.toStdString();
  if ( !isValidFullTopicName( std_topic ) )
    return {};
  try {
    return endpointsInfo( node_->get_subscriptions_info_by_topic( std_topic ) );
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to get subscriptions info for '%s': %s", std_topic.c_str(),
                           ex.what() );
    return {};
  }
}
} // namespace qml6_ros2_plugin
