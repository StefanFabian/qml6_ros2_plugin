// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml6_ros2_plugin/ros2.hpp"
#include "logging.hpp"
#include "qml6_ros2_plugin/action_client.hpp"
#include "qml6_ros2_plugin/babel_fish_dispenser.hpp"
#include "qml6_ros2_plugin/conversion/message_conversions.hpp"
#include "qml6_ros2_plugin/publisher.hpp"
#include "qml6_ros2_plugin/qos.hpp"
#include "qml6_ros2_plugin/service_client.hpp"
#include "qml6_ros2_plugin/subscription.hpp"

#include <QCoreApplication>
#include <QHostInfo>
#include <QJSEngine>
#include <rcl/validate_topic_name.h>
#include <thread>

namespace qml6_ros2_plugin
{

Ros2Qml &Ros2Qml::getInstance()
{
  static Ros2Qml instance;
  return instance;
}

Ros2Qml::Ros2Qml() : count_wrappers( 0 )
{
  babel_fish_ = BabelFishDispenser::getBabelFish();
  graph_timer_.setInterval( 100 );
  QObject::connect( &graph_timer_, &QTimer::timeout, this, [this]() {
    if ( graph_event_ != nullptr && graph_event_->check_and_clear() )
      emit graphChanged();
  } );

  auto *core_app = QCoreApplication::instance();
  if ( !core_app )
    return;
  QObject::connect( QCoreApplication::instance(), &QCoreApplication::aboutToQuit, this,
                    &Ros2Qml::shutdown );
}

Ros2Qml::~Ros2Qml()
{
  if ( node_ == nullptr )
    return;

  QML_ROS2_PLUGIN_WARN(
      "Ros2Qml destructing but context still alive. Please call Ros2.shutdown() "
      "before application exit to ensure safe clean up. Otherwise the middleware might crash." );
  shutdown();
}

void Ros2Qml::registerDependant() { ++count_wrappers; }

void Ros2Qml::unregisterDependant()
{
  int count = --count_wrappers;
  if ( count < 0 ) {
    QML_ROS2_PLUGIN_WARN(
        "Stop spinning was called more often than start spinning! This is a bug!" );
    ++count_wrappers;
  }
}

QString Ros2Qml::hostname() const { return QHostInfo::localHostName(); }

bool Ros2Qml::isInitialized() const { return context_ != nullptr; }

void Ros2Qml::init( const QString &name, Ros2InitOptions *options )
{
  const QStringList &arguments = QCoreApplication::arguments();
  init( name, arguments, options );
}

void Ros2Qml::init( const QString &name, const QStringList &argv, Ros2InitOptions *options )
{
  if ( context_ != nullptr ) {
    QML_ROS2_PLUGIN_WARN( "Was already initialized. Second call to init ignored." );
    return;
  }
  //  std::this_thread::sleep_for(std::chrono::seconds(10));
  int argc = argv.size();
  std::vector<const char *> p_args( argc );
  std::vector<std::string> args( argc );
  std::string arg_debug_string;
  for ( int i = 0; i < argv.size(); ++i ) {
    args[i] = argv[i].toStdString();
    arg_debug_string += args[i] + " ";
    p_args[i] = args[i].c_str();
  }
  QML_ROS2_PLUGIN_DEBUG( "Initializing QML Ros2 with args: %s", arg_debug_string.c_str() );
  context_ = rclcpp::Context::make_shared();
  context_->init( argc, p_args.data(),
                  options ? options->rclcppInitOptions() : rclcpp::InitOptions() );
  rclcpp::NodeOptions node_options;
  node_options.context( context_ );
#if RCLCPP_VERSION_MAJOR >= 28
  node_options.enable_logger_service( true );
#endif
  std::string node_namespace = options ? options->getNamespace().toStdString() : "";
  node_ = rclcpp::Node::make_shared( name.toStdString(), node_namespace, node_options );
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  // StaticSingleThreadedExecutor may be a bit faster but will keep a reference to the subscription
  // and therefore not unsubscribe if the subscription is reset.
  auto executor = rclcpp::experimental::executors::EventsExecutor::make_unique( executor_options );
  executor->add_node( node_ );
  graph_event_ = node_->get_node_graph_interface()->get_graph_event();
  graph_timer_.start();
  emit initialized();

  executor_thread_ = std::thread( [exec = std::move( executor )]() { exec->spin(); } );
  QML_ROS2_PLUGIN_DEBUG( "QML Ros2 initialized." );
}

void Ros2Qml::shutdown()
{
  QML_ROS2_PLUGIN_DEBUG( "Shutting down Ros2Qml..." );
  emit aboutToShutdown();
  graph_timer_.stop();
  graph_event_ = nullptr;
  rclcpp::shutdown( context_, "Shutting down Ros2Qml." );
  if ( executor_thread_.joinable() )
    executor_thread_.join();
  node_ = nullptr;
  context_ = nullptr;
  QML_ROS2_PLUGIN_DEBUG( "Ros2Qml shut down." );
}

bool Ros2Qml::ok() const { return context_ != nullptr && rclcpp::ok( context_ ); }

QVariant Ros2Qml::createEmptyMessage( const QString &datatype ) const
{
  try {
    auto message = babel_fish_.create_message_shared( datatype.toStdString() );
    return conversion::msgToMap( message );
  } catch ( ros_babel_fish::BabelFishException &ex ) {
    QML_ROS2_PLUGIN_WARN( "Failed to create empty message for datatype '%s': %s",
                          datatype.toStdString().c_str(), ex.what() );
  }
  return QVariant();
}

QVariant Ros2Qml::createEmptyServiceRequest( const QString &datatype ) const
{
  try {
    auto message = babel_fish_.create_service_request_shared( datatype.toStdString() );
    return conversion::msgToMap( message );
  } catch ( ros_babel_fish::BabelFishException &ex ) {
    QML_ROS2_PLUGIN_WARN( "Failed to create empty service request for datatype '%s': %s",
                          datatype.toStdString().c_str(), ex.what() );
  }
  return {};
}

QVariant Ros2Qml::createEmptyActionGoal( const QString &datatype ) const
{
  try {
    auto message = babel_fish_.create_action_goal_shared( datatype.toStdString() );
    return conversion::msgToMap( message );
  } catch ( ros_babel_fish::BabelFishException &ex ) {
    QML_ROS2_PLUGIN_WARN( "Failed to create empty action goal request for datatype '%s': %s",
                          datatype.toStdString().c_str(), ex.what() );
  }
  return {};
}

std::shared_ptr<rclcpp::Node> Ros2Qml::node() { return node_; }

std::shared_ptr<rclcpp::Context> Ros2Qml::context() const { return context_; }

/***************************************************************************************************/
/************************************* Ros2QmlSingletonWrapper **************************************/
/***************************************************************************************************/

Ros2QmlSingletonWrapper::Ros2QmlSingletonWrapper()
{
  connect( &Ros2Qml::getInstance(), &Ros2Qml::initialized, this,
           &Ros2QmlSingletonWrapper::initialized );
  connect( &Ros2Qml::getInstance(), &Ros2Qml::aboutToShutdown, this,
           &Ros2QmlSingletonWrapper::aboutToShutdown );
  connect( &Ros2Qml::getInstance(), &Ros2Qml::graphChanged, this,
           &Ros2QmlSingletonWrapper::graphChanged );
  Ros2Qml::getInstance().registerDependant();
}

Ros2QmlSingletonWrapper::~Ros2QmlSingletonWrapper()
{
  Ros2Qml::getInstance().unregisterDependant();
}

QString Ros2QmlSingletonWrapper::hostname() const { return Ros2Qml::getInstance().hostname(); }

QObject *Ros2QmlSingletonWrapper::createInitOptions() { return new Ros2InitOptions; }

QoSWrapper Ros2QmlSingletonWrapper::QoS() { return {}; }

QoSWrapper Ros2QmlSingletonWrapper::BestAvailableQoS()
{
  return QoSWrapper( rclcpp::BestAvailableQoS() );
}
QoSWrapper Ros2QmlSingletonWrapper::ClockQoS() { return QoSWrapper( rclcpp::ClockQoS() ); }

QoSWrapper Ros2QmlSingletonWrapper::SensorDataQoS()
{
  return QoSWrapper( rclcpp::SensorDataQoS() );
}

QoSWrapper Ros2QmlSingletonWrapper::ServicesQoS() { return QoSWrapper( rclcpp::ServicesQoS() ); }

QoSWrapper Ros2QmlSingletonWrapper::SystemDefaultsQoS()
{
  return QoSWrapper( rclcpp::SystemDefaultsQoS() );
}

bool Ros2QmlSingletonWrapper::isInitialized() const
{
  return Ros2Qml::getInstance().isInitialized();
}

void Ros2QmlSingletonWrapper::init( const QString &name, QObject *options )
{
  Ros2Qml::getInstance().init( name, qobject_cast<Ros2InitOptions *>( options ) );
}

void Ros2QmlSingletonWrapper::init( const QString &name, const QStringList &args, QObject *options )
{
  Ros2Qml::getInstance().init( name, args, qobject_cast<Ros2InitOptions *>( options ) );
}

void Ros2QmlSingletonWrapper::shutdown() { Ros2Qml::getInstance().shutdown(); }

bool Ros2QmlSingletonWrapper::ok() const { return Ros2Qml::getInstance().ok(); }

Time Ros2QmlSingletonWrapper::now() const
{
  if ( !Ros2Qml::getInstance().isInitialized() )
    return Time();
  return Time( Ros2Qml::getInstance().node()->now() );
}

QString Ros2QmlSingletonWrapper::getName()
{
  if ( !isInitialized() )
    return {};
  return QString::fromStdString( Ros2Qml::getInstance().node()->get_name() );
}

QString Ros2QmlSingletonWrapper::getNamespace()
{
  if ( !isInitialized() )
    return {};
  return QString::fromStdString( Ros2Qml::getInstance().node()->get_namespace() );
}

bool Ros2QmlSingletonWrapper::isValidTopic( const QString &topic ) const
{
  int validation_result;
  size_t invalid_index;
  rcl_ret_t ret =
      rcl_validate_topic_name( topic.toStdString().c_str(), &validation_result, &invalid_index );
  if ( ret != RCL_RET_OK ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to validate topic name '%s': %s", topic.toStdString().c_str(),
                           rcl_get_error_string().str );
    return false;
  }
  return validation_result == RCL_TOPIC_NAME_VALID;
}

QStringList Ros2QmlSingletonWrapper::queryTopics( const QString &datatype ) const
{
  return Ros2Qml::getInstance().queryTopics( datatype );
}

QList<TopicInfo> Ros2QmlSingletonWrapper::queryTopicInfo() const
{
  return Ros2Qml::getInstance().queryTopicInfo();
}

QStringList Ros2QmlSingletonWrapper::queryTopicTypes( const QString &name ) const
{
  return Ros2Qml::getInstance().queryTopicTypes( name );
}

QVariantMap Ros2QmlSingletonWrapper::getTopicNamesAndTypes() const
{
  QVariantMap result;
  QMap<QString, QStringList> topic_names_and_types = Ros2Qml::getInstance().getTopicNamesAndTypes();
  for ( auto it = topic_names_and_types.begin(); it != topic_names_and_types.end(); ++it ) {
    result.insert( it.key(), QVariant( it.value() ) );
  }
  return result;
}

QStringList Ros2QmlSingletonWrapper::getTopicTypes( const QString &name ) const
{
  return Ros2Qml::getInstance().queryTopicTypes( name );
}

QStringList Ros2QmlSingletonWrapper::queryServices( const QString &datatype ) const
{
  return Ros2Qml::getInstance().queryServices( datatype );
}

QVariantMap Ros2QmlSingletonWrapper::getServiceNamesAndTypes() const
{
  QVariantMap result;
  QMap<QString, QStringList> topic_names_and_types = Ros2Qml::getInstance().getServiceNamesAndTypes();
  for ( auto it = topic_names_and_types.begin(); it != topic_names_and_types.end(); ++it ) {
    result.insert( it.key(), QVariant( it.value() ) );
  }
  return result;
}

QStringList Ros2QmlSingletonWrapper::getServiceTypes( const QString &name ) const
{
  QMap<QString, QStringList> service_names_and_types =
      Ros2Qml::getInstance().getServiceNamesAndTypes();
  return service_names_and_types.value( name );
}

QStringList Ros2QmlSingletonWrapper::queryActions( const QString &datatype ) const
{
  return Ros2Qml::getInstance().queryActions( datatype );
}

QVariantMap Ros2QmlSingletonWrapper::getActionNamesAndTypes() const
{
  QVariantMap result;
  QMap<QString, QStringList> action_names_and_types = Ros2Qml::getInstance().getActionNamesAndTypes();
  for ( auto it = action_names_and_types.begin(); it != action_names_and_types.end(); ++it ) {
    result.insert( it.key(), QVariant( it.value() ) );
  }
  return result;
}

QStringList Ros2QmlSingletonWrapper::getActionTypes( const QString &name ) const
{
  QMap<QString, QStringList> action_names_and_types = Ros2Qml::getInstance().getActionNamesAndTypes();
  return action_names_and_types.value( name );
}

QVariantList Ros2QmlSingletonWrapper::getNodeNamesAndNamespaces() const
{
  return Ros2Qml::getInstance().getNodeNamesAndNamespaces();
}

QVariantMap Ros2QmlSingletonWrapper::getNodeInfo( const QString &name, const QString &ns ) const
{
  return Ros2Qml::getInstance().getNodeInfo( name, ns );
}

QVariantList Ros2QmlSingletonWrapper::getAllNodeInfo() const
{
  return Ros2Qml::getInstance().getAllNodeInfo();
}

QVariantList Ros2QmlSingletonWrapper::getPublishersInfoByTopic( const QString &topic ) const
{
  return Ros2Qml::getInstance().getPublishersInfoByTopic( topic );
}

QVariantList Ros2QmlSingletonWrapper::getSubscriptionsInfoByTopic( const QString &topic ) const
{
  return Ros2Qml::getInstance().getSubscriptionsInfoByTopic( topic );
}

QVariant Ros2QmlSingletonWrapper::createEmptyMessage( const QString &datatype ) const
{
  return Ros2Qml::getInstance().createEmptyMessage( datatype );
}

QVariant Ros2QmlSingletonWrapper::createEmptyServiceRequest( const QString &datatype ) const
{
  return Ros2Qml::getInstance().createEmptyServiceRequest( datatype );
}

QVariant Ros2QmlSingletonWrapper::createEmptyActionGoal( const QString &datatype ) const
{
  return Ros2Qml::getInstance().createEmptyActionGoal( datatype );
}

IO Ros2QmlSingletonWrapper::io() const { return {}; }

QObject *Ros2QmlSingletonWrapper::getLogger( const QString &name )
{
  if ( name.isEmpty() ) {
    const rclcpp::Node::SharedPtr &node = Ros2Qml::getInstance().node();
    if ( node == nullptr ) {
      QML_ROS2_PLUGIN_ERROR( "Can not get node logger before Ros2 is initialized." );
      return nullptr;
    }
    return new Logger( node->get_logger() );
  }
  return new Logger( rclcpp::get_logger( name.toStdString() ) );
}

QJSValue Ros2QmlSingletonWrapper::debug()
{
  if ( !initLogging() )
    return {};
  return dynamic_cast<Logger *>( logger_.toQObject() )->debug();
}

QJSValue Ros2QmlSingletonWrapper::info()
{
  if ( !initLogging() )
    return {};
  return dynamic_cast<Logger *>( logger_.toQObject() )->info();
}

QJSValue Ros2QmlSingletonWrapper::warn()
{
  if ( !initLogging() )
    return {};
  return dynamic_cast<Logger *>( logger_.toQObject() )->warn();
}

QJSValue Ros2QmlSingletonWrapper::error()
{
  if ( !initLogging() )
    return {};
  return dynamic_cast<Logger *>( logger_.toQObject() )->error();
}

QJSValue Ros2QmlSingletonWrapper::fatal()
{
  if ( !initLogging() )
    return {};
  return dynamic_cast<Logger *>( logger_.toQObject() )->fatal();
}

QObject *Ros2QmlSingletonWrapper::createPublisher( const QString &topic, const QString &type,
                                                   const qml6_ros2_plugin::QoSWrapper &qos )
{
  return new Publisher( topic, type, qos );
}

QObject *Ros2QmlSingletonWrapper::createPublisher( const QString &topic, const QString &type,
                                                   quint32 queue_size )
{
  return createPublisher( topic, type, QoSWrapper().reliable().keep_last( queue_size ) );
}

QObject *Ros2QmlSingletonWrapper::createSubscription( const QString &topic, const QoSWrapper &qos )
{
  return new Subscription( topic, QString(), qos );
}

QObject *Ros2QmlSingletonWrapper::createSubscription( const QString &topic, quint32 queue_size )
{
  return createSubscription( topic, QoSWrapper().keep_last( queue_size ) );
}

QObject *Ros2QmlSingletonWrapper::createSubscription( const QString &topic,
                                                      const QString &message_type,
                                                      const QoSWrapper &qos )
{
  return new Subscription( topic, message_type, qos );
}

QObject *Ros2QmlSingletonWrapper::createSubscription( const QString &topic,
                                                      const QString &message_type, quint32 queue_size )
{
  return createSubscription( topic, message_type, QoSWrapper().keep_last( queue_size ) );
}

QObject *Ros2QmlSingletonWrapper::createServiceClient( const QString &name, const QString &type )
{
  return createServiceClient( name, type, QoSWrapper( rclcpp::ServicesQoS() ) );
}

QObject *Ros2QmlSingletonWrapper::createServiceClient( const QString &name, const QString &type,
                                                       const qml6_ros2_plugin::QoSWrapper &qos )
{
  return new ServiceClient( name, type, qos );
}

QObject *Ros2QmlSingletonWrapper::createActionClient( const QString &name, const QString &type )
{
  return new ActionClient( name, type );
}

bool Ros2QmlSingletonWrapper::initLogging()
{
  if ( !logger_.isUndefined() && !logger_.isNull() )
    return true;
  const std::shared_ptr<rclcpp::Node> &node = Ros2Qml::getInstance().node();
  if ( node == nullptr ) {
    QML_ROS2_PLUGIN_ERROR( "You need to initialize Ros2 before calling a log function!" );
    return false;
  }
  QJSEngine *engine = qjsEngine( this );
  if ( !engine ) {
    QML_ROS2_PLUGIN_ERROR( "Failed to get QJSEngine in initLogging. Can not create logger." );
    return false;
  }
  logger_ = engine->newQObject( new Logger( node->get_logger() ) );
  return true;
}
} // namespace qml6_ros2_plugin
