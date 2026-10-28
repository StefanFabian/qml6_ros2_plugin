// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <qml6_ros2_plugin/qos.hpp>

#include <QCoreApplication>
#include <QJSEngine>
#include <gtest/gtest.h>

using namespace qml6_ros2_plugin;

TEST( QoS, policies )
{
  QoSWrapper qos = QoSWrapper().reliable().transient_local().keep_last( 5 );
  EXPECT_EQ( qos.reliability(), qos_reliability_policy::Reliable );
  EXPECT_EQ( qos.durability(), qos_durability_policy::TransientLocal );
  EXPECT_EQ( qos.history(), qos_history_policy::KeepLast );
  EXPECT_EQ( qos.depth(), 5 );

  qos = QoSWrapper().keep_all();
  EXPECT_EQ( qos.reliability(), qos_reliability_policy::BestEffort );
  EXPECT_EQ( qos.durability(), qos_durability_policy::Volatile );
  EXPECT_EQ( qos.history(), qos_history_policy::KeepAll );

  qos = QoSWrapper( rclcpp::SystemDefaultsQoS() );
  EXPECT_EQ( qos.reliability(), qos_reliability_policy::SystemDefault );
  EXPECT_EQ( qos.durability(), qos_durability_policy::SystemDefault );

  qos = QoSWrapper( rclcpp::BestAvailableQoS() );
  EXPECT_EQ( qos.reliability(), qos_reliability_policy::BestAvailable );
  EXPECT_EQ( qos.durability(), qos_durability_policy::BestAvailable );
}

TEST( QoS, toString )
{
  EXPECT_EQ(
      QoSWrapper().reliable().transient_local().keep_last( 5 ).toString(),
      "QoS(reliability: Reliable, durability: TransientLocal, history: KeepLast, depth: 5)" );
}

TEST( QoS, javascriptAccess )
{
  QJSEngine engine;
  engine.globalObject().setProperty(
      "qos", engine.toScriptValue( QoSWrapper().reliable().transient_local().keep_last( 5 ) ) );
  // Strict comparison against the QML enums requires numbers; depth is a property, not a function.
  for ( const char *property : { "reliability", "durability", "history", "depth" } ) {
    EXPECT_EQ( engine.evaluate( QString( "typeof qos." ) + property ).toString().toStdString(),
               "number" )
        << property;
  }
  EXPECT_EQ( engine.evaluate( "qos.reliability" ).toInt(), qos_reliability_policy::Reliable );
  EXPECT_EQ( engine.evaluate( "qos.durability" ).toInt(), qos_durability_policy::TransientLocal );
  EXPECT_EQ( engine.evaluate( "qos.history" ).toInt(), qos_history_policy::KeepLast );
  EXPECT_EQ( engine.evaluate( "qos.depth" ).toInt(), 5 );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  QCoreApplication app( argc, argv );
  return RUN_ALL_TESTS();
}
