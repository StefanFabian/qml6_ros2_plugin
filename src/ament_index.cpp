// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml6_ros2_plugin/ament_index.hpp"
#include "logging.hpp"
#include <QRegularExpression>
#include <exception>

#if __has_include( <ament_index_cpp/version.h> )
  #include <ament_index_cpp/version.h>
#else
  #define AMENT_INDEX_CPP_VERSION_GTE( major, minor, patch ) false
#endif

#include <ament_index_cpp/get_package_prefix.hpp>
#if AMENT_INDEX_CPP_VERSION_GTE( 1, 13, 2 )
  #include <ament_index_cpp/get_package_share_path.hpp>
#else
  #include <ament_index_cpp/get_package_share_directory.hpp>
#endif
#include <ament_index_cpp/get_packages_with_prefixes.hpp>
#include <ament_index_cpp/get_resource.hpp>

namespace qml6_ros2_plugin
{

QString AmentIndex::getPackageShareDirectory( const QString &package_name )
{
  if ( package_name.isEmpty() ) {
    return {};
  }
  try {
#if AMENT_INDEX_CPP_VERSION_GTE( 1, 13, 2 )
    std::filesystem::path path =
        ament_index_cpp::get_package_share_path( package_name.toStdString() );
    return QString::fromStdString( path.string() );
#elif AMENT_INDEX_CPP_VERSION_GTE( 1, 13, 0 )
    std::filesystem::path path;
    ament_index_cpp::get_package_share_directory( package_name.toStdString(), path );
    return QString::fromStdString( path.string() );
#else
    return QString::fromStdString(
        ament_index_cpp::get_package_share_directory( package_name.toStdString() ) );
#endif
  } catch ( const ament_index_cpp::PackageNotFoundError & ) {
    return {};
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR_THROTTLE( 5000, "Failed to get share directory of package '%s': %s",
                                    qPrintable( package_name ), ex.what() );
    return {};
  }
}

QString AmentIndex::getPackagePrefix( const QString &package_name )
{
  if ( package_name.isEmpty() ) {
    return {};
  }
  try {
#if AMENT_INDEX_CPP_VERSION_GTE( 1, 13, 0 )
    std::filesystem::path path;
    ament_index_cpp::get_package_prefix( package_name.toStdString(), path );
    return QString::fromStdString( path.string() );
#else
    return QString::fromStdString( ament_index_cpp::get_package_prefix( package_name.toStdString() ) );
#endif
  } catch ( const ament_index_cpp::PackageNotFoundError & ) {
    return {};
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR_THROTTLE( 5000, "Failed to get prefix of package '%s': %s",
                                    qPrintable( package_name ), ex.what() );
    return {};
  }
}

QStringList AmentIndex::getPackages()
{
  try {
    QStringList result;
    const std::map<std::string, std::string> &packages =
        ament_index_cpp::get_packages_with_prefixes();
    result.reserve( packages.size() );
    for ( const auto &s : packages ) result.append( QString::fromStdString( s.first ) );
    return result;
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR_THROTTLE( 5000, "Failed to get packages: %s", ex.what() );
    return {};
  }
}

//! Whitelist keeping type and name a single path component so the lookup can't leave the index.
static bool isValidResourceIdentifier( const QString &identifier )
{
  static const QRegularExpression pattern( "^[A-Za-z0-9_-]+$" );
  return pattern.match( identifier ).hasMatch();
}

QVariant AmentIndex::getResource( const QString &resource_type, const QString &resource_name )
{
  if ( !isValidResourceIdentifier( resource_type ) || !isValidResourceIdentifier( resource_name ) )
    return {};
  try {
#if AMENT_INDEX_CPP_VERSION_GTE( 1, 13, 0 )
    const auto resource =
        ament_index_cpp::get_resource( resource_type.toStdString(), resource_name.toStdString() );
    if ( !resource.resourcePath )
      return {};
    return QVariantMap{ { "content", QString::fromStdString( resource.contents ) },
                        { "prefix", QString::fromStdString( resource.resourcePath->string() ) } };
#else
    std::string content;
    std::string prefix;
    if ( !ament_index_cpp::get_resource( resource_type.toStdString(), resource_name.toStdString(),
                                         content, &prefix ) )
      return {};
    return QVariantMap{ { "content", QString::fromStdString( content ) },
                        { "prefix", QString::fromStdString( prefix ) } };
#endif
  } catch ( const std::exception &ex ) {
    QML_ROS2_PLUGIN_ERROR_THROTTLE( 5000, "Failed to get resource '%s' of type '%s': %s",
                                    qPrintable( resource_name ), qPrintable( resource_type ),
                                    ex.what() );
    return {};
  }
}
} // namespace qml6_ros2_plugin
