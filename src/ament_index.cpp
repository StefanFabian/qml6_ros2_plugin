// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include "qml6_ros2_plugin/ament_index.hpp"

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
  } catch ( ament_index_cpp::PackageNotFoundError &ex ) {
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
  } catch ( ament_index_cpp::PackageNotFoundError &ex ) {
    return {};
  }
}

QStringList AmentIndex::getPackages()
{
  QStringList result;
  const std::map<std::string, std::string> &packages = ament_index_cpp::get_packages_with_prefixes();
  result.reserve( packages.size() );
  for ( const auto &s : packages ) result.append( QString::fromStdString( s.first ) );
  return result;
}
} // namespace qml6_ros2_plugin
