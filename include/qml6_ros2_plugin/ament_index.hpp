// Copyright (c) 2021 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML_ROS2_PLUGIN_AMENT_INDEX_HPP
#define QML_ROS2_PLUGIN_AMENT_INDEX_HPP

#include <QtCore>

namespace qml6_ros2_plugin
{

/*!
 * Wrapper for ament_index_cpp.
 */
class AmentIndex : public QObject
{
  Q_OBJECT

public:
  /*!
   * Queries the path to the share directory for a given package.
   * @param package_name The name of the package.
   * @return The fully-qualified path to the share directory of the given
   * package or an empty string if the package is not found.
   */
  Q_INVOKABLE QString getPackageShareDirectory( const QString &package_name );

  /*!
   * Alias for `getPackageShareDirectory` to match the ament_index_cpp API. Use this in new code.
   * @see getPackageShareDirectory(const QString &)
   */
  Q_INVOKABLE QString getPackageSharePath( const QString &package_name );

  /*!
   * Queries the path to the prefix directory for a given package.
   * @param package_name The name of the package.
   * @return The fully-qualified path to the prefix directory of the given
   * package or an empty string if the package is not found.
   */
  Q_INVOKABLE QString getPackagePrefix( const QString &package_name );

  //! @return A list of all packages.
  Q_INVOKABLE QStringList getPackages();

  /*!
   * Queries a resource registered in the ament index.
   * @param resource_type The type of the resource, e.g. "rclcpp_components".
   *   Only letters, digits, '_' and '-' are allowed.
   * @param resource_name The name of the resource, e.g. the package name.
   *   Only letters, digits, '_' and '-' are allowed.
   * @return An object with the resource's \c content and the \c prefix path of the install
   * location it was found in, or undefined if an argument is invalid, the resource is not found
   * or the lookup failed.
   */
  Q_INVOKABLE QVariant getResource( const QString &resource_type, const QString &resource_name );
};
} // namespace qml6_ros2_plugin

#endif // QML_ROS2_PLUGIN_AMENT_INDEX_HPP
