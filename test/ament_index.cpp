// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#include <qml6_ros2_plugin/ament_index.hpp>

#include <cstdlib>
#include <filesystem>
#include <gtest/gtest.h>
#include <optional>

#if __has_include( <ament_index_cpp/version.h> )
  #include <ament_index_cpp/version.h>
#else
  #define AMENT_INDEX_CPP_VERSION_GTE( major, minor, patch ) false
#endif

#if AMENT_INDEX_CPP_VERSION_GTE( 1, 13, 2 )
  #include <ament_index_cpp/get_package_share_path.hpp>
#else
  #include <ament_index_cpp/get_package_share_directory.hpp>
#endif

using namespace qml6_ros2_plugin;
namespace fs = std::filesystem;

/*!
 * Restores AMENT_PREFIX_PATH after the test, including leaving it unset if it was unset.
 */
class AmentPrefixPathTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if ( const char *original = std::getenv( "AMENT_PREFIX_PATH" ) )
      original_ = original;
  }

  void TearDown() override
  {
    if ( original_ )
      setenv( "AMENT_PREFIX_PATH", original_->c_str(), 1 );
    else
      unsetenv( "AMENT_PREFIX_PATH" );
  }

  std::optional<std::string> original_;
  AmentIndex index_;
};

/*!
 * Points AMENT_PREFIX_PATH to the overlay and underlay prefixes in test/test_ament_index
 * (overlay first).
 */
class AmentIndexTestData : public AmentPrefixPathTest
{
protected:
  void SetUp() override
  {
    AmentPrefixPathTest::SetUp();
    ASSERT_TRUE( original_.has_value() );

    fs::path data_dir;
#if AMENT_INDEX_CPP_VERSION_GTE( 1, 13, 2 )
    data_dir = ament_index_cpp::get_package_share_path( "qml6_ros2_plugin" );
#elif AMENT_INDEX_CPP_VERSION_GTE( 1, 13, 0 )
    ament_index_cpp::get_package_share_directory( "qml6_ros2_plugin", data_dir );
#else
    data_dir = ament_index_cpp::get_package_share_directory( "qml6_ros2_plugin" );
#endif
    data_dir /= "test/test_ament_index";
    overlay_ = data_dir / "overlay";
    underlay_ = data_dir / "underlay";
    setenv( "AMENT_PREFIX_PATH", ( overlay_.string() + ":" + underlay_.string() ).c_str(), 1 );
  }

  static QString str( const fs::path &path ) { return QString::fromStdString( path.string() ); }

  static QVariant resource( const QString &content, const fs::path &prefix )
  {
    return QVariantMap{ { "content", content }, { "prefix", str( prefix ) } };
  }

  fs::path overlay_;
  fs::path underlay_;
};

static bool isUndefined( const QVariant &value ) { return !value.isValid(); }

TEST( AmentIndex, getResourceEmptyArguments )
{
  AmentIndex index;
  // get_resource throws std::runtime_error on empty args; the wrapper must guard.
  EXPECT_TRUE( isUndefined( index.getResource( "", "some_name" ) ) );
  EXPECT_TRUE( isUndefined( index.getResource( "some_type", "" ) ) );
  EXPECT_TRUE( isUndefined( index.getResource( "", "" ) ) );
}

TEST( AmentIndex, getResourceNotFound )
{
  AmentIndex index;
  EXPECT_TRUE( isUndefined(
      index.getResource( "nonexistent_resource_type", "nonexistent_resource_name" ) ) );
}

TEST_F( AmentPrefixPathTest, withoutAmentPrefixPath )
{
  // ament_index_cpp throws when AMENT_PREFIX_PATH is unset; the wrapper must catch and return empty.
  unsetenv( "AMENT_PREFIX_PATH" );
  EXPECT_TRUE( isUndefined( index_.getResource( "some_type", "some_name" ) ) );
  EXPECT_EQ( index_.getPackagePrefix( "some_pkg" ), QString() );
  EXPECT_EQ( index_.getPackageShareDirectory( "some_pkg" ), QString() );
  EXPECT_EQ( index_.getPackages(), QStringList() );
}

TEST_F( AmentIndexTestData, getResource )
{
  EXPECT_EQ( index_.getResource( "test_type", "underlay_pkg" ),
             resource( "line1\nline2\n", underlay_ ) );
  EXPECT_EQ( index_.getResource( "test_type", "shared_pkg" ),
             resource( "overlay content", overlay_ ) );
  EXPECT_EQ( index_.getResource( "test_type", "empty_resource" ), resource( "", overlay_ ) );
  EXPECT_TRUE( isUndefined( index_.getResource( "test_type", "overlay_pkg" ) ) );
  EXPECT_TRUE( isUndefined( index_.getResource( "unknown_type", "shared_pkg" ) ) );
}

TEST_F( AmentIndexTestData, getResourceRejectsPathTraversal )
{
  // Both traversals resolve to the existing packages/shared_pkg file if not rejected.
  ASSERT_EQ( index_.getResource( "packages", "shared_pkg" ), resource( "", overlay_ ) );
  EXPECT_TRUE( isUndefined( index_.getResource( "test_type/../packages", "shared_pkg" ) ) );
  EXPECT_TRUE( isUndefined( index_.getResource( "test_type", "../packages/shared_pkg" ) ) );
  EXPECT_TRUE( isUndefined( index_.getResource( "..", "shared_pkg" ) ) );
}

TEST_F( AmentIndexTestData, getPackagePrefix )
{
  EXPECT_EQ( index_.getPackagePrefix( "overlay_pkg" ), str( overlay_ ) );
  EXPECT_EQ( index_.getPackagePrefix( "underlay_pkg" ), str( underlay_ ) );
  EXPECT_EQ( index_.getPackagePrefix( "shared_pkg" ), str( overlay_ ) );
  EXPECT_EQ( index_.getPackagePrefix( "unknown_pkg" ), QString() );
  EXPECT_EQ( index_.getPackagePrefix( "" ), QString() );
}

TEST_F( AmentIndexTestData, getPackageShareDirectory )
{
  EXPECT_EQ( index_.getPackageShareDirectory( "overlay_pkg" ),
             str( overlay_ / "share" / "overlay_pkg" ) );
  EXPECT_EQ( index_.getPackageShareDirectory( "underlay_pkg" ),
             str( underlay_ / "share" / "underlay_pkg" ) );
  EXPECT_EQ( index_.getPackageShareDirectory( "shared_pkg" ),
             str( overlay_ / "share" / "shared_pkg" ) );
  EXPECT_EQ( index_.getPackageShareDirectory( "unknown_pkg" ), QString() );
  EXPECT_EQ( index_.getPackageShareDirectory( "" ), QString() );
}

TEST_F( AmentIndexTestData, getPackages )
{
  EXPECT_EQ( index_.getPackages(), QStringList( { "overlay_pkg", "shared_pkg", "underlay_pkg" } ) );
}

int main( int argc, char **argv )
{
  testing::InitGoogleTest( &argc, argv );
  return RUN_ALL_TESTS();
}
