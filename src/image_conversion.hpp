// Copyright (c) 2025 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML6_ROS2_PLUGIN_IMAGE_CONVERSION_HPP
#define QML6_ROS2_PLUGIN_IMAGE_CONVERSION_HPP

#include "./logging.hpp"
#include <QVideoFrame>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace qml6_ros2_plugin
{
inline QVideoFrameFormat::PixelFormat getVideoFramePixelFormat( const std::string &encoding )
{
  if ( encoding == sensor_msgs::image_encodings::RGB8 ||
       encoding == sensor_msgs::image_encodings::RGB16 ) {
    return QVideoFrameFormat::Format_RGBX8888;
  } else if ( encoding == sensor_msgs::image_encodings::BGR8 ||
              encoding == sensor_msgs::image_encodings::BGR16 ) {
    return QVideoFrameFormat::Format_BGRX8888;
  } else if ( encoding == sensor_msgs::image_encodings::MONO8 ||
              encoding == sensor_msgs::image_encodings::TYPE_8UC1 ) {
    return QVideoFrameFormat::Format_Y8;
  } else if ( encoding == sensor_msgs::image_encodings::MONO16 ||
              encoding == sensor_msgs::image_encodings::TYPE_16UC1 ) {
    return QVideoFrameFormat::Format_Y16;
  } else if ( encoding == sensor_msgs::image_encodings::RGBA8 ||
              encoding == sensor_msgs::image_encodings::RGBA16 ) {
    return QVideoFrameFormat::Format_RGBA8888;
  } else if ( encoding == sensor_msgs::image_encodings::BGRA8 ||
              encoding == sensor_msgs::image_encodings::BGRA16 ) {
    return QVideoFrameFormat::Format_BGRA8888;
  } else if ( encoding == sensor_msgs::image_encodings::TYPE_32FC1 ) {
    return QVideoFrameFormat::Format_Y16;
#if RCLCPP_VERSION_MAJOR >= 28
  } else if ( encoding == sensor_msgs::image_encodings::YUYV ||
              encoding == sensor_msgs::image_encodings::YUV422_YUY2 ) {
#else
  } else if ( encoding == sensor_msgs::image_encodings::YUV422_YUY2 ) {
#endif
    return QVideoFrameFormat::Format_YUYV;
#if RCLCPP_VERSION_MAJOR >= 28
  } else if ( encoding == sensor_msgs::image_encodings::UYVY ||
              encoding == sensor_msgs::image_encodings::YUV422 ) {
#else
  } else if ( encoding == sensor_msgs::image_encodings::YUV422 ) {
#endif
    return QVideoFrameFormat::Format_UYVY;
  }
  QML_ROS2_PLUGIN_WARN_THROTTLE( 5000, "Unsupported image encoding '%s' received.", encoding.c_str() );
  return QVideoFrameFormat::Format_Invalid;
}

template<int IDX_1, int IDX_2, int IDX_3, typename CHANNEL_TYPE>
void convertTo3Channel( const sensor_msgs::msg::Image::ConstSharedPtr &image, uchar *data,
                        int bytes_per_line )
{
  const uchar *src_data = image->data.data();
  int src_step = image->step;
  int width = image->width;
  int height = image->height;
  constexpr int divider = sizeof( CHANNEL_TYPE ) == 2 ? 256 : 1;

  for ( int y = 0; y < height; ++y ) {
    const auto *src_row = reinterpret_cast<const CHANNEL_TYPE *>( src_data + y * src_step );
    uchar *dst_row = data + y * bytes_per_line;
    for ( int x = 0; x < width; ++x ) {
      const CHANNEL_TYPE *src_pixel = src_row + x * 3;
      uchar *dst_pixel = dst_row + x * 4;
      dst_pixel[IDX_1] = src_pixel[0] / divider;
      dst_pixel[IDX_2] = src_pixel[1] / divider;
      dst_pixel[IDX_3] = src_pixel[2] / divider;
      dst_pixel[3] = 255; // Alpha channel
    }
  }
}

template<int IDX_1, int IDX_2, int IDX_3, int IDX_4, typename CHANNEL_TYPE>
void convertTo4Channel( const sensor_msgs::msg::Image::ConstSharedPtr &image, uchar *data,
                        int bytes_per_line )
{
  const uchar *src_data = image->data.data();
  int src_step = image->step;
  int width = image->width;
  int height = image->height;
  constexpr int divider = sizeof( CHANNEL_TYPE ) == 2 ? 256 : 1;

  for ( int y = 0; y < height; ++y ) {
    const auto *src_row = reinterpret_cast<const CHANNEL_TYPE *>( src_data + y * src_step );
    uchar *dst_row = data + y * bytes_per_line;
    for ( int x = 0; x < width; ++x ) {
      const CHANNEL_TYPE *src_pixel = src_row + x * 4;
      uchar *dst_pixel = dst_row + x * 4;
      dst_pixel[IDX_1] = src_pixel[0] / divider;
      dst_pixel[IDX_2] = src_pixel[1] / divider;
      dst_pixel[IDX_3] = src_pixel[2] / divider;
      dst_pixel[IDX_4] = src_pixel[3] / divider;
    }
  }
}

template<typename T>
void convertToY16( const sensor_msgs::msg::Image::ConstSharedPtr &image, uchar *data,
                   int bytes_per_line )
{
  const T *src_data = reinterpret_cast<const T *>( image->data.data() );
  int src_step = image->step / sizeof( T );
  int width = image->width;
  int height = image->height;

  for ( int y = 0; y < height; ++y ) {
    const T *src_row = src_data + y * src_step;
    uchar *dst_row = data + y * bytes_per_line;
    for ( int x = 0; x < width; ++x ) {
      T value = src_row[x];
      // Scale the value to 16 bits
      uint16_t scaled_value = static_cast<uint16_t>( std::min(
          std::max( static_cast<int>( value * 65535.0 / std::numeric_limits<T>::max() ), 0 ),
          65535 ) );
      dst_row[x * 2] = static_cast<uchar>( scaled_value & 0xFF );
      dst_row[x * 2 + 1] = static_cast<uchar>( ( scaled_value >> 8 ) & 0xFF );
    }
  }
}

template<int BYTES_PER_PIXEL>
void copyImageData( const sensor_msgs::msg::Image::ConstSharedPtr &image, uchar *data,
                    int bytes_per_line )
{
  const uchar *src_data = image->data.data();
  int src_step = image->step;
  int width = image->width;
  int height = image->height;

  if ( bytes_per_line == src_step ) {
    std::memcpy( data, src_data, height * bytes_per_line );
    return;
  }

  for ( int y = 0; y < height; ++y ) {
    const uchar *src_row = src_data + y * src_step;
    uchar *dst_row = data + y * bytes_per_line;
    std::memcpy( dst_row, src_row, width * BYTES_PER_PIXEL );
  }
}

inline bool writeImageToVideoFrame( const sensor_msgs::msg::Image::ConstSharedPtr &image,
                                    QVideoFrame &frame )
{
  if ( !frame.map( QVideoFrame::WriteOnly ) )
    return false;

  uchar *data = frame.bits( 0 );
  if ( frame.planeCount() > 1 ) {
    QML_ROS2_PLUGIN_ERROR_THROTTLE( 1000, "Multi-planar QVideoFrame formats are not supported." );
    frame.unmap();
    return false;
  }
  bool success = true;
  switch ( frame.pixelFormat() ) {
  case QVideoFrameFormat::Format_RGBX8888:
    if ( image->encoding == sensor_msgs::image_encodings::RGB8 ) {
      convertTo3Channel<0, 1, 2, uint8_t>( image, data, frame.bytesPerLine( 0 ) );
    } else if ( image->encoding == sensor_msgs::image_encodings::RGB16 ) {
      convertTo3Channel<0, 1, 2, uint16_t>( image, data, frame.bytesPerLine( 0 ) );
    } else {
      success = false;
    }
    break;
  case QVideoFrameFormat::Format_BGRX8888:
    if ( image->encoding == sensor_msgs::image_encodings::BGR8 ) {
      convertTo3Channel<2, 1, 0, uint8_t>( image, data, frame.bytesPerLine( 0 ) );
    } else if ( image->encoding == sensor_msgs::image_encodings::BGR16 ) {
      convertTo3Channel<2, 1, 0, uint8_t>( image, data, frame.bytesPerLine( 0 ) );
    } else {
      success = false;
    }
    break;
  case QVideoFrameFormat::Format_Y8:
    if ( image->encoding == sensor_msgs::image_encodings::MONO8 ||
         image->encoding == sensor_msgs::image_encodings::TYPE_8UC1 ) {
      copyImageData<1>( image, data, frame.bytesPerLine( 0 ) );
    } else {
      success = false;
    }
    break;
  case QVideoFrameFormat::Format_Y16:
    if ( image->encoding == sensor_msgs::image_encodings::MONO16 ||
         image->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ) {
      copyImageData<2>( image, data, frame.bytesPerLine( 0 ) );
    } else if ( image->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ) {
      convertToY16<float>( image, data, frame.bytesPerLine( 0 ) );
    } else {
      success = false;
    }
    break;
  case QVideoFrameFormat::Format_RGBA8888:
    if ( image->encoding == sensor_msgs::image_encodings::RGBA8 ) {
      convertTo4Channel<0, 1, 2, 3, uint8_t>( image, data, frame.bytesPerLine( 0 ) );
    } else if ( image->encoding == sensor_msgs::image_encodings::RGBA16 ) {
      convertTo4Channel<0, 1, 2, 3, uint16_t>( image, data, frame.bytesPerLine( 0 ) );
    } else {
      success = false;
    }
    break;
  case QVideoFrameFormat::Format_BGRA8888:
    if ( image->encoding == sensor_msgs::image_encodings::BGRA8 ) {
      convertTo4Channel<0, 1, 2, 3, uint8_t>( image, data, frame.bytesPerLine( 0 ) );
    } else if ( image->encoding == sensor_msgs::image_encodings::BGRA16 ) {
      convertTo4Channel<0, 1, 2, 3, uint16_t>( image, data, frame.bytesPerLine( 0 ) );
    } else {
      success = false;
    }
    break;
  case QVideoFrameFormat::Format_YUYV:
    if ( image->encoding == sensor_msgs::image_encodings::YUV422_YUY2 ) {
      std::memcpy( data, image->data.data(), image->data.size() );
    } else {
      success = false;
    }
    break;
  case QVideoFrameFormat::Format_UYVY:
    if ( image->encoding == sensor_msgs::image_encodings::YUV422 ) {
      std::memcpy( data, image->data.data(), image->data.size() );
    } else {
      success = false;
    }
    break;
  default:
    QML_ROS2_PLUGIN_ERROR_THROTTLE( 1000, "Unexpected pixel format in QVideoFrame: %d",
                                    frame.pixelFormat() );
    success = false;
    break;
  }

  frame.unmap();
  return success;
}
} // namespace qml6_ros2_plugin

#endif // QML6_ROS2_PLUGIN_IMAGE_CONVERSION_HPP
