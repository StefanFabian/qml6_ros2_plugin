// Copyright (c) 2026 Stefan Fabian. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.

#ifndef QML6_ROS2_PLUGIN_INTERNAL_WINDOW_RATE_TRACKER_HPP
#define QML6_ROS2_PLUGIN_INTERNAL_WINDOW_RATE_TRACKER_HPP

#include "qml6_ros2_plugin/internal/ring_buffer.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>

namespace qml6_ros2_plugin::internal
{

template<std::size_t N>
class WindowRateTracker
{
public:
  using Clock = std::chrono::steady_clock;
  using TimePoint = Clock::time_point;

  struct Rates {
    double frequency_hz = 0.0;
    double bandwidth_bps = 0.0;
  };

  void addSample( TimePoint timestamp, std::size_t bytes = 0 )
  {
    samples_.push_back( Sample{ timestamp, bytes } );
  }

  void clear() { samples_.clear(); }

  Rates rates( TimePoint now ) const
  {
    if ( samples_.size() < 2 )
      return {};

    TimePoint oldest = samples_.front().timestamp;
    TimePoint newest = samples_.back().timestamp;
    std::size_t total_bytes = 0;

    for ( const auto &sample : samples_ ) { total_bytes += sample.bytes; }

    const auto span_ns = std::chrono::duration_cast<std::chrono::nanoseconds>( newest - oldest );
    if ( span_ns.count() <= 0 )
      return {};

    const double span_s = static_cast<double>( span_ns.count() ) * 1e-9;
    const double base_frequency = static_cast<double>( samples_.size() - 1 ) / span_s;
    const double base_bandwidth = static_cast<double>( total_bytes ) / span_s;

    const auto staleness_ns = std::chrono::duration_cast<std::chrono::nanoseconds>( now - newest );
    const double staleness_s = std::max( 0.0, static_cast<double>( staleness_ns.count() ) * 1e-9 );
    const double expected_period_s = 1 / base_frequency;
    const double decay_threshold_s = 1.5 * expected_period_s;

    double factor = 1.0;
    if ( staleness_s > decay_threshold_s ) {
      factor = 1 - std::min( staleness_s / span_s, 1.0 );
    }

    return { base_frequency * factor, base_bandwidth * factor };
  }

private:
  struct Sample {
    TimePoint timestamp;
    std::size_t bytes;
  };

  RingBuffer<Sample, N> samples_;
};

} // namespace qml6_ros2_plugin::internal

#endif // QML6_ROS2_PLUGIN_INTERNAL_WINDOW_RATE_TRACKER_HPP
