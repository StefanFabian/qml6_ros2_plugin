==============
ImageTransport
==============


Seeing what the robot sees is one of the most important features of any user interface.
To enable this, this library provides the
:cpp:class:`ImageTransportSubscription <qml6_ros2_plugin::ImageTransportSubscription>`.
It allows easy subscription of camera messages and provides them in a QML native format to a QVideoSink.

Example:

.. code-block:: qml

  ImageTransportSubscription {
    id: imageSubscription
    // Enter a valid image topic here
    topic: "/front_rgbd_cam/color/image_rect_color"
    // This is the default transport, change if compressed is not available
    defaultTransport: "compressed"
    videoSink: output.videoSink
  }

  VideoOutput {
    id: output
    anchors.fill: parent
    // Can be used in increments of 90 to rotate the video
    orientation: 90
  }

The :cpp:class:`ImageTransportSubscription <qml6_ros2_plugin::ImageTransportSubscription>`
uses the ``videoSink`` of a ``VideoOutput`` to display the camera images as they are received.
Additionally, it can be configured to show a blank image after x milliseconds using the ``timeout`` property
which is set to 3000ms (3s) by default. This can be disabled by setting the ``timeout`` to 0.

The QML ROS2 plugin internally keeps track of all image transport subscriptions and will only create one
shared subscription per topic to avoid unnecessary computation and bandwidth overhead.

API
---

.. doxygenclass:: qml6_ros2_plugin::ImageTransportSubscription
   :members:

