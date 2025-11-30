## Changes coming from qml_ros2_plugin

### Time

* The `Time` and `Duration` types now use `sec` and `nanosec` as properties
  similar to the ROS2 message definitions. The previous properties `seconds`,
  `nanoseconds`, and `clockType` were removed and are now available as methods.
  This change was done to make the types more consistent with the message.

### ImageTransport

* Due to changes from Qt5 to Qt6, a rewrite was necessary. For the usage the only
  change is that you now have to set the `videoSink` property of the `ImageTransportSubscription`
  instead of using it as a source for the `VideoOutput`.  
  See the `image_subscription.qml` example for details.
