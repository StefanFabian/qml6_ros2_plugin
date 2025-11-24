## Changes coming from qml_ros2_plugin

### Time

* The `Time` and `Duration` types now use `sec` and `nanosec` as properties
  similar to the ROS2 message definitions. The previous properties `seconds`,
  `nanoseconds`, and `clockType` were removed and are now available as methods.
  This change was done to make the types more consistent with the message.
