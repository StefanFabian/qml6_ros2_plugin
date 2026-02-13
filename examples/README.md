# Examples

Examples on how to use the QML ROS Plugin

| Dependencies     |                                                 |
|------------------|-------------------------------------------------|
| qmlscene         | `sudo apt install qmlscene-qt6`                 |
| QtMultimedia     | `sudo apt install qml6-module-qtmultimedia`     |
| QtQuick          | `sudo apt install qml6-module-qtquick`          |
| QtQuick.Controls | `sudo apt install qml6-module-qtquick-controls` |
| QtQuick.Layouts  | `sudo apt install qml6-module-qtquick-layouts`  |
| QtQuick.Window   | `sudo apt install qml6-module-qtquick-window`   |
| USB Cam          | `sudo apt install ros-${ROS_DISTRO}-usb-cam`    |

Quick install all dependencies for ROS 2 jazzy (otherwise replace jazzy with your distro)

```
sudo apt install qmlscene-qt6 qml6-module-qtmultimedia qml6-module-qtquick qml6-module-qtquick-controls qml6-module-qtquick-layouts qml6-module-qtquick-window ros-jazzy-turtle-tf2-py ros-jazzy-usb-cam
```

Run using: `qmlscene -qt=qt6 FILE`

**Example:**

```
qmlscene -qt=qt6 publisher.qml
```

----------
**Note:**
For the `tf_transforms.qml` and `turtle_demo_control.qml`, you can use the following simulation:

```
ros2 launch turtle_tf2_py turtle_tf2_demo.launch.py
```

For the `image_subscription.qml`, you can use the usb_cam package and run

```
ros2 run usb_cam usb_cam_node_exe --ros-args -p pixel_format:=mjpeg2rgb
```

to stream your webcam.

**Troubleshooting:** If you get an error that compressed is not available, you're missing the `image_transport_plugins`.

Install on Ubuntu using

```
sudo apt install ros-{DISTRO}-image-transport-plugins
```
