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

Quick install all dependencies for ROS 2 jazzy (otherwise replace jazzy with your distro)

```
sudo apt install qmlscene-qt6 qml6-module-qtmultimedia qml6-module-qtquick qml6-module-qtquick-controls qml6-module-qtquick-layouts qml6-module-qtquick-window ros-jazzy-turtle-tf2-py
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
