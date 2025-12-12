^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qml6_ros2_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.25.122 (2025-12-12)
---------------------
* Added encoding information to ImageTransportSubscription and fixed conversion from float to Y16.
  In accordance with depth image standards in ROS 16UC1 is interpreted as mm whereas float is in m. The conversion now respects that.
* Contributors: Stefan Fabian

2.25.121 (2025-12-08)
---------------------
* Backport for kilted.
* Contributors: Stefan Fabian

2.25.120 (2025-12-08)
---------------------
* Initial release.
* Contributors: Stefan Fabian
