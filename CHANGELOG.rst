^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qml6_ros2_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.26.10 (2026-01-08)
--------------------
* Fixed segfault if Service or ActionClient are no longer associated with qjsEngine when callback is invoked.
* Improved image conversion to also deal with infinite values in depth images.
* Fixed crash when ServiceClient is processing request while client is destroyed.
  Use QPointer for callbacks to prevent crashes due to the object being destroyed while an asynchronous operation is still in progress.
* Contributors: Stefan Fabian

1.25.121 (2025-12-12)
---------------------
* Added encoding information to ImageTransportSubscription and fixed conversion from float to Y16.
  In accordance with depth image standards in ROS 16UC1 is interpreted as mm whereas float is in m. The conversion now respects that.
* Contributors: Stefan Fabian

1.25.120 (2025-12-08)
---------------------
* Initial release.
* Contributors: Stefan Fabian
