^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qml6_ros2_plugin
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.26.31 (2026-03-18)
--------------------
* Added missing build export depends.
* Contributors: Stefan Fabian

1.26.30 (2026-03-09)
--------------------
* Fix yaml conversion not handling QJSValue correctly.
* Updated documentation.
* Fix BGR color swap and added support for NV21. (`#18 <https://github.com/StefanFabian/qml6_ros2_plugin/issues/18>`_)
  * Fix BGR color swap and added support for NV21.
* Use a symlink instead of installing twice. (`#14 <https://github.com/StefanFabian/qml6_ros2_plugin/issues/14>`_)
  * Use a symlink instead of installing twice.
  This fixes issues where an application may link against both files through plugins which separates the singletons as they exist per inode and creates complex issues.
* Don't pass QJSValue between threads (`#9 <https://github.com/StefanFabian/qml6_ros2_plugin/issues/9>`_)
  * Move all QJSValue to data structures and only pass id for retrieval to fix threading segmentation faults.
  Also fixes check service ready being called while object is already destructed.
  * Fixed deprecation of ament_index_cpp methods.  Change ament_index_cpp method based on available version.
* Added funding details.
* Contributors: Stefan Fabian

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
