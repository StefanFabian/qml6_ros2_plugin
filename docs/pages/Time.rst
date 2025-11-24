====
Time
====

To preserve the accuracy and allow for compatible serialization of message objects, anonymous wrappers for
:cpp:class:`Time <qml6_ros2_plugin::Time>` and :cpp:class:`Duration <qml6_ros2_plugin::Duration>` were
introduced.
These wrapper types are used for time and duration fields in received messages and the
current time can be obtained using the ``Ros2.now()`` method of the :ref:`Ros2 Singleton`.

Example:

.. code-block:: qml

  property var currentTime: Ros2.now()

Both wrapper types can be converted to QML/JavaScript ``Date`` objects using the
:cpp:func:`toJSDate() <qml6_ros2_plugin::Time::toJSDate>` method at the cost of micro- and nanosecond accuracy.

Please note that due to limitations in QML and JavaScript mathematical operations for Time and Duration are not possible.

Time and Duration have a ``sec`` and a ``nanosec`` property representing the seconds and nanoseconds part of the time or
duration similar to the ROS 2 message type.
While they can be modified, there is no change notification emitted when doing so due to limitations of the Qt type used.
Also note that ``nanosec`` is the time or duration part of the second and not a total nanosecond count.
If you require the total nanosecond count, use the ``nanoseconds()`` method.

API
---
.. doxygenclass:: qml6_ros2_plugin::Time
  :members:

.. doxygenclass:: qml6_ros2_plugin::Duration
  :members:

.. doxygenenum:: qml6_ros2_plugin::ros_clock_types::Ros2ClockTypes
   :project: project
