=============
Tf Transforms
=============

There are two methods for looking up **tf2** transforms.

Component
---------
The ``TfTransform`` component can be used to subscribe to transforms between
two frames.

.. code-block:: qml

  TfTransform {
    id: tfTransform
    active: true // This is the default, if false no updates will be received
    sourceFrame: "turtle1"
    targetFrame: "world"
  }

It provides a ``valid`` property that indicates if a valid transform has been received.
If it is valid, it contains a ``transform`` property with the stamped transform ``geometry_msgs/msg/TransformStamped``
and for convenience also a ``translation`` and a ``rotation`` property which refer to the translation and rotation in
the transform.

Using the ``rate`` property, you can also change the maximum rate at which the transform is updated.

Static
------
You can also use the ``TfTransformListener`` singleton to look up transforms if you just need it once.

.. code-block:: qml

  Button {
    text: "Look Up"
    onClicked: {
      var transformStamped = TfTransformListener.lookUpTransform(inputTargetFrame.text, inputSourceFrame.text)
      if (!transformStamped.valid)
      {
        transformResult.text = "Transform from '" + inputSourceFrame.text + "' to '" + inputTargetFrame.text + "' was not valid!\n" +
                                "Exception: " + transformStamped.exception + "\nMessage: " + transformStamped.message
        return
      }
      transformResult.text = "Position:\n" + printVector3(transformStamped.transform.translation) + "\nOrientation:\n" + printRotation(transformStamped.transform.rotation)
    }
  }

Use the provided ``Ros2.now()`` static methods to look up at specific time
points. For the latest, you can pass ``new Date(0)``.
Be aware that in JavaScript durations are given in milliseconds.


.. Warning:: Be aware that `canLookUp` can return a ``boolean`` value or
  a ``string`` error message. You should explicitly test for that since strings
  are truthy, too.

Namespaced
----------
The global ``TfTransformListener`` singleton follows the design of ``tf2_ros::TransformListener``
and therefore always subscribes to ``/tf`` and ``/tf_static``. For multi-robot
setups where each robot publishes its own tf tree under a namespace
(``<namespace>/tf`` and ``<namespace>/tf_static``), use the ``TfBuffer``
component. Each instance owns its own tf buffer fed from the topics under the
configured namespace and exposes the same ``canTransform`` / ``lookUpTransform``
API as the singleton.

.. code-block:: qml

  TfBuffer {
    id: robot1Buffer
    ns: "/robot1" // subscribes to /robot1/tf and /robot1/tf_static
  }

  TfBuffer {
    id: robot2Buffer
    ns: "/robot2" // subscribes to /robot2/tf and /robot2/tf_static
  }

  TfTransform {
    buffer: robot1Buffer
    sourceFrame: "base_link"
    targetFrame: "map"
  }

An empty ``ns`` (the default) subscribes to ``/tf`` and ``/tf_static``
and therefore behaves like the global singleton path.

Passing an instance to a ``TfTransform`` via its ``buffer`` property sources
transforms from that buffer instead of the global singleton. Leaving ``buffer``
unset (the default) keeps the existing behavior of using the global
``TfTransformListener``.

The same ``lookUpTransform`` / ``canTransform`` methods that are available on
the ``TfTransformListener`` singleton can also be called directly on a
``TfBuffer`` instance.
Additionally, you can query the authority that provided the transform for a
frame using ``getFrameAuthority(frame_id)``.

Frame Queries
-------------
Both the ``TfTransformListener`` singleton and ``TfBuffer`` instances expose
methods for querying individual frames or the entire frame tree. Each frame is
represented by a ``TfFrameInfo`` value object with the following properties:

- ``frameId`` — the frame ID string
- ``parentId`` — the parent frame ID (empty for root frames)
- ``authority`` — the node name that publishes the transform for this frame
- ``translation`` — a map ``{x, y, z}`` of the translation from parent to frame
- ``rotation`` — a quaternion map ``{w, x, y, z}`` from parent to frame
- ``isStatic`` — ``true`` if the transform was received on ``/tf_static``
- ``children`` — a list of direct child frame IDs
- ``frequency`` — the publishing rate in Hz (computed over a 5-second window)

.. code-block:: qml

  // Query a single frame
  var frame = TfTransformListener.getFrame("base_link")
  if (frame) {
      console.log("Parent:", frame.parentId)
      console.log("Authority:", frame.authority)
      console.log("Frequency:", frame.frequency.toFixed(1), "Hz")
      console.log("Children:", frame.children)
  }

  // Enumerate all frames
  var allFrames = TfTransformListener.getAllFrames()
  for (var i = 0; i < allFrames.length; ++i) {
      var f = allFrames[i]
      console.log(f.frameId, "->", f.parentId)
  }

  // Get the age of a transform in seconds
  var age = TfTransformListener.getTransformAge("base_link")
  if (age >= 0)
      console.log("Transform age:", age.toFixed(3), "s")

The same methods are available on ``TfBuffer`` instances:

.. code-block:: qml

  TfBuffer {
      id: robotBuffer
      ns: "/robot1"
  }

  // ...
  var frame = robotBuffer.getFrame("base_link")
  var all = robotBuffer.getAllFrames()
  var age = robotBuffer.getTransformAge("base_link")

API
---

.. doxygenclass:: qml6_ros2_plugin::TfTransformListenerWrapper
  :members:

.. doxygenclass:: qml6_ros2_plugin::TfBuffer
  :members:

.. doxygenclass:: qml6_ros2_plugin::TfFrameInfo
  :members:

.. doxygenclass:: qml6_ros2_plugin::TfTransform
  :members:
