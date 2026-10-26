==============
Ros2 Singleton
==============

The ``Ros2`` singleton provides interfaces to static methods and convenience
methods.

In QML it is available as ``Ros2``, e.g.:

.. code-block:: qml

  if (Ros2.ok()) console.log("Ros2 is ok!")

ROS Initialization
------------------

First, you need to initialize the node used by your QML application, e.g., in the ``onCompleted`` handler:

.. code-block:: qml

  Component.onCompleted: {
    Ros2.init("node_name")
  }

You can also conditionally initialize by checking if it was already initialized using ``Ros2.isInitialized``.
As described in the API documentation for :cpp:func:`Ros2.init <qml6_ros2_plugin::Ros2QmlSingletonWrapper::init>`, you can pass either just the
node name or additionally use provided command line args instead of the command
line args provided to your executable.

Additionally, you can use the init options to set options for the node, such as the namespace or domain id.

.. code-block:: qml

  Component.onCompleted: {
    let initOptions = Ros2.createInitOptions()
    initOptions.setNamespace("/my_namespace")
    initOptions.setDomainId(42) // or initOptions.useDefaultDomainId()
    Ros2.init("node_name", initOptions)
  }

Before application exit make sure to call ``Ros2.shutdown()`` to cleanly shutdown the node.
In Qt application this should be done automatically, if it is not, you might get a warning in the console and the
application could crash due to the middleware being destructed before the ROS 2 qml plugin cleaned up.

Query Graph
------------

You can also use the Ros2 singleton to query the available topics, services and actions.
These are available as:

* | ``QVariantMap getTopicNamesAndTypes()``
  | Retrieves a map of topic names and their types as a list of strings.
* | ``QVariantMap getServiceNamesAndTypes()``
  | Retrieves a map of service names and their types as a list of strings.
* | ``QVariantMap getActionNamesAndTypes()``
  | Retrieves a map of action names and their types as a list of strings.

Note that the return value is a ``QVariantMap``, which is a wrapped ``QMap<QString, QStringList>``
to make it compatible with QML.
Also check out the ``graph_queries.qml`` example in the repo's examples folder.

Additionally, for topics three convenience methods are also provided:

* | ``QStringList queryTopics( const QString &datatype = QString())``
  | Queries a list of topics with the given datatype or all topics if no type provided.
* | ``QList<TopicInfo> queryTopicInfo()``
  | Retrieves a list of all advertised topics including their datatypes. See :cpp:class:`TopicInfo`
* | ``QStringList queryTopicTypes( const QString &name )``
  | Retrieves the datatypes for a given topic.

And for actions and services:

* | ``QStringList queryServices( const QString &datatype = QString())``
  | Queries a list of services with the given datatype or all services if no type provided.
* | ``QStringList queryActions( const QString &datatype = QString())``
  | Queries a list of actions with the given datatype or all actions if no type provided.

Example:

.. code-block:: qml

  // Retrieves a list of topics with the type sensor_msgs/Image
  var topics = Ros2.queryTopics("sensor_msgs/msg/Image")
  // Another slower and less clean method of this would be
  var cameraTopics = []
  var topics = Ros2.queryTopicInfo()
  for (var i = 0; i < topics.length; ++i) {
    if (topics[i].datatypes.includes("sensor_msgs/msg/Image")) cameraTopics.push(topics[i].name)
  }
  // The types of a specific topic can be retrieved as follows
  var datatypes = Ros2.queryTopicTypes("/topic/that/i/care/about")
  // Using this we can make an even worse implementation of the same functionality
  var cameraTopics = []
  var topics = Ros2.queryTopics() // Gets all topics
  for (var i = 0; i < topics.length; ++i) {
    if (Ros2.queryTopicTypes(topics[i]).includes("sensor_msgs/msg/Image")) cameraTopics.push(topics[i])
  }

The nodes in the graph and the interfaces of a single node can be queried using:

* | ``QVariantList getNodeNamesAndNamespaces()``
  | Retrieves a list of all nodes. Each entry has the keys ``name``, ``namespace`` and ``fullName``.
* | ``QVariantMap getNodeInfo( const QString &name, const QString &ns )``
  | Retrieves the node with the given name (without namespace) and namespace, e.g., ``"/"`` or ``"/my_ns"``.
    The result has the keys of a ``getNodeNamesAndNamespaces`` entry and additionally ``publishers``,
    ``subscriptions``, ``serviceServers``, ``serviceClients``, ``actionServers`` and ``actionClients``,
    each a list of entries with the keys ``name`` and ``datatypes``.
    The internal topics and services of actions are not included.
    Returns an empty map if the node is not in the graph or, with a logged warning, if the name or namespace
    is invalid, e.g., the full name was passed as ``name``.
* | ``QVariantList getAllNodeInfo()``
  | Retrieves all nodes, each entry as returned by ``getNodeInfo``.

The endpoints on a topic, e.g., to find out which nodes are connected and with which QoS settings, can be queried using:

* | ``QVariantList getPublishersInfoByTopic( const QString &topic )``
  | Retrieves the publishers on the given fully qualified topic, e.g., ``/front_camera/image_raw``.
    Each entry has the keys ``nodeName``, ``nodeNamespace``, ``datatype`` and ``qos``, its QoS settings,
    see `Quality of Service`_.
    These are the settings reported by the middleware (RMW). The reported history and depth are RMW dependent,
    e.g., Fast DDS reports ``HistoryPolicy.Unknown`` and a depth of 0.
    Returns an empty list with a logged warning if the topic name is invalid or not fully qualified.
* | ``QVariantList getSubscriptionsInfoByTopic( const QString &topic )``
  | Retrieves the subscriptions on the given topic with the same keys as ``getPublishersInfoByTopic``.

Example:

.. code-block:: qml

  var nodes = Ros2.getAllNodeInfo()
  for (var i = 0; i < nodes.length; ++i) {
    for (var j = 0; j < nodes[i].publishers.length; ++j) {
      var topic = nodes[i].publishers[j].name
      var subscriptions = Ros2.getSubscriptionsInfoByTopic(topic)
      var reliable = subscriptions.filter(sub => sub.qos.reliability === ReliabilityPolicy.Reliable)
      console.log(nodes[i].fullName + " publishes " + topic + " to " + reliable.length + " reliable subscribers")
    }
  }

The ``graphChanged`` signal is emitted when the graph changed, e.g., a node, publisher, subscription, service or
action appeared or vanished, including those of this application.
Changes are combined, so it is emitted at most every 100 ms.
Use it to refresh queried graph information only when needed instead of polling:

.. code-block:: qml

  Connections {
    target: Ros2
    function onGraphChanged() { nodeList.model = Ros2.getNodeNamesAndNamespaces() }
  }

Quality of Service
------------------
QoS settings are created using ``Ros2.QoS()`` or one of the preset profiles, e.g., ``Ros2.SensorDataQoS()``,
and configured by chaining, e.g., ``Ros2.QoS().reliable().transient_local().keep_last(10)``.

The settings of a QoS object can be read using the read-only ``reliability``, ``durability``, ``history`` and ``depth``
properties. The policies can be compared to the ``ReliabilityPolicy``, ``DurabilityPolicy`` and ``HistoryPolicy`` enums.

.. code-block:: qml

  var qos = Ros2.QoS().reliable().transient_local().keep_last(5)
  if (qos.reliability === ReliabilityPolicy.Reliable && qos.durability === DurabilityPolicy.TransientLocal)
    console.log("Late joining subscribers receive up to " + qos.depth + " messages.")

Create Empty Message
--------------------
You can also create empty messages and service requests as javascript objects using the ``Ros2`` singleton.

.. code-block:: qml

  var message = Ros2.createEmptyMessage("geometry_msgs/msg/Point")
  // This creates an empty instance of the mssage, we can override the fields
  message.x = 1; message.y = 2; message.z = 1

  // Same can be done with service requests
  var serviceRequest = Ros2.createEmptyServiceRequest("std_srvs/srv/SetBool")
  // This creates an empty instance of the service request with all members set to their
  // default, we can override the fields
  serviceRequest.data = true

Logging
-------
The Ros2 singleton also provides access to the ``Ros2`` logging functionality.
See `Logging`:ref:.

IO
--
You can also save and read data that can be serialized in the yaml format using:

.. code-block:: qml

  var obj = {"key": [1, 2, 3], "other": "value"}
  if (!Ros2.io.writeYaml("/home/user/file.yaml", obj))
    Ros2.error("Could not write file!")
  // and read it back
  obj = Ros2.io.readYaml("/home/user/file.yaml")
  if (!obj) Ros2.error("Failed to load file!")

API
---

.. doxygenclass:: qml6_ros2_plugin::Ros2InitOptions
  :members:

.. doxygenclass:: qml6_ros2_plugin::TopicInfo
  :members:

.. doxygenclass:: qml6_ros2_plugin::IO
  :members:

.. doxygenclass:: qml6_ros2_plugin::QoSWrapper
  :members:

.. doxygenenum:: qml6_ros2_plugin::qos_reliability_policy::ReliabilityPolicy
   :project: project

.. doxygenenum:: qml6_ros2_plugin::qos_durability_policy::DurabilityPolicy
   :project: project

.. doxygenenum:: qml6_ros2_plugin::qos_history_policy::HistoryPolicy
   :project: project

.. doxygenclass:: qml6_ros2_plugin::Ros2QmlSingletonWrapper
  :members:
