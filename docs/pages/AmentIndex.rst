=============
Ament Index
=============

The Ament Index is a system used in ROS 2 to locate resources such as packages, libraries, and other files.
It is available in QML through the ``AmentIndex`` singleton.

Example

.. code-block:: qml

  ComboBox {
    id: packageSelector
    model: AmentIndex.getPackages()
    onCurrentTextChanged: {
      let packageShare = AmentIndex.getPackageSharePath(currentText)
      console.log("Package share directory for", currentText, "is", packageShare)
    }
  }

Resources
---------

Arbitrary resources registered in the index can be queried with ``getResource(resourceType, resourceName)``.
It returns an object with the resource's ``content`` and the ``prefix`` of the install location it was found in,
or ``undefined`` if the resource does not exist.
Type and name may only contain letters, digits, ``_`` and ``-``.

.. code-block:: qml

  let resource = AmentIndex.getResource("package_run_dependencies", "my_package")
  if (resource !== undefined)
    console.log("Found in", resource.prefix, "with content:", resource.content)

API
---

.. doxygenclass:: qml6_ros2_plugin::AmentIndex
   :members:
