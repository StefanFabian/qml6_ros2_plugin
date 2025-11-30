import QtQuick
import QtQuick.Controls
import QtQuick.Controls.Material
import QtQuick.Layouts
import QtMultimedia
import Ros2

ApplicationWindow {
  id: page
  width: 620
  height: 400

  // This connection makes sure the application exits if this ROS node is requested to shutdown
  Connections {
    target: Ros2

    function onShutdown() {
      Qt.quit()
    }
  }

  ImageTransportSubscription {
    id: imageSubscription
    // Enter a valid image topic here
    topic: "/image_raw"
    // This is the default transport, typically, I would advise to use a compressed stream.
    // However, at least for yuyv and uyvy the compressed transport is bugged.
    defaultTransport: "raw"
    videoSink: output.videoSink
  }
  ColumnLayout {
    anchors.fill: parent
    anchors.margins: 12
    RowLayout {
      Label {
        text: "Topic:"
      }
      ComboBox {
        id: topicSelector
        Layout.fillWidth: true
        model: ["/image_raw"]

        function refresh() {
          let topics = Ros2.queryTopics("sensor_msgs/msg/Image")
          if (topics.length === 0) {
            topics = ["/image_raw"]
          }
          model = topics
        }

        onCurrentTextChanged: {
          if (!Ros2.isValidTopic(currentText)) return
          imageSubscription.topic = currentText
          Ros2.info("Switched to topic: " + currentText)
        }
        Component.onCompleted: refresh()
      }
      Button {
        text: "Refresh"
        onClicked: topicSelector.refresh()
      }
    }
    VideoOutput {
      Layout.fillHeight: true
      id: output
      // Can be used in increments of 90 to rotate the video
      orientation: 0
    }
    Text {
      text: "If you don't have any image topics, run the following to use your webcam:"
    }
    TextInput {
      readOnly: true
      selectByMouse: true
      text: "ros2 run usb_cam usb_cam_node_exe"
    }
  }

  Component.onCompleted: {
    // Initialize ROS with the given name. The command line args are passed by the plugin
    // Optionally, you can call init with a string list ["arg1", "arg2"] after the name to use those
    // args instead of the ones supplied by the command line.
    Ros2.init("qml_image_subscription_demo")
    //Ros2.getLogger("qml6_ros2_plugin").setLoggerLevel(Ros2LoggerLevel.Debug)
  }

}
