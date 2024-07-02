#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <object_detection_msgs/ObjectDetectionInfoArray.h>

namespace object_processor {

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class ObjectProcessor
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  ObjectProcessor(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~ObjectProcessor();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */
  void infoCallback(const object_detection_msgs::ObjectDetectionInfoArray& message);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! ROS topic subscriber.
  ros::Subscriber subscriber_;

  //! ROS topic name to subscribe to.
  std::string subscriberTopic_;
};

} /* namespace */