#include <ros/ros.h>
#include <motion_decoder/image_converter.hpp>
#include <apriltags_ros/AprilTagDetectionArray.h>
#include <apriltags_ros/AprilTagDetection.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>


ImageConverter* ic;

void apriltag_detection_callback(const apriltags_ros::AprilTagDetectionArray msg)
{
  ROS_INFO("In subscribe\n");
  //msg.id, msg.size, msg.pose
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(msg.detections[0].pose.pose.position.x, msg.detections[0].pose.pose.position.y, msg.detections[0].pose.pose.position.z));
  tf::Quaternion q;
  transform.setRotation	(msg.detections[0].pose.pose.orientation)
  br.sendTransform(tf::StampedTransform(transform, "camera", "april_tf"));
  //TODO: Parse message and publish transforms as apriltag_tf and camera
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  
  ros::NodeHandle n;
  //TODO: Add a subscriber to get the AprilTag detections The callback function skelton is given.
  ros::Subscriber sub = n.subscribe("tag_detections_pose", 1000, apriltag_detection_callback);

  ImageConverter converter;
  ic = &converter;
  ros::Rate loop_rate(50);
  ROS_INFO("In main\n");
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
