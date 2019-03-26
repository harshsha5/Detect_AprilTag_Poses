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
  tf::Quaternion quat_tf;
  quaternionMsgToTF(msg.detections[0].pose.pose.orientation , quat_tf);
  transform.setRotation	(quat_tf);

  br.sendTransform(tf::StampedTransform(transform,ros::Time::now(),"camera", "april_tf"));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");
  
  ros::NodeHandle n;
  //TODO: Add a subscriber to get the AprilTag detections The callback function skelton is given.
  ros::Subscriber sub = n.subscribe("tag_detections", 1000, apriltag_detection_callback);

  ImageConverter converter;
  ic = &converter;
  ic->setTagLocations()
  ros::Rate loop_rate(50);
  ROS_INFO("In main\n");
  while(ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
