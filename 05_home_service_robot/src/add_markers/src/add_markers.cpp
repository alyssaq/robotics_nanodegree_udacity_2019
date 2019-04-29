#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Global state variable of marker where 0 = hide, 1 = show.
uint16_t state = 0;
visualization_msgs::Marker marker;

void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  if (state == 0) {
    marker.header.stamp = ros::Time::now();

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.25;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.3f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    ROS_INFO("Publishing marker");
    state = 1;
  } else {
    marker.action = visualization_msgs::Marker::DELETE;

    ROS_INFO("Deleting marker");
    state = 0;
  }

  marker_pub.publish(marker);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  // ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber pose_sub = n.subscribe("amcl_pose", 2, poseCallback);

  marker.header.frame_id = "map";
  marker.ns = "basic_shapes";
  marker.id = 0; // unique ID in this namespace
  marker.type = visualization_msgs::Marker::SPHERE; //CYLINDER;

  ros::spin();
}
