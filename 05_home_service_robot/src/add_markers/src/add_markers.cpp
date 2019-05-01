#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

int GOAL_IDX = 0;
const int NUM_GOALS = 2;
const double GOALS[NUM_GOALS][2] = {
  {7.0, 1.0},
  {3.1, 1.6}
};

visualization_msgs::Marker marker;
ros::Publisher marker_pub;

void add_marker(double pos_x, double pos_y) {
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::SPHERE; //CYLINDER;

  // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
  marker.action = visualization_msgs::Marker::ADD;

  // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  marker.pose.position.x = pos_x;
  marker.pose.position.y = pos_y;
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
  const float color = (float) (GOAL_IDX - GOAL_IDX % 2 + 2) / (float) NUM_GOALS;
  marker.color.r = color;
  marker.color.g = color;
  marker.color.b = color;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();
  ROS_INFO("Publishing marker");
}

void delete_marker() {
  marker.action = visualization_msgs::Marker::DELETE;
  ROS_INFO("Deleting marker");
}

bool is_within_goal_bounds(double pos_x, double pos_y, double goal_x, double goal_y, double drift) {
  return pos_x >= goal_x - drift && pos_x <= goal_x + drift && pos_y >= goal_y - drift && pos_y <= goal_y + drift;
}

void pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
  if (GOAL_IDX >= NUM_GOALS) {
    return;
  }

  const double pos_x = msg->pose.pose.position.x;
  const double pos_y = msg->pose.pose.position.y;
  const double goal_x = GOALS[GOAL_IDX][0];
  const double goal_y = GOALS[GOAL_IDX][1];
  const bool is_pickup = GOAL_IDX % 2 == 0;
  const double drift = 0.1;
  const bool is_at_goal = is_within_goal_bounds(pos_x, pos_y, goal_x, goal_y, drift);

  marker.id = GOAL_IDX - GOAL_IDX % 2 ; // unique ID in this namespace

  ROS_INFO("x: %1.3f, y: %1.3f, at goal: %d", pos_x, pos_y, is_at_goal);
  // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  if (is_pickup && !is_at_goal) {
    // Heading to pick up, publish marker there
    add_marker(goal_x, goal_y);
  } else if (is_pickup && is_at_goal) {
    // Reached pick up, delete marker
    delete_marker();
    GOAL_IDX++;
  } else if (!is_pickup && is_at_goal) {
    // Reached Drop off. leave marker here.
    add_marker(goal_x, goal_y);
    GOAL_IDX++;
  }

  marker_pub.publish(marker);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;

  marker.header.frame_id = "map";
  marker.ns = "basic_shapes";

  ros::Subscriber pose_sub = n.subscribe("amcl_pose", 2, pose_callback);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::spin();
}
