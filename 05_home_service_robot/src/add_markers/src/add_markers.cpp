#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// Global state variable of marker where 0 = hide, 1 = show.
uint16_t state = 1;
visualization_msgs::Marker marker;
ros::Publisher marker_pub;

int num_goals = 2;
uint16_t goal_idx = 0;
const double PICKUP_POS_X = 7.0;
const double PICKUP_POS_Y = 1.0;
const double DROPOFF_POS_X = 3.1;
const double DROPOFF_POS_Y = 1.6;
const double GOALS[2][2] = {
  {PICKUP_POS_X, PICKUP_POS_Y},
  {DROPOFF_POS_X, DROPOFF_POS_Y}
};


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
  marker.color.r = 0.0f;
  marker.color.g = 0.0f;
  marker.color.b = 0.5f;
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
  if (goal_idx >= num_goals) {
    return;
  }

  const double pos_x = msg->pose.pose.position.x;
  const double pos_y = msg->pose.pose.position.y;
  const double goal_x = GOALS[goal_idx][0];
  const double goal_y = GOALS[goal_idx][1];
  const bool is_pickup = goal_idx % 2 == 0;
  const double drift = 0.1;
  const bool is_at_goal = is_within_goal_bounds(pos_x, pos_y, goal_x, goal_y, drift);

  ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", pos_x, pos_y, msg->pose.pose.position.z);
  // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

  if (is_pickup && !is_at_goal) {
    // Heading to pick up, publish marker there
    add_marker(goal_x, goal_y);
  } else if (is_pickup && is_at_goal) {
    // Reached pick up, delete marker
    delete_marker();
    goal_idx++;
  } else if (!is_pickup && is_at_goal) {
    // Reached Drop off. leave marker here.
    add_marker(goal_x, goal_y);
    goal_idx++;
  }


  // if (state == 0 && pos_x >= PICKUP_POS_X-drift && pos_x <= PICKUP_POS_X+drift && pos_y >= PICKUP_POS_Y-drift && pos_y <= PICKUP_POS_Y+drift) {
  //   // Arrived at pick up location
  //   marker.action = visualization_msgs::Marker::DELETE;
  //   ROS_INFO("Deleting marker");
  //   state = 1;
  // } else if (state == 1 && pos_x >= DROPOFF_POS_X-drift && pos_x <= DROPOFF_POS_X+drift && pos_y >= DROPOFF_POS_Y-drift && pos_y <= DROPOFF_POS_Y+drift) {
  //   // Arrived at drop off location
  //   add_marker(pos_x, pos_y);
  // }

  marker_pub.publish(marker);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;

  marker.header.frame_id = "map";
  marker.ns = "basic_shapes";
  marker.id = 0; // unique ID in this namespace

  ros::Subscriber pose_sub = n.subscribe("amcl_pose", 2, pose_callback);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // add_marker(7.0, 1.0);

  ros::spin();
}
