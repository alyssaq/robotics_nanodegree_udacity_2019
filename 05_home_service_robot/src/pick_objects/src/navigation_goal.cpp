#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

const int NUM_GOALS = 2;
const double GOALS[NUM_GOALS][2] = {
  {7.0, 1.0},
  {3.1, 1.6}
};

bool move_to_position(double pos_x, double pos_y) {
  // define a client for to send goal requests to the move_base server through a SimpleActionClient
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  // wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // send positions absolute to map. For relative to robo, use robot_footprint
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // position and orientation for the robot to reach
  goal.target_pose.pose.position.x = pos_x;
  goal.target_pose.pose.position.y = pos_y;
  goal.target_pose.pose.position.z =  0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal location at %1.2f %1.2f", pos_x, pos_y);
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Robo has reached the destination");
    return true;
  }
  else {
    ROS_INFO("Robo failed to reach the destination");
    return false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Rate r(1);

  if (!ros::ok()) {
    ROS_WARN("ROS is not OK");
    return 0;
  }

  for (int i = 0; i < NUM_GOALS; i++) {
    const bool ok = move_to_position(GOALS[i][0], GOALS[i][1]);
    if (!ok) {
      break;
    }
    sleep(5);
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
