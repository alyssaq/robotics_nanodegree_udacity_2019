#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

const double PICKUP_POS_X = 7.0;
const double PICKUP_POS_Y = 1.0;
const double DROPOFF_POS_X = 3.1;
const double DROPOFF_POS_Y = 1.6;

bool moveToPosition(double xPos, double yPos) {
  // define a client for to send goal requests to the move_base server through a SimpleActionClient
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // send positions absolute to map. For relative to robo, use robot_footprint
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // position and orientation for the robot to reach
  goal.target_pose.pose.position.x = xPos;
  goal.target_pose.pose.position.y = yPos;
  goal.target_pose.pose.position.z =  0.0;
  goal.target_pose.pose.orientation.x = 0.0;
  goal.target_pose.pose.orientation.y = 0.0;
  goal.target_pose.pose.orientation.z = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal location at %1.2f %1.2f", xPos, yPos);
  ac.sendGoal(goal);
  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("You have reached the destination");
    return true;
  }
  else {
    ROS_INFO("The robot failed to reach the destination");
    return false;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Rate r(1);

  // uint16_t goal = 1;
  int num_goals = 2;
  const double goals[num_goals][2] = {
    {PICKUP_POS_X, PICKUP_POS_Y},
    {DROPOFF_POS_X, DROPOFF_POS_Y}
  };

  while (ros::ok()) {
    for (int i = 0; i < num_goals; i++) {
      const bool ok = moveToPosition(goals[i][0], goals[i][1])
      if (!ok) {
        break
      }
      sleep(5);
      ros::spinOnce();
      r.sleep();
    }
    // if (goal == 1) {
    //   if (moveToPosition(PICKUP_POS_X, PICKUP_POS_Y)) {
    //     goal = 0;
    //   } else {
    //     break;
    //   };
    // } else {
    //   moveToPosition(DROPOFF_POS_X, DROPOFF_POS_Y);
    //   break;
    // }

    // sleep(5);
    // ros::spinOnce();
    // r.sleep();
  }

  return 0;
}
