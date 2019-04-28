#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

bool moveToPosition(double xPos, double yPos) {
  // define a client for to send goal requests to the move_base server through a SimpleActionClient
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
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

  ROS_INFO("Sending goal location ...");
  ac.sendGoal(goal);

  ac.waitForResult();

  if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("You have reached the destination at %1.2f %1.2f", xPos, yPos);
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
int goal = 1;
  while (ros::ok()) {
    if (goal == 1) {
      if (moveToPosition(7.0, 3.0)) {
        ROS_INFO("next one");
        goal = 0;
      };
    } else {
      moveToPosition(0.0, 0.0);
      goal = 1;
    }
    sleep(5);
    ros::spinOnce();
    r.sleep();
  }


  return 0;
}
