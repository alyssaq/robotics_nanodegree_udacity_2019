#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving robot");
    ROS_INFO("Driving robot with velocity linear:%1.2f, angular:%1.2f", lin_x, ang_z);

    // Request velocity
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    if (!client.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img) {
    int white_pixel = 255;
    int white_ball_pos = -1;
    int left = static_cast<int>(img.width / 3);
    int right = left * 2;

    // Loop through each pixel in the image and check if there's a white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image

    for (int i = 0; i < img.height * img.step; i++) {
        if (img.data[i] == white_pixel) {
            white_ball_pos = i % img.width;
            break;
        }
    }

    // Depending on the white ball position, call the drive_robot function and pass velocities
    // Request a stop when there's no white ball seen by the camera
    if (white_ball_pos < 0) { // stop
      drive_robot(0.0, 0.0);
    } else if (white_ball_pos < left) {
      drive_robot(0.0, 0.5);
    } else if (white_ball_pos > right) {
      drive_robot(0.0, -0.5);
    } else { // forward
      drive_robot(0.5, 0.0);
    }
}

int main(int argc, char** argv) {
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
