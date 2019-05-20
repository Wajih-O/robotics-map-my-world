#define _USE_MATH_DEFINES

#include <cmath>

#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#include <tuple>
#include <map>
#include <std_msgs/Float64.h>

// Define a global client that can request services
ros::ServiceClient client;
ros::Publisher camera_platform_joint_pub;

enum POS {Left, Middle, Right};

// a map to translate pos to drive command
std::map<POS, std::pair<float, float>> pos2action;

// A helper function to get the ball position as POS
POS get_pixel_pos(int pixel_w_pos, std::tuple<int, int> regions_limits) {
    if (pixel_w_pos < std::get<0>(regions_limits)) {
        return POS::Left;
    }
    if (pixel_w_pos < std::get<1>(regions_limits)) {
        return POS::Middle;
    }
    return POS::Right;
}

void move_camera_platform(float angle) {
     std_msgs::Float64 camera_platform_joint_angle;
     camera_platform_joint_angle.data = angle;
     camera_platform_joint_pub.publish(camera_platform_joint_angle);

    // Return a response message
    auto msg_feedback = "Joint angles set - camera_platform_joint: " + std::to_string(angle) ;
    ROS_INFO_STREAM(msg_feedback.c_str());
}


// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget dtt;
    dtt.request.linear_x = lin_x;
    dtt.request.angular_z = ang_z;
    if (!client.call(dtt)) {
        ROS_ERROR("Failed to request the service !");
    }
}


void scan_around() {
    // for (int index = 0; index<4; ++index) {
        // Todo calibrate motors speed and then explore only with camera
        // then rotate the robot where the camera has found the ball and the camera back to the angle 0
        // move_camera_platform(float(1+index)*M_PI/2);

    // }

    drive_robot(0, 0.3);
    ros::Duration(2).sleep();
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    // compute image region limits
    auto third_width = img.width / 3;
    std::tuple<int,int> regions_limits = {third_width, 2*third_width};

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera

    // TODO: refactor using boost optional

    int i = 0;
    int color_depth = 3;
    while(i < img.height) {
        int j=0;
        while(j < img.step) {
            auto R = img.data[i*img.step + j];
            auto G = img.data[i*img.step + j + 1];
            auto B = img.data[i*img.step + j + 2];

            if ( (R == 255) && (G == 255) && (B == 255)) {
             // ball found
            POS ball_pos = get_pixel_pos(j/color_depth, regions_limits);
            std::string msg = "Ball found ! image position :" + std::to_string(ball_pos) + \
            ",  pixel (data) value: (" + std::to_string(R) + "," + std::to_string(G) +"," + std::to_string(B) + ")";
             ROS_INFO_STREAM(msg.c_str());
             auto action = pos2action[ball_pos]; // get the right actuation/twist
             drive_robot(std::get<0>(action), std::get<1>(action)); // drive with actuation
             return ;
            }
            j = j + color_depth;
        }
        ++ i;
    }
    // ball (as white pixel not found so request stop)
    drive_robot(.0, .0);
    scan_around();
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    pos2action[POS::Left] = std::make_pair(0.1, 0.1 );
    pos2action[POS::Middle] = std::make_pair(0.15, 0.0 );
    pos2action[POS::Right] = std::make_pair(0.1, -0.1 );

    // Define a publisher for
    camera_platform_joint_pub = n.advertise<std_msgs::Float64>("/my_robot/camera_platform_joint_position_controller/command", 10);

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}