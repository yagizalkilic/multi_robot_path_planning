#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Pose.h>
#include <string>
#include <vector>
#include <thread>
#include <ctime> 
#include "../include/project_utilities.h"
#include <multi_robot_path_scheduling/velocities_durations_orientations.h>


// Global variables to store the subscribed data
std::vector<std::vector<double>> received_velocities;
std::vector<std::vector<double>> received_durations;
std::vector<std::vector<double>> received_orientations;
std::vector<std::vector<double>> received_angular_velocities;

std::vector<turtlesim::Pose> received_poses;

std_msgs::Int8 received_amount;
std_msgs::Int32MultiArray received_schedule_interval;

bool first_initialization_complete = false;
bool second_initialization_complete = false;


/**
 * Teleports the turtles on the given path.
 *
 * @param nh NodeHandle
 * @param path physical path to follow
 * @param schedule schedule to follow the path
 * @param turtle_name id of the turtle
 * @param pose current turtle position
 * @return true if path traversing is completed
 */
bool moveTurtleAlongPath(ros::NodeHandle& nh, std::vector<double> velocities, std::vector<double> orientations, std::vector<double> durations,
    std::vector<double> ang_vels, const std::string& turtle_name, const turtlesim::Pose& pose, ros::Publisher velocity_publisher)
{
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/" + turtle_name + "/teleport_absolute");

    for ( size_t i = 0; i < velocities.size(); i++)
    {
        ros::Time start_time = ros::Time::now();
        turtlesim::TeleportAbsolute teleport_srv;

        teleport_srv.request.x = pose.x;
        teleport_srv.request.y = pose.y;
        teleport_srv.request.theta = orientations[i];

        geometry_msgs::Twist velMsg;
        velMsg.linear.x = velocities[i];
        velMsg.angular.z = 0; 

        teleport_client.call(teleport_srv);
        velocity_publisher.publish(velMsg);

        if (i == velocities.size() - 2)
        {
            ROS_INFO("Turtle %s reached the last coordinate, stopping.", turtle_name.c_str());

            geometry_msgs::Twist stopMsg;
            stopMsg.linear.x = 0;
            stopMsg.angular.z = 0;

            velocity_publisher.publish(stopMsg);

            return true;
        }

        // Sleep to simulate the turtle moving along the path
        ros::Duration(durations[i]).sleep();
        ros::Time end_time = ros::Time::now();
        ros::Duration time_spent = end_time - start_time;
        ROS_INFO("Time spent in the loop: %f seconds", time_spent.toSec());
    }
}

/**
 * Run each turtle on a thread.
 *
 * @param nh NodeHandle
 * @param turtle_index position of turtle in turtle_list
 * @param turtle_name id of the turtle
 * @return true if path traversing is completed
 */
void turtleThread(ros::NodeHandle& nh, int turtle_index, const std::string& turtle_name, ros::Publisher velocity_publisher)
{
    while (ros::ok())
    {
        std::vector<double> velocities = received_velocities[turtle_index];
        std::vector<double> orientations = received_orientations[turtle_index];        
        std::vector<double> durations = received_durations[turtle_index];
        std::vector<double> angular_velocities = received_angular_velocities[turtle_index];

        turtlesim::Pose& pose = received_poses[turtle_index];

        // Move the turtle along its path
        if (moveTurtleAlongPath(nh, velocities, orientations, durations, angular_velocities, turtle_name, pose, velocity_publisher))
        {
            return;
        }
    }
}

/**
 * Callback to subscribe to AGV_amount
 *
 * @param amt_msg message containing AGV amount info
 */
void AGV_amount_callback(const std_msgs::Int8::ConstPtr& amt_msg)
{
    received_amount.data = amt_msg->data;
}


void scheduleCallback(const multi_robot_path_scheduling::velocities_durations_orientations::ConstPtr& msg, int robot_index)
{
    received_velocities[robot_index] = msg->velocities;
    received_durations[robot_index] = msg->durations;
    received_orientations[robot_index] = msg->orientations;
    received_angular_velocities[robot_index] = msg->angular_velocities;
}

bool is_empty_schedule(std::vector<std::vector<double>> all_velocities, std::vector<std::vector<double>> all_durations, 
    std::vector<std::vector<double>> all_orientations, std::vector<std::vector<double>> all_ang_vels )
{
    for ( auto durations : all_durations )
    {
        if ( durations.empty() )
        {
            return true;
        }
    }
    for ( auto velocities : all_velocities )
    {
        if ( velocities.empty() )
        {
            return true;
        }
    }
    for ( auto orientations : all_orientations )
    {
        if ( orientations.empty() )
        {
            return true;
        }
    }
    for ( auto ang_vels : all_ang_vels )
    {
        if ( ang_vels.empty() )
        {
            return true;
        }
    }
    return false;
}

bool poseEquality(const turtlesim::Pose& pose1, const turtlesim::Pose& pose2, double tolerance = 1e-4) {
    // Compare x, y, theta (angle)
    if (std::abs(pose1.x - pose2.x) > tolerance ||
        std::abs(pose1.y - pose2.y) > tolerance ||
        std::abs(pose1.theta - pose2.theta) > tolerance) {
        return false;  // Poses are different
    }

    return true;  // Poses are equal
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_subscriber");
    ros::NodeHandle nh;

    received_amount.data = 0;

    // Subscribe to the published AGV_amount
    std::string amount_subscriber_name = "AGV_amount";
    ros::Subscriber amount_sub = nh.subscribe(amount_subscriber_name, 100, &AGV_amount_callback);

    ros::Rate rate(100);

    // Wait until the initialization is complete (received_amount is greater than zero and the flag is set)
    while (ros::ok() && !first_initialization_complete) 
    {
        if(received_amount.data != 0)
        {
            first_initialization_complete = true;
        }
        ros::spinOnce();
        rate.sleep();
    }

    received_durations.resize(received_amount.data);
    received_velocities.resize(received_amount.data);
    received_orientations.resize(received_amount.data);
    received_angular_velocities.resize(received_amount.data);
    received_poses.resize(received_amount.data);


    auto empty_pose = received_poses[0];


    // Create a vector to store subscribers for the published schedules
    std::vector<ros::Subscriber> schedule_subscribers;

    // Create a vector to store subscribers for the published turtle poses
    std::vector<ros::Subscriber> turtle_pose_subscribers;


    for (size_t i = 0; i < received_amount.data; i++)
    {
        std::string turtle_name = "turtle_" + std::to_string(i + 1);
        // Subscribe to each schedule
        std::string schedule_name = "schedule_" + std::to_string(i + 1);
        ros::Subscriber schedule_subscriber = nh.subscribe<multi_robot_path_scheduling::velocities_durations_orientations>(
            schedule_name, 100, boost::bind(scheduleCallback, _1, i));
        schedule_subscribers.push_back(schedule_subscriber);

        // Subscribe to each turtle pose
        std::string turtle_pose_name = turtle_name + "/pose";
        ros::Subscriber turtle_pose_subscriber = nh.subscribe<turtlesim::Pose>(turtle_pose_name, 100, [i](const turtlesim::Pose::ConstPtr& msg) {
            received_poses[i] = *msg;
        });
        turtle_pose_subscribers.push_back(turtle_pose_subscriber);

    }

    // Wait until the initialization is complete (received_amount is greater than zero and the flag is set)
    while (ros::ok() && !second_initialization_complete) 
    {
        if( !is_empty_schedule(received_velocities, received_durations, received_orientations, received_angular_velocities) 
            && !poseEquality(empty_pose, received_poses[0]) )
        {
            second_initialization_complete = true;
        }
        ros::spinOnce();
        rate.sleep();
    }

    for (size_t i = 0; i < received_amount.data; i++)
    {
        std::string turtle_name = "turtle_" + std::to_string(i + 1);

        // Create publishers for each turtle velocity
        ros::Publisher cmdVelPublisher = nh.advertise<geometry_msgs::Twist>(turtle_name + "/cmd_vel", 100);

        // Start a thread for each turtle
        std::thread turtle_thread(turtleThread, std::ref(nh), i, turtle_name, cmdVelPublisher);
        turtle_thread.detach();
    }

    // Main thread for ros operations if needed
    while (ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
