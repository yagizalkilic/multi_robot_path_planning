#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32MultiArray.h>
#include <turtlesim/TeleportAbsolute.h>
#include <string>
#include <vector>
#include <thread>

// Global variables to store the latest paths received
std::vector<nav_msgs::Path> received_paths;
std::vector<std_msgs::Int32MultiArray> received_schedules;

// Function to move a turtle along a given path
bool moveTurtleAlongPath(ros::NodeHandle& nh, const nav_msgs::Path& path, const std_msgs::Int32MultiArray& schedule, const std::string& turtle_name)
{
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/" + turtle_name + "/teleport_absolute");

    for ( size_t i = 0; i < schedule.data.size(); i++)
    {
        turtlesim::TeleportAbsolute teleport_srv;
        auto& pose_stamped = path.poses[schedule.data[i]];
        teleport_srv.request.x = pose_stamped.pose.position.x;
        teleport_srv.request.y = pose_stamped.pose.position.y;
        teleport_srv.request.theta = atan2(2.0 * (pose_stamped.pose.orientation.w * pose_stamped.pose.orientation.z),
                                           1.0 - 2.0 * (pose_stamped.pose.orientation.z * pose_stamped.pose.orientation.z));

        if (i == schedule.data.size() - 1 || pose_stamped.pose.position.x == 0.0 && pose_stamped.pose.position.y == 0.0)
        {
            ROS_INFO("Turtle %s reached the last coordinate, stopping.", turtle_name.c_str());
            return true;
        }

        if (teleport_client.call(teleport_srv))
        {
            ROS_INFO("Turtle %s moved to position (x: %.2f, y: %.2f, theta: %.2f)",
                     turtle_name.c_str(), pose_stamped.pose.position.x, pose_stamped.pose.position.y,
                     teleport_srv.request.theta);
        }
        else
        {
            ROS_ERROR("Failed to call teleport service for Turtle %s", turtle_name.c_str());
            break;
        }

        // Sleep to simulate the turtle moving along the path
        ros::Duration(0.02).sleep();


    }
}

// Thread function for each turtle
void turtleThread(ros::NodeHandle& nh, int turtle_index, const std::string& turtle_name)
{
    while (ros::ok())
    {
        if (!received_paths.empty() && !received_schedules.empty() && turtle_index < received_paths.size())
        {
            // Get the path for this turtle
            const nav_msgs::Path& path = received_paths[turtle_index];
            const std_msgs::Int32MultiArray& schedule = received_schedules[turtle_index];

            // Move the turtle along its path
            if (moveTurtleAlongPath(nh, path, schedule, turtle_name))
            {
                return;
            }
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_follower_node");
    ros::NodeHandle nh;

    // Specify the turtle names for each path (Make sure you have spawned turtles with these names)
    std::vector<std::string> turtle_names = {"turtle_1", "turtle_2", "turtle_3", "turtle_4"}; // Adjust the names as needed

    // Initialize the vector to store paths
    received_paths.resize(turtle_names.size());
    received_schedules.resize(turtle_names.size());

    // Subscribe to the published paths
    std::vector<ros::Subscriber> path_subscribers;

    // Subscribe to the published schedules
    std::vector<ros::Subscriber> schedule_subscribers;

    for (size_t i = 0; i < turtle_names.size(); i++)
    {
        std::string subscriber_name = "path_" + std::to_string(i + 1);
        ros::Subscriber path_subscriber = nh.subscribe<nav_msgs::Path>(subscriber_name, 1, [i](const nav_msgs::Path::ConstPtr& msg) {
            received_paths[i] = *msg;
        });
        path_subscribers.push_back(path_subscriber);

        std::string schedule_name = "schedule_" + std::to_string(i + 1);
        ros::Subscriber schedule_subscriber = nh.subscribe<std_msgs::Int32MultiArray>(schedule_name, 1, [i](const std_msgs::Int32MultiArray::ConstPtr& msg) {
            received_schedules[i] = *msg;
        });
        schedule_subscribers.push_back(schedule_subscriber);

        // Start a thread for each turtle
        std::thread turtle_thread(turtleThread, std::ref(nh), i, turtle_names[i]);
        turtle_thread.detach();
    }

    // Main loop
    ros::spin();

    return 0;
}
