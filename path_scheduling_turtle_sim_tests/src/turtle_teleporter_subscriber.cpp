#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>
#include <turtlesim/TeleportAbsolute.h>
#include <string>
#include <vector>
#include <thread>

// Global variables to store the subscribed data
std::vector<nav_msgs::Path> received_paths;
std::vector<std_msgs::Int32MultiArray> received_schedules;
std_msgs::Int8 received_amount;

bool initialization_complete = false;

/**
 * Teleports the turtles on the given path.
 *
 * @param nh NodeHandle
 * @param path physical path to follow
 * @param schedule schedule to follow the path
 * @param turtle_name id of the turtle
 * @return true if path traversing is completed
 */
bool moveTurtleAlongPath(ros::NodeHandle& nh, const nav_msgs::Path& path, const std_msgs::Int32MultiArray& schedule, const std::string& turtle_name)
{
    ros::ServiceClient teleport_client = nh.serviceClient<turtlesim::TeleportAbsolute>("/" + turtle_name + "/teleport_absolute");

    for ( size_t i = 0; i < schedule.data.size(); i++)
    {
        turtlesim::TeleportAbsolute teleport_srv;
        auto& pose_stamped = path.poses[schedule.data[i]];
        teleport_srv.request.x = pose_stamped.pose.position.x;
        teleport_srv.request.y = pose_stamped.pose.position.y;
        teleport_srv.request.theta = pose_stamped.pose.orientation.w;

        if (i == schedule.data.size() - 1 || pose_stamped.pose.position.x == 0.0 || pose_stamped.pose.position.y == 0.0)
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

/**
 * Run each turtle on a thread.
 *
 * @param nh NodeHandle
 * @param turtle_index position of turtle in turtle_list
 * @param turtle_name id of the turtle
 * @return true if path traversing is completed
 */
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

/**
 * Callback to subscribe to AGV_amount
 *
 * @param amt_msg message containing AGV amount info
 */
void AGV_amount_callback(const std_msgs::Int8::ConstPtr& amt_msg)
{
    received_amount.data = amt_msg->data;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "turtle_teleporter_subscriber_node");
    ros::NodeHandle nh;

    received_amount.data = 0;

    // Subscribe to the published AGV_amount
    std::string amount_subscriber_name = "AGV_amount";
    ros::Subscriber amount_sub = nh.subscribe(amount_subscriber_name, 10, &AGV_amount_callback);

    ros::Rate rate(1);

    // Wait until the initialization is complete (received_amount is greater than zero and the flag is set)
    while (ros::ok() && !initialization_complete) 
    {
        if(received_amount.data != 0)
        {
            initialization_complete = true;
        }
        ros::spinOnce();
        rate.sleep();
    }

    // Initialize the vector to store paths
    received_paths.resize(received_amount.data);
    received_schedules.resize(received_amount.data);

    // Create a vector to store subscribers for the published paths
    std::vector<ros::Subscriber> path_subscribers;

    // Create a vector to store subscribers for the published schedules
    std::vector<ros::Subscriber> schedule_subscribers;

    for (size_t i = 0; i < received_amount.data; i++)
    {
        // Subscribe to each path
        std::string turtle_name = "turtle_" + std::to_string(i + 1);
        std::string subscriber_name = "path_" + std::to_string(i + 1);
        ros::Subscriber path_subscriber = nh.subscribe<nav_msgs::Path>(subscriber_name, 1, [i](const nav_msgs::Path::ConstPtr& msg) {
            received_paths[i] = *msg;
        });
        path_subscribers.push_back(path_subscriber);

        //Subscribe to each schedule
        std::string schedule_name = "schedule_" + std::to_string(i + 1);
        ros::Subscriber schedule_subscriber = nh.subscribe<std_msgs::Int32MultiArray>(schedule_name, 1, [i](const std_msgs::Int32MultiArray::ConstPtr& msg) {
            received_schedules[i] = *msg;
        });
        schedule_subscribers.push_back(schedule_subscriber);

        // Start a thread for each turtle
        std::thread turtle_thread(turtleThread, std::ref(nh), i, turtle_name);
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
