#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32MultiArray.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include "../include/project_utilities.h"
#include "../include/agv_collision_space.h" 
#include "../include/rrt.h"
#include "../include/coordination_visualization.h"
#include "../include/rrt_star.h"

std::vector<ros::Publisher> getPathPublishers(const std::vector<std::vector<Point>> stored_paths, ros::NodeHandle nh)
{
    std::vector<ros::Publisher> path_publishers;
    for ( int i = 0; i < stored_paths.size(); i++ )
    {
        auto stored_path = stored_paths[i];
        std::string publisher_name = "path_" + std::to_string(i + 1);
        ros::Publisher path_pub = nh.advertise<nav_msgs::Path>(publisher_name, 10);

        path_publishers.push_back(path_pub);
    }

    return path_publishers;
}

std::vector<ros::Publisher> getSchedulePublishers( int pub_amt, ros::NodeHandle nh)
{
    std::vector<ros::Publisher> schedule_publishers;
    for ( int i = 0; i < pub_amt; i++ )
    {
        std::string publisher_name = "schedule_" + std::to_string(i + 1);

        ros::Publisher schedule_pub = nh.advertise<std_msgs::Int32MultiArray>(publisher_name, 10);

        schedule_publishers.push_back(schedule_pub);
    }

        std::vector<std_msgs::Int32MultiArray> schedule_msgs;

    return schedule_publishers;
}

nav_msgs::Path convertPathToMsg(std::vector<Point> stored_path)
{
        nav_msgs::Path path;
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "world";

        for ( auto i : stored_path )
        {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = (float)(i.coordinates[0]) / 10.0;
            pose.pose.position.y = (float)(i.coordinates[1]) / 10.0;
            pose.pose.position.z = 0.0;
            pose.pose.orientation.x = 0.0;
            pose.pose.orientation.y = 0.0;
            pose.pose.orientation.z = 0.0;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }

    return path;
}

// Function to publish turtles for each path
void publishTurtlesForPaths(const std::vector<std::vector<Point>> stored_paths)
{
    turtlesim::Pose turtle_pose;
    turtle_pose.x = 0.0;
    turtle_pose.y = 0.0;
    turtle_pose.theta = 0.0;

    for (size_t i = 0; i < stored_paths.size(); ++i)
    {
        std::string turtle_name = "turtle_" + std::to_string(i + 1);

        turtle_pose.x = (float)(stored_paths[i][0].coordinates[0]) / 10.0;
        turtle_pose.y = (float)(stored_paths[i][0].coordinates[1]) / 10.0;
        turtle_pose.theta = 1.0;

        turtlesim::Spawn spawn;
        spawn.request.x = turtle_pose.x;
        spawn.request.y = turtle_pose.y;
        spawn.request.theta = turtle_pose.theta;
        spawn.request.name = turtle_name;
        ros::service::call("/spawn", spawn);
    }
}

std::vector<std_msgs::Int32MultiArray> calculateSchedule(std::vector<Node> path)
{
    Point current_point = path.front().point;
    std::vector<std::vector<int>> schedules(current_point.dimension);

    for ( int i = 0; i < current_point.dimension; i++ )
    {
        schedules[i].push_back(0);
    }

    for ( int i = 0; i < path.size(); i++ )
    {
        Point next_point = path[i].point;
        while ( current_point.coordinates != next_point.coordinates )
        {
            for ( int k = 0; k < current_point.dimension; k++)
            {
                if (current_point.coordinates[k] == next_point.coordinates[k])
                {
                    schedules[k].push_back(schedules[k].back());
                }
                else if (current_point.coordinates[k] < next_point.coordinates[k])
                {
                    schedules[k].push_back(schedules[k].back() + 1);
                    current_point.coordinates[k] += 1;
                }
                else
                {
                    schedules[k].push_back(schedules[k].back() - 1);
                    current_point.coordinates[k] -= 1;
                }
            }
        }
    }

    std::vector<std_msgs::Int32MultiArray> schedule_msgs;

    // Convert schedules to a flat array in schedule_msg
    for (const auto& dimension_schedule : schedules)
    {
        std_msgs::Int32MultiArray schedule_msg;
        for (const auto& value : dimension_schedule)
        {
            schedule_msg.data.push_back(value);
        }
        schedule_msgs.push_back(schedule_msg);
    }

    return schedule_msgs;
}


int main(int argc, char** argv)
{
    // Initialize root
    TApplication *path_finding_app = new TApplication("path_finding", &argc, argv);

    // Coordination space properties
    int x_bound = 100; // max x value of any point on path
    int y_bound = 100; // max y value of any point on path
    int AGV_amount = 4; // amount of all_paths = robot amount iteration = iteration + 1
    int AGV_radius = 5; // radius of a circular agv
    int path_min_stops = 3; // min number of times slope can be shifted
    int path_max_stops = 4; // max number of times slope can be shifted
    int path_length_min = 25; // min length of a path segment
    int path_length_max = 40; // max length of a path segment  

    // Initialize the space information, determine paths and collisions on time and space
    std::cout << "Initializing coordination space..." << std::endl;
    auto collision_space = AGVCollisionSpace( x_bound, y_bound, AGV_amount, AGV_radius, path_min_stops, 
                                           path_max_stops, path_length_min, path_length_max );

    auto all_paths = collision_space.get_paths();
    auto all_collisions_time = collision_space.get_collision_map();
    auto all_collisions_space = collision_space.get_collision_points();
    int collision_side_dimension_length = collision_space.get_side_dimension_length();

    ros::init(argc, argv, "random_path_generator");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<turtlesim::Kill>("/kill");

    turtlesim::Kill srv;
    srv.request.name = "turtle1"; // Replace "turtle1" with the name of the turtle you want to kill.

    client.call(srv);

    publishTurtlesForPaths(all_paths);

    std::vector<ros::Publisher> path_publishers = getPathPublishers(all_paths, nh);

    // Initialize visualizer, plot collision space
    auto visualizer = CoordinationVisualization(x_bound, y_bound, collision_side_dimension_length);

    visualizer.draw_space(all_paths, all_collisions_space, AGV_radius, "space_canvas");

    // Construct the RRT and determine the time path
    std::cout << "Constructing the RRT..." << std::endl;

    auto coordination_RRT = RRT(AGV_amount, collision_side_dimension_length, all_collisions_time);
    coordination_RRT.show_longest_generation();
    auto path_RRT = coordination_RRT.get_final_path();
    auto all_nodes_RRT = coordination_RRT.get_all_nodes();

    visualizer.draw_time(AGV_amount, path_RRT, all_nodes_RRT, all_collisions_time, "rrt_time_canvas");

    // Construct the RRT* and determine the time path
    std::cout << "Constructing the RRT*..." << std::endl;

    auto coordination_RRT_star = RRTStar(AGV_amount, collision_side_dimension_length, all_collisions_time);
    coordination_RRT_star.show_longest_generation();
    auto path_RRT_star = coordination_RRT_star.get_final_path();
    auto all_nodes_RRT_star = coordination_RRT_star.get_all_nodes();

    visualizer.draw_time(AGV_amount, path_RRT_star, all_nodes_RRT_star, all_collisions_time, "rrt_star_time_canvas");

    auto schedule_msgs = calculateSchedule(path_RRT);
    auto schedule_publishers = getSchedulePublishers(schedule_msgs.size(), nh);

    ros::Rate rate(1); // Publish rate in Hz, adjust as needed

    while (ros::ok())
    {
        for ( int i = 0; i < path_publishers.size(); i++ )
        {
            path_publishers[i].publish(convertPathToMsg(all_paths[i]));
        }

        for ( int i = 0; i < schedule_publishers.size(); i++ )
        {
            schedule_publishers[i].publish(schedule_msgs[i]);
        }

        ros::spinOnce();
        rate.sleep();
    }

    path_finding_app->Run();

    return 0;
}

