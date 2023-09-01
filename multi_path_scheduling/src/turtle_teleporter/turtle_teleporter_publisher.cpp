#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include "../../include/project_utilities.h"
#include "../../include/agv_collision_space.h" 
#include "../../include/rrt.h"
#include "../../include/coordination_visualization.h"
#include "../../include/rrt_star.h"
#include "../../include/physical_path.h" 


/**
 * Generates publishers for each path.
 *
 * @param stored_paths all paths that were generated
 * @param nh NodeHandle
 * @return path_publishers a vector of Publisher objects
 */
std::vector<ros::Publisher> getPathPublishers(const std::vector<PhysicalPath> stored_paths, ros::NodeHandle nh)
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

/**
 * Generates publishers for each schedule.
 *
 * @param pub_amt amount of schedule publishers to be generated
 * @param nh NodeHandle
 * @return schedule_publishers a vector of Publisher objects
 */
std::vector<ros::Publisher> getSchedulePublishers( int pub_amt, ros::NodeHandle nh)
{
    std::vector<ros::Publisher> schedule_publishers;
    for ( int i = 0; i < pub_amt; i++ )
    {
        std::string publisher_name;
        publisher_name = "schedule_" + std::to_string(i + 1);

        ros::Publisher schedule_pub = nh.advertise<std_msgs::Int32MultiArray>(publisher_name, 10);

        schedule_publishers.push_back(schedule_pub);
    }

        std::vector<std_msgs::Int32MultiArray> schedule_msgs;

    return schedule_publishers;
}

/**
 * Converts path information into a publishable message.
 *
 * @param stored_path path to be converted
 * @return path publishable message
 */
nav_msgs::Path convertPathToMsg(std::vector<Point> stored_path)
{
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";

    double old_x, old_y, old_z, old_w;

    for ( int i = 0; i < stored_path.size() - 5; i++ )
    {
        double cur_x = (float)(stored_path[i].coordinates[0]) / 10.0;
        double cur_y = (float)(stored_path[i].coordinates[1]) / 10.0;

        double next_x = (float)(stored_path[i+5].coordinates[0]) / 10.0;
        double next_y = (float)(stored_path[i+5].coordinates[1]) / 10.0;

        if ( cur_x == next_x && cur_y == next_y )
        {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = path.header.stamp;
            pose.header.frame_id = path.header.frame_id;
            pose.pose.position.x = cur_x;
            pose.pose.position.y = cur_y;
            pose.pose.orientation.w = old_w;

            path.poses.push_back(pose);
            continue;
        }

        // Determine the orientation of the turtle
        double angle = atan2(next_y - cur_y, next_x - cur_x);

        // Add the pose to the path
        geometry_msgs::PoseStamped pose;
        pose.header.stamp = path.header.stamp;
        pose.header.frame_id = path.header.frame_id;
        pose.pose.position.x = cur_x;
        pose.pose.position.y = cur_y;

        // Store the orientation in w. Since this is a 2d object no need for a quaternion.
        pose.pose.orientation.w = angle;

        old_w = angle;

        path.poses.push_back(pose);
    }

    return path;
}

/**
 * Publish each turtle taking the beginning of their path as pose.
 *
 * @param stored_paths paths that turtles will be following
 */
void publishTurtlesForPaths(const std::vector<PhysicalPath> stored_paths)
{
    turtlesim::Pose turtle_pose;
    turtle_pose.x = 0.0;
    turtle_pose.y = 0.0;
    turtle_pose.theta = 0.0;

    for (size_t i = 0; i < stored_paths.size(); ++i)
    {
        std::string turtle_name = "turtle_" + std::to_string(i + 1);

        double cur_x = (float)(stored_paths[i].get_final_physical_path_points()[0].coordinates[0]) / 10.0;
        double cur_y = (float)(stored_paths[i].get_final_physical_path_points()[0].coordinates[1]) / 10.0;

        double next_x = (float)(stored_paths[i].get_final_physical_path_points()[1].coordinates[0]) / 10.0;
        double next_y = (float)(stored_paths[i].get_final_physical_path_points()[1].coordinates[1]) / 10.0;

        double angle = atan2(next_y - cur_y, next_x - cur_x);

        turtle_pose.x = cur_x;
        turtle_pose.y = cur_y;
        turtle_pose.theta = angle;

        turtlesim::Spawn spawn;
        spawn.request.x = turtle_pose.x;
        spawn.request.y = turtle_pose.y;
        spawn.request.theta = turtle_pose.theta;
        spawn.request.name = turtle_name;
        ros::service::call("/spawn", spawn);
    }
}

/**
 * Given the path (time representation), looks at current movements of all robots;
 * and if a robot is slower than the fastest robot, fills the remaining schedule of
 * the slower robot with zeros.
 *
 * @param path schedule that the robots will follow
 * @return schedule_msgs schedule of each robot
 */
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
    int AGV_radius = 6; // radius of a circular agv
    double AGV_max_velocity = 4; // max velocity AGV can drive with
    int path_min_stops = 2; // min number of times slope can be shifted
    int path_max_stops = 3; // max number of times slope can be shifted
    int path_length_min = 15; // min length of a path segment
    int path_length_max = 25; // max length of a path segment  
    double segment_size = 1.5;

    // Initialize the space information, determine paths and collisions on time and space
    std::cout << "Initializing coordination space..." << std::endl;
    auto collision_space = AGVCollisionSpace( x_bound, y_bound, AGV_amount, segment_size, AGV_radius, 
                                              path_min_stops,path_max_stops, path_length_min, path_length_max );

    auto all_paths = collision_space.get_paths();
    auto all_collisions_time = collision_space.get_collision_map();
    auto all_collisions_space = collision_space.get_collision_points();
    auto all_collisions_dimensions = collision_space.get_time_dimensions();

    // Initialize visualizer, plot collision space
    auto visualizer = CoordinationVisualization(x_bound, y_bound, all_collisions_dimensions);

    visualizer.draw_space(all_paths, all_collisions_space, AGV_radius, "space_canvas");

    // Construct the RRT and determine the time path
    std::cout << "Constructing the RRT..." << std::endl;

    auto coordination_RRT = RRT(AGV_amount, all_collisions_dimensions, all_collisions_time);
    coordination_RRT.show_longest_generation();
    auto path_RRT = coordination_RRT.get_final_path();
    auto all_nodes_RRT = coordination_RRT.get_all_nodes();

    visualizer.draw_time(AGV_amount, path_RRT, all_nodes_RRT, all_collisions_time, "rrt_time_canvas");

    // Construct the RRT* and determine the time path
    std::cout << "Constructing the RRT*..." << std::endl;

    auto coordination_RRT_star = RRTStar(AGV_amount, all_collisions_dimensions, all_collisions_time);
    coordination_RRT_star.show_longest_generation();
    auto path_RRT_star = coordination_RRT_star.get_final_path();
    auto all_nodes_RRT_star = coordination_RRT_star.get_all_nodes();

    visualizer.draw_time(AGV_amount, path_RRT_star, all_nodes_RRT_star, all_collisions_time, "rrt_star_time_canvas");

    // Start ros operations
    ros::init(argc, argv, "turtle_teleporter_publisher_node");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient<turtlesim::Kill>("/kill");

    // Kill the default turtle
    turtlesim::Kill srv;
    srv.request.name = "turtle1";

    client.call(srv);

    publishTurtlesForPaths(all_paths);

    std::vector<ros::Publisher> path_publishers = getPathPublishers(all_paths, nh);

    auto schedule_msgs = calculateSchedule(path_RRT);
    auto schedule_publishers = getSchedulePublishers(schedule_msgs.size(), nh);

    // Publish robot amount
    ros::Publisher robot_amount_pub = nh.advertise<std_msgs::Int8>("AGV_amount", 10);
    std_msgs::Int8 robot_amount;
    robot_amount.data = AGV_amount;

    ros::Rate rate(1); // Publish rate in Hz, adjust as needed

    while (ros::ok())
    {
        robot_amount_pub.publish(robot_amount);
        for ( int i = 0; i < path_publishers.size(); i++ )
        {
            path_publishers[i].publish(convertPathToMsg(all_paths[i].get_final_physical_path_points()));
        }

        for ( int i = 0; i < schedule_publishers.size(); i++ )
        {

            if ( i != schedule_publishers.size() - 1)
            {
                schedule_publishers[i].publish(schedule_msgs[i]);
            }
            else
            {
                schedule_publishers[i].publish(schedule_msgs[i]);
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    path_finding_app->Run();

    return 0;
}

