#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_srvs/Empty.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/Kill.h>
#include <multi_robot_path_scheduling/velocities_durations_orientations.h>
#include "../../include/project_utilities.h"
#include "../../include/agv_collision_space.h" 
#include "../../include/rrt.h"
#include "../../include/coordination_visualization.h"
#include "../../include/rrt_star.h"
#include "../../include/physical_path.h"
#include "../../include/node_schedule.h" 
#include "../../include/coordinated_motion_profiler.h" 
#include <thread>

/**
 * Generates velocity, duration, orientation publishers for each robot.
 *
 * @param pub_amt amount of schedule publishers to be generated
 * @param nh NodeHandle
 * @return schedule_publishers a vector of Publisher objects
 */
std::vector<ros::Publisher> get_schedule_publishers( int pub_amt, ros::NodeHandle nh)
{
    std::vector<ros::Publisher> schedule_publishers;
    for ( int i = 0; i < pub_amt; i++ )
    {
        std::string publisher_name;
        publisher_name = "schedule_" + std::to_string(i + 1);

        ros::Publisher schedule_pub = 
            nh.advertise<multi_robot_path_scheduling::velocities_durations_orientations>(publisher_name, 100);

        schedule_publishers.push_back(schedule_pub);
    }

    return schedule_publishers;
}

/**
 * Publish each turtle taking the beginning of their path as pose.
 *
 * @param stored_paths paths that turtles will be following
 */
void publishTurtlesForPaths(const std::vector<PhysicalPath> stored_paths)
{

    for (size_t i = 0; i < stored_paths.size(); ++i)
    {
        std::string turtle_name = "turtle_" + std::to_string(i + 1);

        double cur_x = (float)(stored_paths[i].get_final_physical_path_points()[0].coordinates[0]) / 10.0;
        double cur_y = (float)(stored_paths[i].get_final_physical_path_points()[0].coordinates[1]) / 10.0;

        double next_x = (float)(stored_paths[i].get_final_physical_path_points()[1].coordinates[0]) / 10.0;
        double next_y = (float)(stored_paths[i].get_final_physical_path_points()[1].coordinates[1]) / 10.0;

        double angle = atan2(next_y - cur_y, next_x - cur_x);

        turtlesim::Spawn spawn;
        spawn.request.x = cur_x;
        spawn.request.y = cur_y;
        spawn.request.theta = angle;
        spawn.request.name = turtle_name;
        ros::service::call("/spawn", spawn);
    }
}


int main(int argc, char** argv)
{
    // Initialize root
    TApplication *path_finding_app = new TApplication("path_finding", &argc, argv);

    // Coordination space properties
    int x_bound = 100; // max x value of any point on path
    int y_bound = 100; // max y value of any point on path
    int AGV_amount = 7; // amount of all_paths = robot amount iteration = iteration + 1
    int AGV_radius = 6; // radius of a circular agv
    double AGV_max_velocity = 1.0; // max velocity AGV can drive with
    int path_min_stops = 1; // min number of times slope can be shifted
    int path_max_stops = 1; // max number of times slope can be shifted
    int path_length_min = 30; // min length of a path segment
    int path_length_max = 35; // max length of a path segment  
    double segment_size = 3.0;
    double max_acceleration = 0.8;
    double dt = 0.05;

    // Initialize the space information, determine paths and collisions on time and space
    std::cout << "Initializing coordination space..." << std::endl;
    auto collision_space = AGVCollisionSpace( x_bound, y_bound, AGV_amount, segment_size, AGV_radius, 
                                              path_min_stops,path_max_stops, path_length_min, path_length_max,
                                              true, "/home/milvus/ros_ws/src/multi_path_scheduling/test_paths/grid_test.txt" );

    auto all_paths = collision_space.get_paths();
    auto all_collisions_time = collision_space.get_collision_map();
    auto all_collisions_space = collision_space.get_collision_points();
    auto all_collisions_dimensions = collision_space.get_time_dimensions();

    // Initialize visualizer, plot collision space
    auto visualizer = CoordinationVisualization(x_bound, y_bound, all_collisions_dimensions);

    visualizer.draw_space(all_paths, all_collisions_space, AGV_radius, "space_canvas");

    // Construct the RRT* and determine the time path
    std::cout << "Constructing the RRT*..." << std::endl;

    auto coordination_RRT_star = RRTStar(AGV_amount, all_collisions_dimensions, all_collisions_time);
    coordination_RRT_star.show_longest_generation();
    auto path_RRT_star = coordination_RRT_star.get_final_path();
    auto all_nodes_RRT_star = coordination_RRT_star.get_all_nodes();

    visualizer.draw_time(AGV_amount, path_RRT_star, all_nodes_RRT_star, all_collisions_time, "rrt_star_time_canvas");

    // Combine RRT* and the physical paths to create a schedule
    std::cout << "Calculating schedule..." << std::endl;
    auto velocity_scheduler = CoordinatedMotionProfiler(all_paths, path_RRT_star, AGV_max_velocity, max_acceleration, dt);
    auto velocity_schedule = velocity_scheduler.get_schedule();

    auto individual_schedules = velocity_scheduler.get_individual_schedules();

    visualizer.draw_velocity(AGV_max_velocity, AGV_amount, dt, individual_schedules, "tv_canvas");

    std::thread pathFindingThread([&]() {
        path_finding_app->Run();
    });

    // Start ros operations
    std::cout << "Giving instructions..." << std::endl;
    ros::init(argc, argv, "turtle_restrained_publisher");
    ros::NodeHandle nh;

    ros::ServiceClient kill_client = nh.serviceClient<turtlesim::Kill>("/kill");

    // Kill the default turtle
    turtlesim::Kill kill_srv;
    kill_srv.request.name = "turtle1";

    kill_client.call(kill_srv);

    publishTurtlesForPaths(all_paths);

    // Publish robot amount
    ros::Publisher robot_amount_pub = nh.advertise<std_msgs::Int8>("AGV_amount", 10);
    std_msgs::Int8 robot_amount;
    robot_amount.data = AGV_amount;

    // Get schedule publishers
    auto schedule_publishers = get_schedule_publishers(AGV_amount, nh);

    ros::Rate rate(1); // Publish rate in Hz, adjust as needed

    while (ros::ok())
    {
        robot_amount_pub.publish(robot_amount);

        for( int i = 0; i < AGV_amount; i++ )
        {
            // Publish velocity, duration, orientation for each robot
            multi_robot_path_scheduling::velocities_durations_orientations schedule_msg;

            std::vector<double> velocities;
            std::vector<double> durations;
            std::vector<double> orientations;
            std::vector<double> angular_velocities;

            for ( int k = 0; k < individual_schedules[i].size(); k++ )
            {
                velocities.push_back(individual_schedules[i][k].second);
                durations.push_back(dt);
                orientations.push_back(0.0);
                angular_velocities.push_back(0.0);

            }

            schedule_msg.velocities = velocities;
            schedule_msg.durations = durations;
            schedule_msg.orientations = orientations;
            schedule_msg.angular_velocities = angular_velocities;

            schedule_publishers[i].publish(schedule_msg);
        }

        ros::spinOnce();
        rate.sleep();
    }

    // Join the pathFindingThread before exiting the program
    if (pathFindingThread.joinable()) {
        pathFindingThread.join();
    }

    return 0;
}