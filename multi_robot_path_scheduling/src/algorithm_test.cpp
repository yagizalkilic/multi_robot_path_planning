#include "../include/project_utilities.h"
#include "../include/agv_collision_space.h" 
#include "../include/rrt.h"
#include "../include/coordination_visualization.h"
#include "../include/rrt_star.h"

int main(int argc, char *argv[]) 
{
    // Initialize root
    TApplication *path_finding_app = new TApplication("path_finding", &argc, argv);

    // Coordination space properties
    int x_bound = 300; // max x value of any point on path
    int y_bound = 300; // max y value of any point on path
    int AGV_amount = 4; // amount of all_paths = robot amount iteration = iteration + 1
    int AGV_radius = 10; // radius of a circular agv
    int path_min_stops = 3; // min number of times slope can be shifted
    int path_max_stops = 4; // max number of times slope can be shifted
    int path_length_min = 50; // min length of a path segment
    int path_length_max = 70; // max length of a path segment  

    // Initialize the space information, determine paths and collisions on time and space
    std::cout << "Initializing coordination space..." << std::endl;
    auto collision_space = AGVCollisionSpace( x_bound, y_bound, AGV_amount, AGV_radius, path_min_stops, 
                                           path_max_stops, path_length_min, path_length_max );

    auto all_paths = collision_space.get_paths();
    auto all_collisions_time = collision_space.get_collision_map();
    auto all_collisions_space = collision_space.get_collision_points();
    int collision_side_dimension_length = collision_space.get_side_dimension_length();

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

    path_finding_app->Run();
    return 0;
}
