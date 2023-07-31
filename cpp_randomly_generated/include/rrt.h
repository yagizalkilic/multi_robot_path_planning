#ifndef RRT_H
#define RRT_H

#include <vector>
#include <unordered_map>
#include <iostream>
#include <string>
#include <random>
#include <cmath>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <limits.h>

#include "project_utilities.h"

/**
 * Implementation of an RRT.
 *
 * The tree requires a map of pairs of dimensions or in the case of 
 * AGV coordination, pairs of paths. Each pair is a key to another map
 * that contains information about restricted zones. The secondary map
 * maps every point on the zone to a boolean value that represents the
 * existence of a collision.
 */
class RRT {
public:
    RRT(int num_dimensions, int space_side_length, 
        std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash> dimension_pair_correspondence);

    void generate_tree();
    bool is_done(const Point& point) const;
    bool generate_node(int point_count);
    int nearest_node(const Point& point) const;
    bool is_valid(std::vector<Point> points);
    void find_final_path();
    void show_longest_generation() const;
    void initialize_n_space();

    // Getter for the final path
    std::vector<Node> get_final_path() const 
    {
        return path_nodes;
    }

    // Getter for all nodes
    std::vector<Node> get_all_nodes() const 
    {
        return node_list;
    }


private:
    int num_dimensions;
    int space_side_length;
    std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash> dimension_pair_correspondence;

    Point start_point;
    Point target_point;
    
    std::vector<Node> node_list;
    std::vector<Node> path_nodes;

    int max_nodes;
    int max_seconds;

    std::chrono::duration<double> total_elapsed_time;
    std::chrono::duration<double> average_node_generation;
    std::chrono::duration<double> longest_gen_time;

    int radius;
    int final_distance;

};

#endif // RRT_H
