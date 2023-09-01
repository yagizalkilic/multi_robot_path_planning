#ifndef CONNECTED_RRT_STAR_H
#define CONNECTED_RRT_STAR_H

#include "project_utilities.h"

/**
 * Implementation of an connected RRT Star.
 *
 * Improves upon the RRT* implementation by adding a
 * sampling strategy.
 */
class ConnectedRRTStar 
{
public:
    ConnectedRRTStar(int num_dimensions, std::vector<int> space_side_length, 
        std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash> dimension_pair_correspondence);

    void generate_tree();
    bool is_done(const Point& point1, const Point& point2);
    bool generate_node(int point_count, std::vector<Node>* some_node_list, bool is_targeted);
    int nearest_node(const Point& point, std::vector<Node> some_node_list) const;
    bool is_valid(std::vector<Point> points);
    void find_final_path();
    void show_longest_generation() const;
    void initialize_n_space();
    std::vector<int> find_nodes_within_radius(const Node& node, double radius) const;
    void rewire_tree(Node& new_node, std::vector<Node> some_node_list);
    bool is_valid_path(const Point& point1, const Point& point2);
    std::vector<int> find_k_nearest_nodes(const Node& node, int k, std::vector<Node> some_node_list) const;

    // Getter for the final path
    std::vector<Node> get_final_path() const 
    {
        return path_nodes;
    }

    // Getter for all nodes
    std::vector<Node> get_all_nodes() const 
    {
        std::vector<Node> all_node_list;
        for ( auto i : node_list )
        {
            all_node_list.push_back(i);
        }
        for ( auto i : target_node_list )
        {
            all_node_list.push_back(i);
        }
        return all_node_list;
    }


private:
    int num_dimensions;
    std::vector<int> space_side_length;
    std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash> dimension_pair_correspondence;

    Point start_point;
    Point target_point;
    
    std::vector<Node> node_list;
    std::vector<Node> target_node_list;

    std::vector<Node> path_nodes;

    int max_nodes;
    int max_seconds;

    std::chrono::duration<double> total_elapsed_time;
    std::chrono::duration<double> average_node_generation;
    std::chrono::duration<double> longest_gen_time;

    int radius;
    int final_distance;

};

#endif // CONNECTED_RRT_STAR_H
