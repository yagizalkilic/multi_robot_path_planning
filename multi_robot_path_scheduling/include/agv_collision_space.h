#ifndef AGV_COLLISION_SPACE_H
#define AGV_COLLISION_SPACE_H

#include "project_utilities.h"
#include <vector>
#include <unordered_map>
#include <utility>
#include <random>
#include <boost/math/common_factor.hpp>

/**
 * Space information needed to coordinate AGVs along paths.
 *
 * Randomly generates paths and positions for given AGVs, 
 * finds collisions and generates information for plotting.
 * Should be improved so that it can receive predetermined path
 * and position information.
 */
class AGVCollisionSpace
{
public:
    // Constructor
    AGVCollisionSpace(int x_bound, int y_bound, int AGV_amount, int AGV_radius,
                      int path_min_stops, int path_max_stops, int path_length_min, int path_length_max);

    // Public getter for paths
    const std::vector<std::vector<Point>>& get_paths() const
    {
        return paths;
    }

    // Public getter for collision_map
    const std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash>& get_collision_map() const
    {
        return collision_map;
    }

    // Public getter for collision_points
    const std::vector<std::pair<int, int>>& get_collision_points() const
    {
        return collision_points;
    }
    // Public getter for side_dimension_length
    const int& get_side_dimension_length() const
    {
        return path_subdivision_amount;
    }


private:
    int x_bound;
    int y_bound;
    int AGV_amount;
    int AGV_radius;
    int path_min_stops;
    int path_max_stops;
    int path_length_min;
    int path_length_max;
    int path_subdivision_amount;

    std::vector<std::vector<Point>> paths;
    std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash> collision_map;
    std::vector<std::pair<int, int>> collision_points;

    // Private member functions
    void calculate_path_subdivision_amount();
    void initialize_collision_space();
    std::vector<Point> generate_path();
    std::unordered_map<std::pair<int, int>, bool, pair_hash> find_collision_points(const std::vector<Point>& line1, const std::vector<Point>& line2);
    std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash> find_all_collisions();
    bool is_colluded(int new_x, int new_y, int i);

};

#endif // AGV_COLLISION_SPACE_H