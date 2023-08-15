#include "../include/agv_collision_space.h" 

AGVCollisionSpace::AGVCollisionSpace( int x_bound, int y_bound, int AGV_amount,int path_subdivision_amount,  int AGV_radius, int path_min_stops, 
                                      int path_max_stops, int path_length_min, int path_length_max)
{
    // x_bound: max x value of any point on line
    // y_bound: max y value of any point on line
    // AGV_amount: amount of paths = robot amount iteration = iteration + 1
    // path_min_stops: min number of times slope can be shifted
    // path_max_stops: max number of times slope can be shifted
    // path_length_min: min length of a line segment
    // pathlength_max: max length of a line segment
    // path_subdivision_amount: segment amount a line is separated to
    
    this->x_bound = x_bound;
    this->y_bound = y_bound;
    this->AGV_amount = AGV_amount;
    this->AGV_radius = AGV_radius;
    this->path_min_stops = path_min_stops;
    this->path_max_stops = path_max_stops;
    this->path_length_min = path_length_min;
    this->path_length_max = path_length_max;
    this->path_subdivision_amount = path_subdivision_amount;

    initialize_collision_space();
}

/**
 * Calls methods to generate paths and finc collisions along paths.
 */
void AGVCollisionSpace::initialize_collision_space()
{
    std::cout << "  Generating paths..." << std::endl;
    for ( int i = 0; i < AGV_amount; i++ )
    {
        auto new_path = PhysicalPath(x_bound, y_bound, AGV_radius, path_subdivision_amount, 
            path_min_stops, path_max_stops, path_length_min, path_length_max, &paths);
        paths.push_back(new_path);
    }

    std::cout << "  Finding collisions..." << std::endl;
    collision_map = find_all_collisions();
}

/**
 * Given two paths, finds all collisions that occur in the path pair. AGVs are assumed to be
 * circular and each segment is assumed to be traveled at identical times.
 * 
 * @param line1 Points on the first line of the line pair
 * @param line2 Points on the second line of the line pair
 * @return collision_times A map of all collisions that occur on path schedule
 */
std::unordered_map<std::pair<int, int>, bool, pair_hash> AGVCollisionSpace::find_collision_points(const std::vector<Point>& line1, const std::vector<Point>& line2) 
{
    std::unordered_map<std::pair<int, int>, bool, pair_hash> collision_times;

    for ( int i = 0; i < path_subdivision_amount; i++ )
    {
        for( int k = 0; k < path_subdivision_amount; k++ )
        {
            int first = i;
            int second = k;
            auto point = std::make_pair(first, second);
            collision_times[point] = false;

        }
    }

    for (size_t i = 0; i < line1.size(); ++i) 
    {
        for (size_t k = 0; k < line2.size(); ++k) 
        {
            // Calculate the calculate_distance between the points on each segment
            double calculated_distance = calculate_distance(line1[i], line2[k]);
            int first = i;
            int second = k;

            // If the calculate_distance is less than or equal to 2r, there is a collision
            if (calculated_distance <= 2 * AGV_radius) 
            {
                // Pass the segment position. The assumption here is that every
                // segment is traversed in the same amount of time. Velocity
                // should be included for more realistic values
                auto time_point = std::make_pair(first, second);
                collision_times[time_point] = true;
                auto location_point_1 = std::make_pair(line1[i].coordinates[0], line1[i].coordinates[1]);
                auto location_point_2 = std::make_pair(line2[k].coordinates[0], line2[k].coordinates[1]);
                collision_points.push_back(location_point_1);
                collision_points.push_back(location_point_2);
            } 
        }
    }

    return collision_times;
}

/**
 * Calls find_collision_points for each pair of paths
 * 
 * @return collision_dict A map of all pairs of paths and collisions that occur on the pairs
 */
std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash> AGVCollisionSpace::find_all_collisions() 
{
    std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash> collision_dict;

    for (size_t i = 0; i < paths.size() - 1; ++i) 
    {
        for (size_t k = i + 1; k < paths.size(); ++k) 
        {
            std::pair<int, int> collision_pair(i, k);
            auto collisions = find_collision_points(paths[i].get_final_physical_path_points(), paths[k].get_final_physical_path_points());
            collision_dict[collision_pair] = collisions;
        }
    }
    return collision_dict;
}

