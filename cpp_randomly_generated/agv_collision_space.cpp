#include "agv_collision_space.h" 

AGVCollisionSpace::AGVCollisionSpace( int x_bound, int y_bound, int AGV_amount, int AGV_radius, int path_min_stops, 
                                      int path_max_stops, int path_length_min, int path_length_max )
{
    // x_bound: max x value of any point on line
    // y_bound: max y value of any point on line
    // AGV_amount: amount of paths = robot amount iteration = iteration + 1
    // path_min_stops: min number of times slope can be shifted
    // path_max_stops: max number of times slope can be shifted
    // path_length_min: min length of a line segment
    // pathlength_max: max length of a line segment
    // path_subdivision_amount: segment amount a line is separated to
    
    this -> x_bound = x_bound;
    this -> y_bound = y_bound;
    this -> AGV_amount = AGV_amount;
    this -> AGV_radius = AGV_radius;
    this -> path_min_stops = path_min_stops;
    this -> path_max_stops = path_max_stops;
    this -> path_length_min = path_length_min;
    this -> path_length_max = path_length_max;

    std::cout << "  Calculating path segmentations..." << std::endl;
    calculate_path_subdivision_amount();
    initialize_collision_space();
}

/**
 * Calculates the amount of segments a path should be divided into. The segment
 * amount is static for each AGV for coordination graph generation. The segment
 * amount is found by LCM to guarantee integer division. This method should be
 * improved upon for more realistic scenarios.
 */
void AGVCollisionSpace::calculate_path_subdivision_amount()
{
    int least_common_multiple = 1;
    int num_stops = path_min_stops - 1;
    while (num_stops < path_max_stops)
    {
        least_common_multiple = std::lcm(least_common_multiple, num_stops);
        num_stops++;
    }
    while (least_common_multiple < 300)
    {
        least_common_multiple = least_common_multiple * 2;
    }
    path_subdivision_amount = least_common_multiple;
}

/**
 * Calls methods to generate paths and finc collisions along paths.
 */
void AGVCollisionSpace::initialize_collision_space()
{
    std::cout << "  Generating paths..." << std::endl;
    for ( int i = 0; i < AGV_amount; i++ )
    {
        auto new_path = generate_path();
        paths.push_back(new_path);
    }

    std::cout << "  Finding collisions..." << std::endl;
    collision_map = find_all_collisions();
}

/**
 * Generate a random amount of connected lines that create a path, 
 * line amount and line length has a specified interval.
 * @return points Points along the generated path
 */
std::vector<Point> AGVCollisionSpace::generate_path() 
{
    // Generate a random line amount
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> num_stops_dist(path_min_stops + 1, path_max_stops + 1);

    int num_stops = num_stops_dist(gen);

    int partial_segments = path_subdivision_amount / (num_stops - 1);

    std::vector<double> stops_x;
    std::vector<double> stops_y;
    int stop_amount;

    // Generate stop points for a line ( beginning and end points of the line)
    for (int i = 0; i < num_stops; i++) 
    {
        stop_amount = stops_x.size();
        std::uniform_real_distribution<> x_dist(0, x_bound);
        std::uniform_real_distribution<> y_dist(0, y_bound);

        double new_x = x_dist(gen);
        double new_y = y_dist(gen);
        if (stop_amount > 0)
        {
            // Check if the line length is in the specified interval
            double distance = hypot(stops_x[stop_amount - 1] - new_x, stops_y[stop_amount - 1] - new_y);
            while (!(path_length_min < distance && distance < path_length_max))
            {
                new_x = x_dist(gen);
                new_y = y_dist(gen);
                distance = hypot(stops_x[stop_amount - 1] - new_x, stops_y[stop_amount - 1] - new_y);
            }
        }
        
        stops_x.push_back(new_x);
        stops_y.push_back(new_y);
    }

    // Generate the path and gather the points on them
    std::vector<Point> points;

    for (int i = 0; i < num_stops - 1; i++) 
    {
        double x1 = stops_x[i];
        double y1 = stops_y[i];
        double x2 = stops_x[i + 1];
        double y2 = stops_y[i + 1];

        double dx = (x2 - x1) / partial_segments;
        double dy = (y2 - y1) / partial_segments;

        for (int j = 0; j < partial_segments; j++) 
        {
            Point point;
            point.dimension = 2;
            point.coordinates.push_back(x1 + j * dx);
            point.coordinates.push_back(y1 + j * dy);
            points.push_back(point);
        }
    }

    return points;
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
            if (calculated_distance <= 2 * AGV_radius) {
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
            auto collisions = find_collision_points(paths[i], paths[k]);
            collision_dict[collision_pair] = collisions;
        }
    }
    return collision_dict;
}

