#include "../include/physical_path.h" 

PhysicalPath::PhysicalPath( int x_boundary, int y_boundary, int AGV_radius, int path_subdivision_amount, 
                            int path_min_stops, int path_max_stops, int path_length_min , int path_length_max, 
                            std::vector<PhysicalPath>* other_paths )
{   
    this->x_boundary = x_boundary;
    this->y_boundary = y_boundary;
    this->AGV_radius = AGV_radius;
    this->path_subdivision_amount = path_subdivision_amount;
    this->path_min_stops = path_min_stops;
    this->path_max_stops = path_max_stops;
    this->path_length_min = path_length_min;
    this->path_length_max = path_length_max;
    this->other_paths = *other_paths;

    this->unprocessed_path = prepared_path;

    unprocessed_path = generate_random_path();
    process_path();
}

PhysicalPath::PhysicalPath( int path_subdivision_amount, std::vector<Point> prepared_path )
{   
    this->unprocessed_path = prepared_path;
    process_path();
}

/**
 * Generate a random amount of connected lines that create a path, 
 * line amount and line length has a specified interval.
 * @return points Points along the generated path
 */
std::vector<Point> PhysicalPath::generate_random_path() 
{
    // Generate a random line amount
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_int_distribution<> num_stops_dist(path_min_stops + 1, path_max_stops + 1);

    int num_stops = num_stops_dist(gen);

    std::vector<Point> stops;
    int stop_amount;

    // Generate stop points for a line ( beginning and end points of the line)
    for (int i = 0; i < num_stops; i++) 
    {
        Point new_point;
        new_point.dimension = 2;
        stop_amount = stops.size();
        std::uniform_real_distribution<> x_dist((double)(AGV_radius * 2), (double)(x_boundary - AGV_radius * 2));
        std::uniform_real_distribution<> y_dist((double)(AGV_radius * 2), (double)(y_boundary - AGV_radius * 2));

        new_point.coordinates.push_back(x_dist(gen));
        new_point.coordinates.push_back(y_dist(gen));
        if (stop_amount > 0)
        {
            // Check if the line length is in the specified interval
            double distance = calculate_distance(stops[stop_amount - 1], new_point);
            while (!(path_length_min < distance && distance < path_length_max))
            {
                new_point.coordinates.clear();
                new_point.coordinates.push_back(x_dist(gen));
                new_point.coordinates.push_back(y_dist(gen));
                distance = calculate_distance(stops[stop_amount - 1], new_point);
            }
        }

        // Check if the path is impossible ( if the beginning or end point collides with another's beginning or end )
        if ( i == 0 || i == num_stops - 1 )
        {
            if ( is_conflicted_with_other_path(new_point, i) )
            {
                std::cout << "Impossible path generation at: " << new_point.coordinates[0] << ", " << new_point.coordinates[1] << std::endl;
                i--;
                continue;
            }
        }
        
        stops.push_back(new_point);

    }

    return stops;
}

/**
 * Checks if two paths are impossible to execute
 * 
 * @param new_x x coordinate of the new beginning or end point
 * @param new_y y coordinate of the new beginning or end point
 * @param i determines whether this is a beginning or end point (0 if beginning, end otherwise)
 * @return true if there is no possible solution
 */
bool PhysicalPath::is_conflicted_with_other_path(Point new_point, int i)
{
    for ( auto path : other_paths )
    {
        Point point_to_compare;
        if ( i == 0 )
        {
            point_to_compare = path.get_final_physical_path_points().front();
        }
        else
        {
            point_to_compare = path.get_final_physical_path_points().back();
        }

        // Calculate the calculate_distance between the points
        double calculated_distance = calculate_distance(new_point, point_to_compare);

        // If the calculate_distance is less than or equal to 2r there is a collision, 3r is taken for safety
        if (calculated_distance <= 3 * AGV_radius) 
        {
            return true;
        } 
    }
    return false;
}

/**
 * Divide the path into path_subdivision amount of equal length segments
 */
void PhysicalPath::process_path()
{
    // Calculate total path length
    double total_distance = calculate_total_distance( unprocessed_path, 0, unprocessed_path.size() - 1 );
    double segment_distance = total_distance / path_subdivision_amount;

    final_path.push_back(unprocessed_path.front());

    Point cur_point = final_path[0];

    int restricting_point_count = 1;
    Point next_restricting_point = unprocessed_path[restricting_point_count];

    for ( int i = 0; i < path_subdivision_amount - 1; i++ )
    {
        double cur_distance = 0;
        Point temp_point = cur_point;

        while ( cur_distance < segment_distance )
        {
            double distance_to_restricting = calculate_distance(temp_point, next_restricting_point);
            if ( cur_distance + distance_to_restricting <= segment_distance)
            {
                if ( 0.0 < std::fabs(cur_distance + distance_to_restricting - segment_distance) < segment_distance / 50 )
                {
                    std::cout << "fasf "<< std::endl;
                    cur_point = next_restricting_point;
                    final_path.push_back(cur_point);
                    restricting_point_count++;
                    if ( restricting_point_count == unprocessed_path.size() )
                    {
                        return;
                    }
                    next_restricting_point = unprocessed_path[restricting_point_count];
                    break;
                }
                else
                {
                    temp_point = next_restricting_point;
                    cur_distance += distance_to_restricting;
                    restricting_point_count++;
                    if ( restricting_point_count == unprocessed_path.size() )
                    {
                        return;
                    }
                    next_restricting_point = unprocessed_path[restricting_point_count];
                }
            }
            else
            {
                cur_point = generate_point_between(temp_point, next_restricting_point, segment_distance - cur_distance);
                final_path.push_back(cur_point);
                break;
                std::cout << "afas " << cur_point.coordinates[0] << " " << cur_point.coordinates[1] << std::endl;

            }
        }
    }
}