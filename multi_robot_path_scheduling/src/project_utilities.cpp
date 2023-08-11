#include "../include/project_utilities.h"

/**
 * Given two points on n dimensions, calculate the euclidian distance between them.
 *  
 * @param p1 First point
 * @param p1 Second point
 * @return distance between points
 */
double calculate_distance(const Point& p1, const Point& p2) {
    if (p1.dimension != p2.dimension) 
    {
        // Handle the case when points have different dimensions
        throw std::runtime_error("Points have different dimensions");
    }
    if(p1.coordinates == p2.coordinates)
    {
        return 0.0;
    }

    int dimension = p1.dimension;
    double sum = 0.0;

    for (int i = 0; i < dimension; ++i) 
    {
        double diff = p1.coordinates[i] - p2.coordinates[i];
        sum += diff * diff;
    }

    return std::sqrt(sum);
}

/**
 * Given a max and a min segment length, finds a random segment length in between.
 * 
 * @param min_length Minimum length segment can have
 * @param max_length Maximum length segment can have
 * @return segment length
 */
int random_segment_length(int min_length, int max_length)
{
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dist(min_length, max_length);
    return dist(gen);
}

/**
 * Given a max and a min value, finds a random double in between.
 * 
 * @param min_val Minimum value the double can have
 * @param max_val Maximum value the double can have
 * @return double value
 */
double get_random_double(double min_val, double max_val) 
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(min_val, max_val);
    return dis(gen);
}

/**
 * Given an upper bound, lower bound is assumed to be 0, generates a point
 * in num_dimensions.
 * 
 * @param space_side_length Length of a dimension
 * @param num_dimensions Amount of dimensions
 * @return point Newly generated point
 */
Point generate_random_point(int space_side_length, int num_dimensions) {
    Point point;
    point.dimension = num_dimensions;
    for (int i = 0; i < num_dimensions; ++i) 
    {
        point.coordinates.push_back((int)(get_random_double(0, space_side_length)));
    }
    return point;
}

/**
 * Genereates points around a goal point. The shape of the random space is cubical.
 * 
 * @param space_side_length Length of a dimension
 * @param radius Maximum distance from goal to new point for each coordinate item
 * @param goal Point to generate points around
 * @return new_goal Newly generated point
 */
Point generate_around_goal(int space_side_length, int radius, Point goal) {
    Point new_goal;
    new_goal.dimension = goal.dimension;
    int num_dimensions = goal.dimension;
    for (int i = 0; i < num_dimensions; ++i) 
    {
        int vector = (int)get_random_double(1, radius);
        new_goal.coordinates.push_back(goal.coordinates[i] - vector);
    }
    return new_goal;
}

/**
 * Genereates a circle around a point.
 * 
 * @param x X coordinates of the center
 * @param y Y coordinates of the center
 * @param r radius of the circle
 * @return positions Points generated on the circle
 */
std::vector<Point> points_around_point(double x, double y, double r) {
    std::vector<Point> positions;
    const double stepSize = 0.05;
    double t = 0;
    while (t < 2 * M_PI) {
        Point point;
        point.dimension = 2;
        point.coordinates.push_back(r * cos(t) + x);
        point.coordinates.push_back(r * sin(t) + y);
        positions.push_back(point);
        t += stepSize;
    }
    return positions;
}

/**
 * Create a list of differences for each coordinate pair on two points.
 * 
 * @param point1 First point in comparision pair
 * @param point2 Second point in comparision pair
 * @return distances List of distances between coordinate item
 */
std::vector<int> get_distances(Point point1, Point point2)
{
    std::vector<int> distances;
    for (int i = 0; i < point1.dimension; i++)
    {
        distances.push_back(point1.coordinates[i] - point2.coordinates[i]);
    }
    return distances;
}


/**
 * Gets unique integers within a specified index interval of a vector<int>.
 *
 * @param num_list number list to search within
 * @param startIndex Starting index of the interval
 * @param endIndex Ending index of the interval
 * @return vector containing unique numbers
 */
 std::vector<int> get_unique_numbers(std::vector<int> num_list, int start_index, int end_index) 
 {
    std::vector<int> unique_numbers;

    for (int i = start_index; i <= end_index && i < num_list.size(); i++) 
    {
        if (std::find(unique_numbers.begin(), unique_numbers.end(), num_list[i]) == unique_numbers.end())
        {
            unique_numbers.push_back(num_list[i]);
        }
    }

    return unique_numbers;
}

/**
 * Compares neighbours and includes the item only if it is different from its neighbours.
 * Ex: 2 4 5 5 6 9 2 10 10 10 2 3 4 3 3 returns 2 4 5 6 9 2 10 2 3 4 3
 *
 * @param num_list number list to search within
 * @param startIndex Starting index of the interval
 * @param endIndex Ending index of the interval
 * @return vector containing neighbour-unique items
 */
 std::vector<int> get_unique_neighbours(std::vector<int> num_list, int start_index, int end_index) 
 {
    std::vector<int> unique_neighbours;

    for (int i = start_index; i <= end_index && i < num_list.size(); i++) 
    {
        if ( i == start_index )
        {
            unique_neighbours.push_back(num_list[i]);
        }
        else
        {
            if ( num_list[i-1] != num_list[i] )
            {
                unique_neighbours.push_back(num_list[i]);
            }
        }
    }

    return unique_neighbours;
}

/**
 * Returns the vector without the specified number
 *
 * @param num_list number list to search within
 * @param number to exclude
 * @return vector without the number
 */
 std::vector<int> exclude_number(std::vector<int> num_list, int number, int start_index, int end_index)
 {
    std::vector<int> excluded_list;

    for (int i = start_index; i <= end_index && i < num_list.size(); i++) 
    {
        if ( num_list[i] != number )
        {
            excluded_list.push_back(num_list[i]);
        }
    }

    return excluded_list;
}


/**
 * Calculates the total distance between adjacent points within a specified index interval of a nav_msgs::Path.
 *
 * @param path nav_msgs::Path to calculate distances from
 * @param first_index Starting index of the interval (inclusive)
 * @param last_index Ending index of the interval (inclusive)
 * @return Total distance between adjacent points in the interval
 */
double calculate_total_distance(std::vector<Point> physical_path, int first_index, int last_index) 
{
    double totalDistance = 0.0;

    for ( int i = first_index; i < last_index; i++)
    {
        Point currentPoint;
        currentPoint.coordinates.push_back((physical_path[i].coordinates[0]) * 10.0);
        currentPoint.coordinates.push_back((physical_path[i].coordinates[1]) * 10.0);

        Point nextPoint;
        nextPoint.coordinates.push_back((physical_path[i+1].coordinates[0]) * 10.0);
        nextPoint.coordinates.push_back((physical_path[i+1].coordinates[1]) * 10.0);

        totalDistance += calculate_distance(currentPoint, nextPoint);
    }
    return totalDistance / 10.0;
}

/**
 * Calculates the orientation a robot must have to go from point 1 to point 2.
 *
 * @param point1 start point
 * @param point2 end point
 * @return orientation
 */
double calculate_orientation(Point point1, Point point2)
{
    double angle;

    double cur_x = (float)(point1.coordinates[0]) / 10.0;
    double cur_y = (float)(point1.coordinates[1]) / 10.0;

    double next_x = (float)(point2.coordinates[0]) / 10.0;
    double next_y = (float)(point2.coordinates[1]) / 10.0;

    // Determine the orientation of the robot
    angle = atan2(next_y - cur_y, next_x - cur_x);

    return angle;
}

/**
 * Calculates the angular velocity given ending and beginning orientations and travel duration.
 *
 * @param orientation_begin start orientation
 * @param orientation_end end orientation
 * @param duration travel duration
 * @return angular_velocity
 */
double calculate_angular_velocity(double orientation_begin, double orientation_end, double duration) 
{
    if ( duration < 0.0001 )
    {
        return 0.0;
    }
    double delta_theta = orientation_end - orientation_begin;

    if (fabs(delta_theta) < 0.0001) 
    {
        return 0.0;
    }

    // Calculate angular velocity
    double angular_velocity = delta_theta / duration;

    return angular_velocity;
}