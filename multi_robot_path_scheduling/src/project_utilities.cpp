#include "../include/project_utilities.h"

/**
 * Given two points on n dimensions, calculate the euclidian distance between them.
 *  
 * @param p1 First point
 * @param p1 Second point
 * @return distance between points
 */
double calculate_distance(const Point& p1, const Point& p2) {
    if (p1.dimension != p2.dimension) {
        // Handle the case when points have different dimensions
        throw std::runtime_error("Points have different dimensions");
    }

    int dimension = p1.dimension;
    int sum = 0;

    for (int i = 0; i < dimension; ++i) {
        int diff = p1.coordinates[i] - p2.coordinates[i];
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