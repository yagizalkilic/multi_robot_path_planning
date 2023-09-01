#ifndef PHYSICAL_PATH_H
#define PHYSICAL_PATH_H

#include "project_utilities.h"

/**
 * Class that represents a physical path.
 *
 * Each path belongs in the collision space and is
 * divided into n segments that are traversed at each node.
 */
class PhysicalPath 
{
public:
    PhysicalPath( int x_boundary, int y_boundary, int AGV_radius, double segment_distance, 
                  int path_min_stops, int path_max_stops, int path_length_min , int path_length_max, 
                  std::vector<PhysicalPath>* other_paths );

    PhysicalPath( double segment_distance, std::vector<Point> prepared_path );

    std::vector<Point> generate_random_path();

    bool is_conflicted_with_other_path(Point new_point, int i);

    void process_path();

    // Getter for the final physical path
    std::vector<Point> get_final_physical_path_points() const 
    {
        return final_path;
    }

    // Getter for segment distance
    double get_path_subdivision_amount() 
    {
        return path_subdivision_amount;
    }

private:
    int x_boundary;
    int y_boundary;
    int AGV_radius;
    int path_subdivision_amount;
    int path_min_stops;
    int path_max_stops;
    int path_length_min;
    int path_length_max;
    double segment_distance;
    std::vector<PhysicalPath> other_paths;

    std::vector<Point> unprocessed_path;
    std::vector<Point> prepared_path;
    std::vector<Point> final_path;
};

#endif // PHYSICAL_PATH_H
