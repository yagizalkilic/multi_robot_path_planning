#ifndef COORDINATED_MOTION_PROFILER_H
#define COORDINATED_MOTION_PROFILER_H

#include "project_utilities.h"
#include "physical_path.h"
#include "node_schedule.h" 


/**
 * Given the schedule, coordinates velocities of AGVs.
 *
 * For each node travelsal calculates the max travel time,
 * schedule robots according to max travel time, start velocity,
 * and max possible velocity at the end of node.
 */
class CoordinatedMotionProfiler
{
public:
    CoordinatedMotionProfiler( std::vector<PhysicalPath> physical_paths, std::vector<Node> temporal_path, 
        double max_velocity, double max_accel, double dt );
    void calculate_schedule_intervals();
    void convert_to_movement_directions();
    void calculate_interval_paths_and_segmented_schedules();

    // Getter for the motion profiler
    std::vector<std::vector<std::vector<std::pair<double, double>>>> get_schedule() 
    {
        return node_profiles;
    }

    // Getter for the movement directions
    std::vector<std::vector<int>> get_movement_directions() 
    {
        std::vector<std::vector<int>> directions;
        directions.resize(physical_paths.size(), std::vector<int>());
        for ( int i = 0; i < movement_directions.size(); i++ )
        {
            for ( int k = 0; k < movement_directions[i].size(); k++ )
            {
                directions[k].push_back(movement_directions[i][k][2]);
            }
        }
        return directions;
    }

    // Getter for schedules in continious format ( not divided to nodes )
    std::vector<std::vector<std::pair<double, double>>> get_individual_schedules()
    {
        std::vector<std::vector<std::pair<double, double>>> individual_schedules;
        individual_schedules.resize( physical_paths.size(), std::vector<std::pair<double, double>>() );

        std::vector<double> times;
        times.resize(physical_paths.size(), 0.0);

        for ( int i = 0; i < node_profiles.size(); i++) // node list
        {
            for ( int k = 0; k < node_profiles[i].size(); k++ ) // agv_list
            {
                for ( int t = 0; t < node_profiles[i][k].size(); t++ )
                {
                    if (!std::isnan(node_profiles[i][k][t].first))
                    {
                        times[k] += node_profiles[i][k][t].first;
                    }
                    if (!std::isnan(node_profiles[i][k][t].second))
                    {
                        individual_schedules[k].push_back(std::make_pair(times[k], node_profiles[i][k][t].second));
                    }
                    else
                    {
                        individual_schedules[k].push_back(individual_schedules[k].back());
                        std::cout << " nan val encountered! " << std::endl;
                    }
                }
            }
        }
        return individual_schedules;
    }


private:
    std::vector<double> segment_lengths;

    std::vector<std::vector<std::vector<int>>> intervaled_schedules;
    std::vector<std::vector<std::vector<int>>> movement_directions;
    std::vector<std::vector<std::vector<std::pair<double, double>>>> node_profiles;

    std::vector<PhysicalPath> physical_paths;
    std::vector<Node> temporal_path;
    double max_velocity;
    double max_accel;
    double dt;
};

#endif // COORDINATED_MOTION_PROFILER_H
