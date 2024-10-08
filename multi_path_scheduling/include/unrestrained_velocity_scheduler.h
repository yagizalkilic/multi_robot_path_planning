#ifndef UNRESTRAINED_VELOCITY_SCHEDULER_H
#define UNRESTRAINED_VELOCITY_SCHEDULER_H

#include "project_utilities.h"
#include "physical_path.h"

/**
 * Velocity scheduler that does not take into account acceleration limits.
 *
 * Given the schedule in time, and the physical space, schedules the velocities
 * of AGVs. Does not consider any kinematic constraints.
 */
class UnrestrainedVelocityScheduler 
{
public:
    UnrestrainedVelocityScheduler(std::vector<PhysicalPath> physical_paths, 
        std::vector<Node> temporal_path, double max_velocity);

    void calculate_intervals_and_schedules();
    void calculate_distances_between_intervals();
    void calculate_average_velocities_and_total_times_per_interval();
    void calculate_final_schedule();
    void calculate_angular_velocities();

    std::vector<std::vector<std::vector<double>>> get_velocity_duration_orientation();

private:
    std::vector<PhysicalPath> physical_paths;
    std::vector<Node> temporal_path;
    double max_velocity;

    int physical_path_amount;
        
    std::vector<std::vector<int>> schedules;
    std::vector<std::vector<int>> schedule_intervals;

    std::vector<double> schedule_interval_times;
    std::vector<std::vector<double>> schedule_interval_average_velocities;
    std::vector<std::vector<double>> schedule_interval_distances_list;

    std::vector<std::vector<double>> final_duration_schedules;
    std::vector<std::vector<double>> final_velocity_schedules;
    std::vector<std::vector<double>> final_orientation_schedules;
    std::vector<std::vector<double>> final_angular_velocity_schedules;



};

#endif // UNRESTRAINED_VELOCITY_SCHEDULER_H
