#ifndef NODE_SCHEDULE_H
#define NODE_SCHEDULE_H

#include "project_utilities.h"

/**
 * Velocity scheduler for a singular node.
 *
 * Given starting velocity and information regarding
 * upcoming movements, calculates schedules of AGVs in the current node.
 */
class NodeSchedule
{
public:
    NodeSchedule(std::vector<std::vector<double>> schedule_info, double max_velocity, double max_accel, double dt);
    void calculate_max_final_vels();
    void node_scheduler_any_time();
    void node_scheduler_time_constrained();
    void arrange_backward_movement();
    std::vector<std::pair<double, double>> quadratic_scheduler(double start_vel, double max_end_vel, double max_vel, double distance, double duration);
    std::vector<std::pair<double, double>> segment_schedule(std::vector<std::pair<double, double>> current_schedule);
    std::vector<std::pair<double, double>> trapezoidal_velocity_planner(double start_vel, double max_end_vel, double max_vel, double distance);

    std::vector<std::vector<std::vector<double>>> get_velocity_duration_orientation();

    // Getter for the motion profile
    std::vector<std::vector<std::pair<double, double>>> get_profile() const 
    {
        return time_constrained_plans;
    }

    // Getter for the end vels
    std::vector<double> get_end_vels() const 
    {
        return end_vels;
    }


private:

    double max_vel;
    double max_acc;
    double dt;

    std::vector<double> temp_durations;
    double final_duration;

    int agv_amount;
    std::vector<double> distance_to_next_zero_vel_nodes;
    std::vector<double> distance_within_nodes;
    std::vector<double> distance_within_next_nodes;
    std::vector<double> start_vels;
    std::vector<double> movement_types;

    std::vector<bool> has_to_stop;

    std::vector<double> max_end_vels;

    std::vector<std::vector<std::pair<double, double>>> any_time_plans;
    std::vector<std::vector<std::pair<double, double>>> time_constrained_plans;


    std::vector<double> any_time_scheduler_times;
    int longest_any_time_plan;

    std::pair<double, std::vector<std::pair<double, double>>> duration_restricted_schedule;
    std::vector<double> end_vels;

};

#endif // NODE_SCHEDULE_H
