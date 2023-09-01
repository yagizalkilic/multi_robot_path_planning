#include "../../include/coordinated_motion_profiler.h" 

// Movement constants
const int FORWARD = 0;
const int WAIT = 1;
const int BACKWARD = 2;

CoordinatedMotionProfiler::CoordinatedMotionProfiler( std::vector<PhysicalPath> physical_paths, std::vector<Node> temporal_path, 
    double max_velocity, double max_accel, double dt )
{
    this->physical_paths = physical_paths;
    this->temporal_path = temporal_path;
    this->max_velocity = max_velocity;
    this->max_accel = max_accel;
    this->dt = dt;

    movement_directions.resize(physical_paths.size(), std::vector<std::vector<int>>());

    calculate_schedule_intervals();
    convert_to_movement_directions();
    calculate_interval_paths_and_segmented_schedules();

}

void CoordinatedMotionProfiler::calculate_schedule_intervals()
{
    std::vector<std::vector<int>> temp_schedules(physical_paths.size());
    std::vector<std::vector<std::vector<int>>> intervaled_schedules(physical_paths.size());

    Point current_point = temporal_path[0].point;

    // Initial point is (0, 0, 0, ...) in each iteration each axis will be checked
    for ( int i = 0; i < current_point.dimension; i++ )
    {
        temp_schedules[i].push_back(0);
    }

    for ( int i = 1; i < temporal_path.size(); i++ )
    {
        Point next_point;

        if ( i != temporal_path.size() - 1)
        {
            next_point = temporal_path[i].point;
        }
        else
        {
            next_point.dimension = temporal_path[i].point.dimension;
            for ( int k = 0; k < temporal_path[i].point.coordinates.size(); k++ )
            {
                next_point.coordinates.push_back(temporal_path[i].point.coordinates[k] - 1);
            } 
        }
        // Every 2 node represent an interval, the length of the interval is the length
        // of the longest axis traversal.
        std::vector<std::vector<int>> cur_intervals(physical_paths.size());
        while ( current_point.coordinates != next_point.coordinates )
        {
            for ( int k = 0; k < current_point.coordinates.size(); k++)
            {
                if (current_point.coordinates[k] == next_point.coordinates[k])
                {
                    ;
                }
                else if (current_point.coordinates[k] < next_point.coordinates[k])
                {
                    int schedule_to_push = temp_schedules[k].back() + 1;
                    temp_schedules[k].push_back(schedule_to_push);
                    cur_intervals[k].push_back(schedule_to_push);
                    current_point.coordinates[k] += 1;
                }
                else
                {
                    int schedule_to_push = temp_schedules[k].back() - 1;
                    temp_schedules[k].push_back(schedule_to_push);
                    cur_intervals[k].push_back(schedule_to_push);
                    current_point.coordinates[k] -= 1;
                }
            }
        }
        for ( int a = 0; a < intervaled_schedules.size(); a++ )
        {
            intervaled_schedules[a].push_back(cur_intervals[a]);
        }
    }
    this->intervaled_schedules = intervaled_schedules;
}

void CoordinatedMotionProfiler::convert_to_movement_directions()
{
    for ( int i = 0; i < intervaled_schedules.size(); i++ )
    {
        int start_node = 0;
        int end_node = 0;
        for ( int k = 0; k < intervaled_schedules[i].size(); k++ )
        {
            std::vector<int> movement_direction;
            int current_movement;
            if ( intervaled_schedules[i][k].size() == 0)
            {
                current_movement = WAIT;
            }
            else
            {
                start_node = end_node;
                end_node = intervaled_schedules[i][k][intervaled_schedules[i][k].size() - 1];

                if ( start_node < end_node )
                {
                    current_movement = FORWARD;
                }
                else
                {
                    current_movement = BACKWARD;
                }
            }
            movement_direction.push_back(start_node);
            movement_direction.push_back(end_node);
            movement_direction.push_back(current_movement);
            movement_directions[i].push_back(movement_direction);
        }
    }
}

void CoordinatedMotionProfiler::calculate_interval_paths_and_segmented_schedules()
{
    int interval_amount = movement_directions[0].size();
    std::vector<double> current_vels = {0.0, 0.0, 0.0, 0.0};
    for ( int i = 0; i < interval_amount; i++ )
    {
        std::vector<std::vector<double>> current_profiler_requirements;
        for ( int k = 0; k < movement_directions.size(); k++ )
        {
            std::vector<int> current_movement = movement_directions[k][i];
            std::vector<double> motion_profiler_info;

            if ( current_movement[2] == WAIT )
            {
                motion_profiler_info = {0.0, 0.0, 0.0, current_movement[2]};
            }
            else if ( current_movement[2] == FORWARD || current_movement[2] == BACKWARD )
            {
                int start_point_index;
                if ( i != 0 )
                {
                    start_point_index = movement_directions[k][i-1][1];
                }
                else
                {
                    start_point_index = current_movement[0];
                }
                int end_point_index = current_movement[1];

                double distance_within_next_node;


                if ( i == movement_directions[k].size() - 1 || movement_directions[k][i+1][2] == WAIT || 
                    ( current_movement[2] == FORWARD && movement_directions[k][i+1][2] == BACKWARD ) ||
                    ( current_movement[2] == BACKWARD && movement_directions[k][i+1][2] == FORWARD ) )
                {
                    distance_within_next_node = 0.0;
                }
                else
                {
                    distance_within_next_node = calculate_total_distance(physical_paths[k].get_final_physical_path_points(), 
                        movement_directions[k][i][1], movement_directions[k][i+1][1]);
                }

                double distance_within_node = calculate_total_distance(physical_paths[k].get_final_physical_path_points(), 
                    start_point_index, end_point_index);

                motion_profiler_info = {distance_within_node, distance_within_next_node, current_vels[k], current_movement[2]};
            }
            current_profiler_requirements.push_back(motion_profiler_info);
        }
        auto current_profile = NodeSchedule(current_profiler_requirements, max_velocity, max_accel, dt);
        node_profiles.push_back(current_profile.get_profile());
        current_vels = current_profile.get_end_vels();
    }
}
