#include "../include/velocity_scheduler.h"

VelocityScheduler::VelocityScheduler(std::vector<std::vector<Point>> physical_paths, 
	std::vector<Node> temporal_path, double max_velocity)
{
	this->physical_paths = physical_paths;
	this->temporal_path = temporal_path;
	this->physical_path_amount = physical_paths.size();
	this->max_velocity = max_velocity;
	    
    schedules.resize(physical_path_amount);
    schedule_intervals.resize(physical_path_amount);

    final_duration_schedules.resize(physical_path_amount);
    final_velocity_schedules.resize(physical_path_amount);
    final_orientation_schedules.resize(physical_path_amount);
    final_angular_velocity_schedules.resize(physical_path_amount);


    // Calculate extended schedules and schedule intervals
    calculate_intervals_and_schedules();

    // Calculate the physical distance at each schedule interval
    calculate_distances_between_intervals();

    // Calculate average velocity each robot must travel with at each interval
    calculate_average_velocities_and_total_times_per_interval();

    // Calculate the travel duration at each point on physical path
    calculate_final_schedule();

    // Calculate angular velocities for each point on schedule
	calculate_angular_velocities();
    // Broadcast the results as (velocity, duration) pair that corresponds to each point at physical paths
    // this.get_velocity_duration_orientation

    for ( int i = 0; i < 1; i++)
    {
    	std::cout << "lc " << i << std::endl;
    	for ( auto t : final_angular_velocity_schedules[i])
    	{
    		std::cout << t << " ";
    	}
    	std::cout << std::endl;

    }
    for ( int i = 0; i < 1; i++)
    {
    	std::cout << "lc " << i << std::endl;
    	for ( auto t : final_duration_schedules[i])
    	{
    		std::cout << t << " ";
    	}
    	std::cout << std::endl;

    }
    for ( int i = 0; i < 1; i++)
    {
    	std::cout << "lc " << i << std::endl;
    	for ( auto t : final_velocity_schedules[i])
    	{
    		std::cout << t << " ";
    	}
    	std::cout << std::endl;

    }

}


void VelocityScheduler::calculate_intervals_and_schedules()
{
    std::vector<std::vector<int>> temp_schedules(physical_path_amount);

    // Initial point is (0, 0, 0, ...) in each iteration each axis will be checked
    Point current_point = temporal_path[0].point;

    for ( int i = 0; i < current_point.dimension; i++ )
    {
        temp_schedules[i].push_back(0);
    }
    	int tot = 0;

    for ( int i = 1; i < temporal_path.size(); i++ )
    {
        Point next_point = temporal_path[i].point;

        // Every 2 node represent an interval, the length of the interval is the length
        // of the longest axis traversal. We extend every other shorter travelsal by
        // adding a duplicate to the schedule (representing wait)
        std::vector<int> interval_lengths(current_point.dimension, 0);
        while ( current_point.coordinates != next_point.coordinates )
        {
            for ( int k = 0; k < current_point.dimension; k++)
            {
                if (current_point.coordinates[k] == next_point.coordinates[k])
                {
                    ;
                }
                else if (current_point.coordinates[k] < next_point.coordinates[k])
                {
                    temp_schedules[k].push_back(temp_schedules[k].back() + 1);
                    current_point.coordinates[k] += 1;
                    interval_lengths[k]++;  

                }
                else
                {
                    temp_schedules[k].push_back(temp_schedules[k].back() - 1);
                    current_point.coordinates[k] -= 1;
                    interval_lengths[k]++;  
                }
            }
        }
        for ( int a = 0; a < schedule_intervals.size(); a++ )
        {
        	schedule_intervals[a].push_back(interval_lengths[a]);
        }
    }
    schedules = temp_schedules;
}

void VelocityScheduler::calculate_distances_between_intervals()
{
    for ( int i = 0; i < physical_path_amount; i++)
    {
        std::vector<double> current_distances;

        int first_index = -1;
        int next_index = schedule_intervals[i][0];

        for ( int n = 0; n < schedule_intervals[i].size(); n++ )
        {
        	if ( n != 0 )
        	{
        		first_index += schedule_intervals[i][n-1];
        		next_index += schedule_intervals[i][n];
        	}
       		current_distances.push_back(calculate_total_distance(physical_paths[i], 
       			schedules[i][first_index + 1], schedules[i][next_index]));
        }
        schedule_interval_distances_list.push_back(current_distances);
    }
}


void VelocityScheduler::calculate_average_velocities_and_total_times_per_interval()
{
	std::vector<std::vector<double>> average_velocities_lists(physical_path_amount);

	for ( int i = 0; i < schedule_interval_distances_list[0].size(); i++)
	{
	    double max_dist = 0.0;
	    int longest_segment = -1;
	   
	    for ( int k = 0; k < physical_path_amount; k++ )
	    {
	    	if ( schedule_interval_distances_list[k][i] > max_dist)
	    	{
	    		max_dist = schedule_interval_distances_list[k][i];
	    		longest_segment = k;
	    	}
	    }

	    for ( int t = 0; t < physical_path_amount; t++ )
	    {
	    	average_velocities_lists[t].push_back(max_velocity * schedule_interval_distances_list[t][i] / max_dist );
	    }
	    schedule_interval_times.push_back(max_dist / max_velocity);
	}
    schedule_interval_average_velocities = average_velocities_lists;
}


void VelocityScheduler::calculate_final_schedule()
{
	for ( int i = 0; i < physical_path_amount; i++)
	{
		int current_interval_start = -1;
		int current_interval_end = schedule_intervals[i][0];

		std::vector<int> unique_schedule;
		int schedule_count = 0;

		std::vector<std::vector<int>> points_on_intervals;

		for( int  s = 0; s < (schedule_intervals[i].size() ); s++)
		{
			if ( s != 0 )
			{
				current_interval_start += schedule_intervals[i][s-1];
        		current_interval_end += schedule_intervals[i][s];
			}
			std::vector<int> current_points = exclude_number(schedules[i], -1, current_interval_start + 1, current_interval_end);

			for ( auto u_point : current_points )
			{
				unique_schedule.push_back(u_point);
			}

			points_on_intervals.push_back(current_points);

			if ( s == 0 )
			{
				current_interval_start++;
			}
		}

		for( int  k = 0; k < (schedule_intervals[i].size() ); k++)
		{	
			double current_total_time = schedule_interval_times[k];
			double total_distance = schedule_interval_distances_list[i][k];

			if ( points_on_intervals[k].size() == 0 )
			{
				final_duration_schedules[i].push_back(current_total_time);
				final_velocity_schedules[i].push_back(0.0);
				final_orientation_schedules[i].push_back(final_orientation_schedules[i].back());
			}

			for ( int m = 0; m < points_on_intervals[k].size(); m++ )
			{
				int t = unique_schedule[schedule_count];
				int t_next;

				if ( schedule_count == unique_schedule.size() - 1)
				{
					t_next = unique_schedule[schedule_count];
				}
				else
				{
					t_next = unique_schedule[schedule_count+1];
				}
				schedule_count++;

				if (t == -1)
				{
					continue;
				}

				double duration = 0.0;
				double current_distance = calculate_distance(physical_paths[i][t], physical_paths[i][t_next]);

				if (current_total_time > 0.0 && total_distance > current_distance )
				{
					duration = current_total_time * current_distance / total_distance;
				}
				else if ( total_distance == 0.0 )
				{
					duration = current_total_time / points_on_intervals[k].size();
				}
				double orientation = calculate_orientation(physical_paths[i][t], physical_paths[i][t_next]);

				final_duration_schedules[i].push_back(duration);
				final_velocity_schedules[i].push_back(schedule_interval_average_velocities[i][k] / 10.0);
				final_orientation_schedules[i].push_back(orientation);
			}
		}
	}
}

void VelocityScheduler::calculate_angular_velocities()
{
	for (int i = 0; i < final_orientation_schedules.size(); i++)
	{
		for ( int k = 0; k < final_orientation_schedules[i].size() - 1; k++ )
		{
			double angular_velocity = calculate_angular_velocity(final_orientation_schedules[i][k], 
				final_orientation_schedules[i][k+1], final_duration_schedules[i][k]);
			final_angular_velocity_schedules[i].push_back(angular_velocity);
		}
		final_angular_velocity_schedules[i].push_back(0);
	}
}

std::vector<std::vector<std::vector<double>>> VelocityScheduler::get_velocity_duration_orientation()
{
    std::vector<std::vector<std::vector<double>>> velocity_duration_orientations;

    for (int i = 0; i < physical_path_amount; i++)
    {
        std::vector<std::vector<double>> current_path;

        for (int j = 0; j < final_duration_schedules[i].size(); j++)
        {
        	std::vector<double> current_msg;
            double velocity = final_velocity_schedules[i][j];
            double duration = final_duration_schedules[i][j];
            double orientation = final_orientation_schedules[i][j];
            double angular_velocity = final_angular_velocity_schedules[i][j];

            current_msg.push_back(velocity);
            current_msg.push_back(duration);
            current_msg.push_back(orientation);
            current_msg.push_back(angular_velocity);

            current_path.push_back(current_msg);
        }

        velocity_duration_orientations.push_back(current_path);
    }

    return velocity_duration_orientations;
}