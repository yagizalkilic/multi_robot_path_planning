#include "../../include/node_schedule.h" 

// Movement constants
const int FORWARD = 0;
const int WAIT = 1;
const int BACKWARD = 2;

NodeSchedule::NodeSchedule(std::vector<std::vector<double>> schedule_info, double max_velocity, double max_accel, double dt)
{
	for ( auto i : schedule_info )
	{
		distance_within_nodes.push_back(i[0]);
		distance_within_next_nodes.push_back(i[1]);
		start_vels.push_back(i[2]);
		movement_types.push_back(i[3]);
	}

	this->agv_amount = schedule_info.size();
	this->max_vel = max_velocity;
	this->max_acc = max_accel;
	this->dt = dt;

	final_duration = 0.0;

	end_vels.resize(agv_amount, 0.0);
	any_time_scheduler_times.resize(agv_amount, 0.0);
	any_time_plans.resize(agv_amount, std::vector<std::pair<double, double>>());
	time_constrained_plans.resize(agv_amount, std::vector<std::pair<double, double>>());

	calculate_max_final_vels();
	node_scheduler_any_time();
	node_scheduler_time_constrained();
	arrange_backward_movement();
}

void NodeSchedule::calculate_max_final_vels()
{
	for ( int i = 0; i < agv_amount; i++ )
	{
		if ( distance_within_next_nodes[i] == 0.0 )
		{
			max_end_vels.push_back(0.0);
		}
		else
		{
			double restricting_distance = distance_within_next_nodes[i];
			double max_start_vel_next_node = calculate_max_speed_before_deacc(max_acc, restricting_distance);

			if ( max_start_vel_next_node > max_vel )
			{
				max_end_vels.push_back(max_vel);
			}
			else
			{
				max_end_vels.push_back(max_start_vel_next_node);
			}
		}
	}
}

void NodeSchedule::node_scheduler_any_time()
{
	for ( int i = 0; i < agv_amount; i++ )
	{
		double total_distance_to_cover = distance_within_nodes[i];
		if ( movement_types[i] == WAIT )
		{
			any_time_plans[i] = std::vector<std::pair<double, double>>();
			any_time_scheduler_times[i] = 0.0;
		}
		else
		{
			double start_vel = start_vels[i];
			double max_end_vel = max_end_vels[i];
			any_time_plans[i] = trapezoidal_velocity_planner(start_vel, max_end_vel, max_vel, total_distance_to_cover);
			any_time_scheduler_times[i] = any_time_plans[i].back().first;
		}
	}
	longest_any_time_plan = 0;
	double plan_time = 0;
	for ( int i = 0; i < any_time_scheduler_times.size(); i++)
	{
		if ( any_time_scheduler_times[i] > plan_time )
		{
			plan_time = any_time_scheduler_times[i];
			longest_any_time_plan = i;
		}
	}
}

void NodeSchedule::node_scheduler_time_constrained()
{
    double node_time = any_time_scheduler_times[longest_any_time_plan];
    for ( int i = 0; i < any_time_plans.size(); i++ )
    {
    	if ( movement_types[i] == WAIT )
		{
			std::vector<std::pair<double, double>> wait_schedule;
			double wait_time = 0.0;
			while ( wait_time < node_time)
			{
				wait_schedule.push_back(std::make_pair(wait_time, 0.0));
				wait_time += dt;
			}
			time_constrained_plans[i] = wait_schedule;
			end_vels[i] = 0.0;
		}
		else
		{
			if ( i != longest_any_time_plan && any_time_plans[i].size() == 4 )
			{
				double k_value = compute_k(any_time_plans[i][0].second, any_time_plans[i][1].second, any_time_plans[i][3].second, 
					any_time_plans[i][1].first, any_time_plans[i][2].first - any_time_plans[i][1].first, 
					any_time_plans[i][3].first - any_time_plans[i][2].first, node_time, max_acc, distance_within_nodes[i]);

				if ( k_value == 0.0 || std::isnan(k_value))
				{
		    		std::vector<std::pair<double, double>> new_plan = quadratic_scheduler(start_vels[i], max_end_vels[i], max_vel, distance_within_nodes[i], node_time);
		    		end_vels[i] = new_plan.back().second;
		    		time_constrained_plans[i] = new_plan;
				}
				else
				{
					auto k_schedule = get_new_schedule_k(any_time_plans[i][0].second, any_time_plans[i][1].second, any_time_plans[i][3].second, 
						any_time_plans[i][1].first, any_time_plans[i][2].first - any_time_plans[i][1].first, 
						any_time_plans[i][3].first - any_time_plans[i][2].first, node_time, max_acc, distance_within_nodes[i], k_value);

					std::vector<std::pair<double, double>> new_plan = segment_schedule(k_schedule);
		    		time_constrained_plans[i] = new_plan;
		    		end_vels[i] = new_plan.back().second; 
				}
			}
	    	else if ( i != longest_any_time_plan )
	    	{
	    		std::vector<std::pair<double, double>> new_plan = quadratic_scheduler(start_vels[i], max_end_vels[i], max_vel, distance_within_nodes[i], node_time);
	    		end_vels[i] = new_plan.back().second;
	    		time_constrained_plans[i] = new_plan;
	    	}
	    	else
	    	{
	    		std::vector<std::pair<double, double>> new_plan = segment_schedule(any_time_plans[i]);
	    		time_constrained_plans[i] = new_plan;
	    		end_vels[i] = new_plan.back().second;
	    	}
	    }
    }
}

std::vector<std::pair<double, double>> NodeSchedule::quadratic_scheduler(double start_vel, double max_end_vel, double max_vel, double distance, double duration)
{
    double quadratic_a = 3.0 * (max_end_vel - start_vel) / duration / duration + 6.0 * start_vel / duration / duration - 6.0 * distance / std::pow(duration, 3);

    double quadratic_b = 6.0 * distance / duration / duration - 6.0 * start_vel / duration - 2.0 * (max_end_vel - start_vel) / duration;
    
    double quadratic_c = start_vel;

    std::vector<std::pair<double, double>> velocity_set;

    double current_time = 0.0;
    double total_dist = 0.0;
    double current_vel = start_vel;

    while ( current_time < duration )
    {
    	current_time += dt;
    	current_vel = quadratic_a * current_time * current_time + quadratic_b * current_time + quadratic_c;
    	total_dist += current_vel * dt;
    	velocity_set.push_back(std::make_pair(current_time, current_vel));

    }
    return velocity_set;
}

std::vector<std::pair<double, double>> NodeSchedule::segment_schedule(std::vector<std::pair<double, double>> current_schedule)
{
    std::vector<std::pair<double, double>> interpolated_points;
    for (size_t i = 0; i < current_schedule.size() - 1; ++i) 
    {
        std::pair<double, double> start = current_schedule[i];
        std::pair<double, double> end = current_schedule[i + 1];

        double time_distance = end.first - start.first;

        if ( -0.01 <= time_distance && time_distance <= 0.01 )
        {
        	continue;
        }

        int num_segments = static_cast<int>(time_distance / dt);

        double deltaX = (end.first - start.first) / num_segments;
        double deltaY = (end.second - start.second) / num_segments;

        for (int j = 0; j <= num_segments; ++j) 
        {
            double newX = start.first + j * deltaX;
            double newY = start.second + j * deltaY;
            interpolated_points.emplace_back(newX, newY);
        }
    }

    return interpolated_points;
}

std::vector<std::pair<double, double>> NodeSchedule::trapezoidal_velocity_planner(double start_vel, double max_end_vel, double max_vel, double distance)
{
	std::vector<std::pair<double, double>> stops;

	double time_acc_best = (std::sqrt(4.0 * max_acc * distance + 2.0 * start_vel * start_vel + 2.0 * max_end_vel * max_end_vel)) / ( 2.0 * max_acc );
	double time_acc;
	bool can_maintain = false;
	double distance_passed = 0.0;
	double time_passed = 0.0;

	stops.push_back(std::make_pair(time_passed, start_vel));

	if ( time_acc_best >= 0.0 )
	{
		if ( time_acc_best * max_acc + start_vel >= max_end_vel )
		{
			if ( time_acc_best <= (max_vel - start_vel) / max_acc )
			{
				time_acc = time_acc_best;
			}
			else
			{
				time_acc = ( max_vel - start_vel ) / max_acc;
				can_maintain = true;
			}

			time_passed += time_acc;
			distance_passed += time_acc * start_vel + max_acc * time_acc * time_acc / 2.0;
			stops.push_back(std::make_pair(time_passed, start_vel + (time_acc * max_acc)));

			if ( !can_maintain )
			{
				double cur_step_time = ((start_vel + (time_acc * max_acc) - max_end_vel) / max_acc);
				time_passed += cur_step_time;
				distance_passed += (cur_step_time * max_end_vel) + (max_acc * cur_step_time * cur_step_time / 2.0);
				stops.push_back(std::make_pair(time_passed, max_end_vel));
			}
			else
			{
				double remaining_distance = distance - distance_passed;

				double to_end_time = (max_vel - max_end_vel) / max_acc;
				double to_end_dist = max_end_vel * to_end_time + (max_acc * to_end_time * to_end_time / 2.0);

				double maintain_distance = remaining_distance - to_end_dist;
				double maintain_time = maintain_distance / max_vel;

				time_passed += maintain_time;
				distance_passed += maintain_distance;

				stops.push_back(std::make_pair(time_passed, max_vel));

				time_passed += to_end_time;
				distance_passed += to_end_dist;

				stops.push_back(std::make_pair(time_passed, max_end_vel));
			}
		}
		else
		{
			time_acc = (std::sqrt(2.0 * max_acc * distance + start_vel * start_vel) - start_vel) / max_acc;
			double v_early = time_acc * max_acc + start_vel;
			double early_distance = start_vel + start_vel * time_acc + max_acc * time_acc * time_acc / 2.0;
			distance_passed += early_distance;
			time_passed += time_acc;
			stops.push_back(std::make_pair(time_passed, v_early));
		}
	}
	else
	{
		double direction = max_acc;
		if ( start_vel > max_end_vel )
		{
			direction = -max_acc;
		}
		time_acc = (std::sqrt(std::abs(2.0 * direction * distance + start_vel * start_vel)) - start_vel) / direction;
		time_passed += time_acc;
		distance_passed += time_acc * start_vel + direction * time_acc * time_acc / 2.0;
		stops.push_back(std::make_pair(time_passed, start_vel + time_acc * direction));
	}
	return stops;
}

void NodeSchedule::arrange_backward_movement()
{
	for ( int i = 0; i < agv_amount; i++ )
	{
		if ( movement_types[i] == BACKWARD )
		{
			for ( int k = 0; k < time_constrained_plans[i].size(); k++ )
			{
				time_constrained_plans[i][k].second = -time_constrained_plans[i][k].second;
			}
		}
	}
}

double NodeSchedule::compute_k(double V_1, double V_2, double V_3, double t_1, double t_2, double t_3, double T, double a, double D) 
{
    double denominator = 2 * a * V_2 * (std::pow(V_3, 2) - std::pow(V_1, 2));
    
    if (denominator != 0) 
    {
        double term1 = (-std::sqrt(std::pow(a * V_2 * std::pow(V_1, 2) - a * V_2 * std::pow(V_3, 2) - 2 * a + 2 * V_1 + 2 * V_3, 2) - 4 * (a * V_2 * std::pow(V_3, 2) - a * std::pow(V_1, 2) * V_2) * (2 * a * D * V_2 - 2 * a * t_1 * V_2 - 2 * a * t_2 * V_2 - 2 * a * t_3 * V_2 + 2 * a * T * V_2 - 2 * V_1 - 2 * V_3)) - a * V_2 * std::pow(V_1, 2) + a * V_2 * std::pow(V_3, 2) + 2 * a - 2 * V_1 - 2 * V_3);
        double term2 = (2 * std::pow(a, 2) * t_1 * std::pow(V_2, 2) * V_1 + 2 * std::pow(a, 2) * t_2 * std::pow(V_2, 2) * V_1 + 2 * std::pow(a, 2) * t_3 * std::pow(V_2, 2) * V_1 - 2 * std::pow(a, 2) * t_1 * std::pow(V_2, 2) * V_3 - 2 * std::pow(a, 2) * t_2 * std::pow(V_2, 2) * V_3 - 2 * std::pow(a, 2) * t_3 * std::pow(V_2, 2) * V_3 - 2 * std::pow(a, 2) * T * std::pow(V_2, 2) * V_1 + 2 * std::pow(a, 2) * T * std::pow(V_2, 2) * V_3 - std::sqrt(std::pow(a * V_2 * std::pow(V_1, 2) - a * V_2 * std::pow(V_3, 2) - 2 * a + 2 * V_1 + 2 * V_3, 2) - 4 * (a * V_2 * std::pow(V_3, 2) - a * std::pow(V_1, 2) * V_2) * (2 * a * D * V_2 - 2 * a * t_1 * V_2 - 2 * a * t_2 * V_2 - 2 * a * t_3 * V_2 + 2 * a * T * V_2 - 2 * V_1 - 2 * V_3)) + a * V_2 * std::pow(V_1, 2) - a * V_2 * std::pow(V_3, 2) + 2 * a - 2 * V_1 - 2 * V_3);

        if (term1 != 0 && term2 != 0) {
            double k = term1 / denominator;
            std::cout << "k = " << k << std::endl;
            return k;
        }
        else {
            std::cout << "Numerator1 or Numerator2 is zero, cannot compute k." << std::endl;
        }
    }
    else 
    {
    	double k = D * V_2 - t_1 * V_2 - t_2 * V_2 - t_3 * V_2 + T * V_2;
        std::cout << "Denominator is zero, k is undefined." << std::endl;
        return k;
    }

    return 0.0; // Return 0.0 as a default value if an error occurs.
}

std::vector<std::pair<double, double>> NodeSchedule::get_new_schedule_k(double V_1, double V_2, double V_3, double t_1, double t_2, double t_3, double T, double a, double D, double k_value) 
{
    double current_time = 0;
    std::vector<std::pair<double, double>> k_list;
    double total_dist = 0;
    
    double k = k_value;

    double V_1_n = V_1 / k;
    double V_2_n = V_2 / k;
    double V_3_n = V_3 / k;
    
    k_list.push_back(std::make_pair(0, V_1));
    
    current_time += std::abs(V_1 - V_1_n) / a;
    total_dist += V_1_n * std::abs(V_1 - V_1_n) / a + std::abs(V_1 - V_1_n) * std::abs(V_1 - V_1_n) / (2 * a);
    k_list.push_back(std::make_pair(current_time, V_1_n));
    
    current_time += t_1;
    total_dist += (V_2_n - V_1_n) / a * V_1_n + (V_2_n - V_1_n) / (2 * a) * (V_2_n - V_1_n);
    k_list.push_back(std::make_pair(current_time, V_2_n));
    
    current_time += (T - t_1 - t_3 - std::abs(V_1 - V_1_n) / a - std::abs(V_3 - V_3_n) / a);
    total_dist += (T - t_1 - t_3 - std::abs(V_1 - V_1_n) / a - std::abs(V_3 - V_3_n) / a) * V_2_n;
    k_list.push_back(std::make_pair(current_time, V_2_n));
    
    current_time += t_3;
    total_dist += (V_2_n - V_3_n) / a * V_3_n + (V_2_n - V_3_n) / (2 * a) * (V_2_n - V_3_n);
    k_list.push_back(std::make_pair(current_time, V_3_n));
    
    current_time += std::abs(V_3 - V_3_n) / a;
    total_dist += V_3_n * std::abs(V_3 - V_3_n) / a + std::abs(V_3 - V_3_n) * std::abs(V_3 - V_3_n) / (2 * a);
    k_list.push_back(std::make_pair(current_time, V_3));
    
    return k_list;
}