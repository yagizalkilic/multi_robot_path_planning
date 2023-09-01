import random
import math
import matplotlib.pyplot as plt
from sympy import symbols, sqrt


a = 0.5
v_max = 0.4
d = 0.55

def quadratic_velocity(a, b, c, t):
    quadratic_list = []
    quadratic_accel_list = []
    current_time = 0;
    total_dist = 0;
    dt = 0.05
    while current_time < t:
        current_time += dt
        current_vel = a * current_time * current_time + b * current_time + c
        current_dist = current_vel * dt
        total_dist += current_dist
        quadratic_list.append(( current_time, current_vel, total_dist))
        quadratic_accel_list.append(2 * a * current_time + b)
        
        
    return quadratic_list, total_dist, quadratic_accel_list

def compute_K(V_1,V_2,V_3,t_1,t_2,t_3,T,a, D):

    # Calculate the denominator value
    denominator = 2 * a * V_2 * (V_3**2 - V_1**2)
    
    # Check if the denominator is not zero
    if denominator != 0:
    
        # Calculate the numerator values
        numerator1 = (-math.sqrt((a * V_2 * V_1**2 - a * V_2 * V_3**2 - 2 * a + 2 * V_1 + 2 * V_3)**2 - 4 * (a * V_2 * V_3**2 - a * V_1**2 * V_2) * (2 * a * D * V_2 - 2 * a * t_1 * V_2 - 2 * a * t_2 * V_2 - 2 * a * t_3 * V_2 + 2 * a * T * V_2 - 2 * V_1 - 2 * V_3)) - a * V_2 * V_1**2 + a * V_2 * V_3**2 + 2 * a - 2 * V_1 - 2 * V_3)
        numerator2 = (2 * a**2 * t_1 * V_2**2 * V_1 + 2 * a**2 * t_2 * V_2**2 * V_1 + 2 * a**2 * t_3 * V_2**2 * V_1 - 2 * a**2 * t_1 * V_2**2 * V_3 - 2 * a**2 * t_2 * V_2**2 * V_3 - 2 * a**2 * t_3 * V_2**2 * V_3 - 2 * a**2 * T * V_2**2 * V_1 + 2 * a**2 * T * V_2**2 * V_3 - math.sqrt((a * V_2 * V_1**2 - a * V_2 * V_3**2 - 2 * a + 2 * V_1 + 2 * V_3)**2 - 4 * (a * V_2 * V_3**2 - a * V_1**2 * V_2) * (2 * a * D * V_2 - 2 * a * t_1 * V_2 - 2 * a * t_2 * V_2 - 2 * a * t_3 * V_2 + 2 * a * T * V_2 - 2 * V_1 - 2 * V_3)) + a * V_2 * V_1**2 - a * V_2 * V_3**2 + 2 * a - 2 * V_1 - 2 * V_3)
    
        # Check if both numerators are not zero
        if numerator1 != 0 and numerator2 != 0:
            k = numerator1 / denominator
            print("k =", k)
        else:
            print("Numerator1 or Numerator2 is zero, cannot compute k.")
    else:
        print("Denominator is zero, k is undefined.")
        
    return k
        
def get_new_schedule_k(V_1,V_2,V_3,t_1,t_2,t_3,T,a,D):
    current_time = 0
    k_list = []
    total_dist = 0
    
    k = compute_K(V_1,V_2,V_3,t_1,t_2,t_3,T,a, D)

    V_1_n = V_1/k
    V_2_n = V_2/k
    V_3_n = V_3/k
    
    k_list.append((0, V_1, 0))
    
    current_time += abs( V_1 - V_1_n) / a
    total_dist += V_1_n * abs( V_1 - V_1_n) / a + abs( V_1 - V_1_n) * abs( V_1 - V_1_n) / a / 2
    k_list.append((current_time, V_1_n, total_dist ))
    
    current_time += t_1
    total_dist += (V_2_n - V_1_n) / a * V_1_n + (V_2_n - V_1_n) / a * (V_2_n - V_1_n) / 2
    k_list.append((current_time, V_2_n, total_dist))
    
    current_time += (T - t_1 - t_3 - abs( V_1 - V_1_n) / a - abs( V_3 - V_3_n) / a)
    total_dist += (T - t_1 - t_3 - abs( V_1 - V_1_n) / a - abs( V_3 - V_3_n) / a ) * V_2_n
    k_list.append((current_time, V_2_n, total_dist))
    
    current_time += t_3
    total_dist += (V_2_n - V_3_n) / a * V_3_n + (V_2_n - V_3_n) / a * (V_2_n - V_3_n) / 2
    k_list.append((current_time, V_3_n, total_dist))
    
    current_time += abs( V_3 - V_3_n) / a
    total_dist += V_3_n * abs( V_3 - V_3_n) / a + abs( V_3 - V_3_n) * abs( V_3 - V_3_n) / a / 2
    k_list.append((current_time, V_3, total_dist ))
    
    return k_list, total_dist
    
    
    
stops = []

vs = 0.0;
for i in range(20):

    vsg = vs
    ve = random.uniform(0.0, 0.4)
    if (i == 99 ):
        ve = 0
    
    stops.append((0, vs, 0))
    
    ts_best = (math.sqrt(4 * a * d + 2 * vs * vs + 2 * ve * ve) - 2 * vs)  / ( 2 * a )
    
    can_maintain = False
    distance_passed = 0;
    time_passed = 0;
    
    if ( ts_best >= 0 ):
        if ( ts_best * a + vs >= ve):
            if ( ts_best <= (v_max - vs) / a):
                ts = ts_best
    
            else:
                ts = ( v_max  - vs)  / a
                can_maintain = True
                
            time_passed += ts
            distance_passed += ts * vs + 1 / 2 * a * ts * ts
            stops.append((time_passed, vs+ts*a, distance_passed))
    
            
            if ( not can_maintain ):
                cur_step_time = ((vs + ts * a) - ve) / a
                time_passed += cur_step_time
                distance_passed += cur_step_time * ve + 1 / 2 * (a) * cur_step_time * cur_step_time
                stops.append(( time_passed , ve, distance_passed))
                vs = ve
                veg = vs
                print(f"up not maintain {i+1}")
    
                
            else:
                remaining_distance = d - distance_passed
                
                to_end_time = (v_max - ve) / a
                to_end_deacc = ve * to_end_time + 1 / 2 * (a) * to_end_time * to_end_time
                
                maintain_distance = remaining_distance - to_end_deacc
                
                maintain_time = maintain_distance / v_max
                time_passed += maintain_time
                distance_passed += maintain_distance
                
                stops.append((time_passed, v_max, distance_passed))
                
                time_passed += to_end_time
                distance_passed += to_end_deacc
                
                stops.append((time_passed, ve, distance_passed))
                vs = ve
                veg = ve
                print(f"maintain {i+1}")
    
                
        else:
            ts = (math.sqrt(2 * a * d + vs * vs) - vs) / a
            v_premature_e = ts * a + vs
            dist = vs * ts + 1 / 2 * a * ts * ts
            distance_passed += dist
            stops.append((ts, v_premature_e, distance_passed))
            vs = v_premature_e
            veg = v_premature_e
            time_passed += ts
            print(f"down not maintain {i+1}")
                
                
    else:
        print(f"never up {i+1}")
        direction = a
        if ( vs > ve):
            direction = -a
            print((2 * direction * d + vs * vs) - vs)
        ts = (math.sqrt(abs(2 * direction * d + vs * vs)) - vs) / direction
        print(ts)
        
        time_passed += ts
        distance_passed += ts * vs + 1 / 2 * direction * ts * ts
        stops.append((time_passed, vs+ts*direction, distance_passed))
        vs = vs+ts*direction
        veg = vs+ts*direction
        
    print(time_passed)
        
    time_passed = 2 * time_passed 

    # alternate_v = 1/2 * (vsg + veg + a * time_passed + math.sqrt(-4 * a * d - veg * veg + 2 * veg * vsg - vsg * vsg + 2 * a * veg * time_passed + 2 * a * vsg * time_passed + a * a * time_passed * time_passed))
    
    # alternate_v = ( -2 * a * d - vsg * vsg + veg * veg) / (2 * (-a * time_passed - vsg + veg))
    
    # a = -a
    
    # alternate_v = (1/4) * (math.sqrt(abs(a**2 * time_passed**2 - 8 * a * d + 2 * a * vsg * time_passed + 6 * veg * a * time_passed - 3 * vsg**2 + 6 * veg * vsg - 3 * veg**2)) + a * time_passed + vsg + 3 * veg)
    
    quadratic_a = 3 * (veg - vsg) / time_passed / time_passed + 6 * vsg / time_passed / time_passed - 6 * d / math.pow(time_passed,3)
    
    quadratic_b = 6 * d / time_passed / time_passed - 6 * vsg / time_passed - 2 * (veg - vsg) / time_passed
    
    quadratic_c = vsg
    
    quadratic, check_dist_quad, quadratic_accel_list = quadratic_velocity(quadratic_a, quadratic_b, quadratic_c, time_passed)
    


    # time_to_alt = abs(abs(alternate_v - vsg) / a)
    # dist_to_alt = time_to_alt * vsg + 1 / 2 * a * time_to_alt * time_to_alt
    
    # time_to_end =abs( abs(alternate_v - veg) / a)
    # dist_to_end = time_to_end * veg + 1 / 2 * a * time_to_end * time_to_end
    
    # time_in_alt = time_passed - time_to_end - time_to_alt
    # dist_in_alt = time_in_alt * alternate_v
    
    # check_dist_alt = dist_in_alt + dist_to_end + dist_to_alt
    
    # alternate_sch = [(0, vsg, 0), (time_to_alt, alternate_v, dist_to_alt), 
    #                  (time_to_alt + time_in_alt, alternate_v, dist_to_alt + dist_in_alt),
    #                  (time_to_alt + time_in_alt + time_to_end, veg, dist_to_alt + dist_in_alt + dist_to_end)]
            
    check_dist = 0
    for n in range(len(stops)-1):
        first_point = stops[n]
        second_point = stops[n+1]
        check_dist += abs(second_point[1] - first_point[1]) * (second_point[0] - first_point[0]) / 2 + \
            (min(first_point[1], second_point[1]) * (second_point[0] - first_point[0]))
    x_values = [stop[0] for stop in stops]
    y_values_velocity = [stop[1] for stop in stops]
    y_values_third = [stop[2] for stop in stops]

    plt.figure()
    plt.plot(x_values, y_values_velocity, marker='o', label='Velocity')
    plt.plot(x_values, y_values_third, marker='x', label='Distance')
    plt.xlabel('Time, s_vel = ' + "{:.4f}".format(vsg) + ', e_vel = ' + "{:.4f}".format(veg) + ', c_dist = ' + "{:.4f}".format(check_dist))
    plt.ylabel('Velocity - Distance')
    plt.title(f'Motion Profile - Iteration {i+1}')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    # for n in range(len(alternate_sch)-1):
    #     first_point = alternate_sch[n]
    #     second_point = alternate_sch[n+1]
    # x_values = [stopa[0] for stopa in alternate_sch]
    # y_values_velocity = [stopa[1] for stopa in alternate_sch]
    # y_values_third = [stopa[2] for stopa in alternate_sch]

    # plt.figure()
    # plt.plot(x_values, y_values_velocity, marker='o', label='Velocity')
    # plt.plot(x_values, y_values_third, marker='x', label='Distance')
    # plt.xlabel('Time, s_vel = ' + "{:.4f}".format(vsg) + ', e_vel = ' + "{:.4f}".format(veg) + ', c_dist = ' + "{:.4f}".format(check_dist_alt))
    # plt.ylabel('Velocity - Distance')
    # plt.title(f'Alternate Motion Profile - Iteration {i+1}')
    # plt.legend()
    # plt.grid(True)
    # plt.show()
    

    x_values = [stopq[0] for stopq in quadratic]
    y_values_velocity = [stopq[1] for stopq in quadratic]
    y_values_third = [stopq[2] for stopq in quadratic]

    plt.figure()
    plt.plot(x_values, y_values_velocity, marker='o', label='Velocity')
    plt.plot(x_values, y_values_third, marker='x', label='Distance')
    plt.plot(x_values, quadratic_accel_list, marker='x', label='Acceleration')
    plt.xlabel('Time, s_vel = ' + "{:.4f}".format(vsg) + ', e_vel = ' + "{:.4f}".format(veg) + ', c_dist = ' + "{:.4f}".format(check_dist_quad))
    plt.ylabel('Velocity - Distance')
    plt.title(f'Quadratic Motion Profile - Iteration {i+1}')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    if len(stops) == 4:
        n_s, n_s_check = get_new_schedule_k(stops[0][1],stops[1][1],stops[3][1],stops[1][0],
                                 stops[2][0]-stops[1][0],stops[3][0] -stops[2][0],
                                 time_passed,a,check_dist)
        
        x_values = [stop[0] for stop in n_s]
        y_values_velocity = [stop[1] for stop in n_s]
        y_values_third = [stop[2] for stop in n_s]

        plt.figure()
        plt.plot(x_values, y_values_velocity, marker='o', label='Velocity')
        plt.plot(x_values, y_values_third, marker='x', label='Distance')
        plt.xlabel('Time, s_vel = ' + "{:.4f}".format(vsg) + ', e_vel = ' + "{:.4f}".format(veg) + ', c_dist = ' + "{:.4f}".format(check_dist))
        plt.ylabel('Velocity - Distance')
        plt.title(f'Motion Profile Enlengthened - Iteration {i+1}')
        plt.legend()
        plt.grid(True)
        plt.show()
    
    
    stops = []

            
            
        