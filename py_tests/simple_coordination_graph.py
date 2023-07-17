import random
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math

def generate_curve(length_min, length_max, num_segments, min_num_stops, max_num_stops):
    # Generate random starting points for the curve
    
    num_stops = random.randint(min_num_stops, max_num_stops)
    
    partial_segments = num_segments // (num_stops - 1)
    
    
    stops = []
    
    # Generate stop points
    for i in range(num_stops):
        stops.append(( random.randint(0,100), random.randint(0,100)))
    
    x = np.linspace(stops[0][0], stops[1][0], num=partial_segments)
    y = np.linspace(stops[0][1], stops[1][1], num=partial_segments)
    # Generate points for the curve segments
    for i in range(1, len(stops) - 1):
        x = np.concatenate((x, np.linspace(stops[i][0], stops[i+1][0], num=partial_segments)))
        y = np.concatenate((y, np.linspace(stops[i][1], stops[i+1][1], num=partial_segments)))

    return x, y

def find_collision_points(x1, y1, x2, y2, r):
    # Find all collision points between the two curves
    collision_points_x = []
    collision_points_y = []
    for i in range(len(x1)):
        for k in range(len(x2)):
            # Calculate the distance between the points on each segment
            distance = np.sqrt((x1[i] - x2[k])**2 + (y1[i] - y2[k])**2)


            # Check if the distance is less than or equal to the sum of the radius
            if distance <= 2 * r:

                # Calculate the time passed on each line when the collision occurs
                collision_points_x.append(i)
                collision_points_y.append(k)

    return collision_points_x, collision_points_y

def points_around_point(x, y, r):
    #The lower this value the higher quality the circle is with more points generated
    stepSize = 0.05
    
    #Generated vertices
    positions_x = []
    positions_y = []
    
    t = 0
    while t < 2 * math.pi:
        positions_x.append(r * math.cos(t) + x)
        positions_y.append( r * math.sin(t) + y)
        t += stepSize
        
    return positions_x, positions_y

# Define the minimum and maximum lengths, the number of segments, and the radius of the objects
length_min = 20
length_max = 30
num_segments = 1260
radius = 10

# Generate the first curve
x1, y1 = generate_curve(length_min, length_max, num_segments, 3, 5)

# Generate the second curve
x2, y2 = generate_curve(length_min, length_max, num_segments, 3, 5)

# Calculate the collision points
collision_points_x, collision_points_y = find_collision_points(x1, y1, x2, y2, radius)

circle1_x, circle1_y = points_around_point(x1[0], y1[0], radius)
circle2_x, circle2_y = points_around_point(x2[0], y2[0], radius)

# Plot the first curve with round markers at the beginning of lines
plt.figure()
plt.scatter(x1[collision_points_x], y1[collision_points_x],c = "r")
plt.scatter(x2[collision_points_y], y2[collision_points_y],c = "y")
plt.plot(x1, y1, '-o', markersize=1)
plt.text(x1[0], y1[0], "line 1")
plt.scatter(circle1_x, circle1_y,c = "black")
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Routes')
plt.grid()

plt.plot(x2, y2, '-o', markersize=1)
plt.text(x2[0], y2[0], "line 2")
plt.scatter(circle2_x, circle2_y,c = "gray")

total_travel_time_1 = num_segments
total_travel_time_2 = num_segments

# Plot the collision points
plt.figure()
plt.plot(collision_points_x, collision_points_y, 'ro')
plt.xlabel('Time 1')
plt.ylabel('Time 2')
plt.xlim(0, total_travel_time_1)
plt.ylim(0, total_travel_time_2)
plt.title('Collision Points')

collusion_points = set(zip(collision_points_x, collision_points_y))
print(collusion_points)

rrt_p_queue = []
heapq.heapify(rrt_p_queue)

rrt_came_from = {(0, 0): ()}
rrt_cost = {(0, 0): 0}

heapq.heappush(rrt_p_queue, (0, (0, 0)))

while len(rrt_p_queue) != 0:
    current = heapq.heappop(rrt_p_queue)
    if current[1] == (num_segments, num_segments):
        break

    neighbours = []
    for i in range(3):
        for k in range(3):
            if (
                current[1] not in collusion_points
                and (i != 0 or k != 0)
                and 0 <= current[1][0] + i - 1 <= num_segments
                and 0 <= current[1][1] + k - 1 <= num_segments
            ):
                neighbours.append((current[1][0] + i - 1, current[1][1] + k - 1))

    if len(neighbours) == 0:
        continue

    for next_n in neighbours:
        new_cost = (
            rrt_cost[current[1]]
            + np.sqrt((next_n[0] - current[1][0]) ** 2 + (next_n[1] - current[1][1]) ** 2)
        )
        if next_n not in rrt_cost or new_cost < rrt_cost[next_n]:
            rrt_cost[next_n] = new_cost
            priority = (
                new_cost + np.sqrt((next_n[0] - num_segments) ** 2 + (next_n[1] - num_segments) ** 2)
            )
            heapq.heappush(rrt_p_queue, (priority, next_n))
            rrt_came_from[next_n] = current[1]

final_pos_x = [num_segments]
final_pos_y = [num_segments]
final_pos = (num_segments, num_segments)

while final_pos != (0, 0):
    final_pos_x.append(final_pos[0])
    final_pos_y.append(final_pos[1])
    final_pos = rrt_came_from[final_pos]

# Plot the collision points and final positions together
plt.plot(collision_points_x, collision_points_y, 'ro', label='Collision Points')
plt.scatter(final_pos_x, final_pos_y, c='g', label='Final Position')
plt.xlabel('Time 1')
plt.ylabel('Time 2')
plt.xlim(0, total_travel_time_1)
plt.ylim(0, total_travel_time_2)
plt.title('Collision Points and Final Positions')
plt.grid()
plt.legend()

plt.show()

