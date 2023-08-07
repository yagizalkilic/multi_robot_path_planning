%matplotlib qt
import random
import numpy as np
import matplotlib.pyplot as plt
import heapq
import math
import time
import datetime
import sys
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from random import randrange

# Functions related to line generation and presentation

# Generate points around a center in specified interval
def points_around_point(x, y, r):
    # The lower this value the higher quality the circle is with more points generated
    stepSize = 0.05
    
    # Generated vertices
    positions_x = []
    positions_y = []
    
    t = 0
    while t < 2 * math.pi:
        positions_x.append(r * math.cos(t) + x)
        positions_y.append( r * math.sin(t) + y)
        t += stepSize
        
    return positions_x, positions_y

# Generate a line a random amount of connected lines, amount has a specified interval
# Points on the lines are returned.
def generate_curve(length_min, length_max, num_segments, min_num_stops, max_num_stops, x_bound, y_bound):
    # Generate a random line amount
    num_stops = random.randint(min_num_stops, max_num_stops)
    
    partial_segments = num_segments // (num_stops - 1)
      
    stops = []
    
    # Generate stop points
    for i in range(num_stops):
        stops.append(( random.randint(0,x_bound), random.randint(0,y_bound)))
    
    # Generate lines and gather the points on them
    x = np.linspace(stops[0][0], stops[1][0], num=partial_segments)
    y = np.linspace(stops[0][1], stops[1][1], num=partial_segments)
    for i in range(1, len(stops) - 1):
        x = np.concatenate((x, np.linspace(stops[i][0], stops[i+1][0], num=partial_segments)))
        y = np.concatenate((y, np.linspace(stops[i][1], stops[i+1][1], num=partial_segments)))

    return x, y

# Functions related to coordination graphs and robot simulation

# Given four lists representing x and y coordinates of points on two lines,
# calculate points where circular objects moving along the line collide
def find_collision_points(x1, y1, x2, y2, r, dimension_length):
    collusion_points = {}
    for i in range(dimension_length):
        for k in range(dimension_length):
            # Calculate the distance between the points on each segment
            distance = np.sqrt((x1[i] - x2[k])**2 + (y1[i] - y2[k])**2)

            # If the distance is less than or equal to 2r there is a collusion
            if distance <= 2 * r:

                # Pass the segment position. The assumption here is that every
                # segment is traversed in the same amount of time. Velocity
                # should  be included for more realistic values
                collusion_points[(i,k)] = True
            else:
                collusion_points[(i,k)] = False

    return collusion_points

# Find all collusions for each pair of paths
def find_all_collusions(lines, r, dimension_length):
    collusion_dict = {}
    for i in range(len(lines) - 1 ):
        for k in range((i+1), len(lines)):
            collusion_pair = (i, k)
            collusions = find_collision_points(
                lines[i][0], lines[i][1], lines[k][0], lines[k][1], r, dimension_length)
            collusion_dict[collusion_pair] = (collusions)
        
    return collusion_dict


# calculate points where circular objects moving along the line collide
def find_collision_points_old(x1, y1, x2, y2, r):
    collision_points_x = []
    collision_points_y = []
    for i in range(len(x1)):
        for k in range(len(x2)):
            # Calculate the distance between the points on each segment
            distance = np.sqrt((x1[i] - x2[k])**2 + (y1[i] - y2[k])**2)

            # If the distance is less than or equal to 2r there is a collusion
            if distance <= 2 * r:

                # Pass the segment position. The assumption here is that every
                # segment is traversed in the same amount of time. Velocity
                # should  be included for more realistic values
                collision_points_x.append(i)
                collision_points_y.append(k)

    return collision_points_x, collision_points_y

# Find all collusions for each pair of paths
def find_all_collusions_old(lines, r):
    collusion_dict = {}
    for i in range(len(lines) - 1 ):
        for k in range((i+1), len(lines)):
            collusion_pair = (i, k)
            collusions_x, collusions_y = find_collision_points_old(
                lines[i][0], lines[i][1], lines[k][0], lines[k][1], r)
            collusion_dict[collusion_pair] = (collusions_x, collusions_y)
        
    return collusion_dict

# Functions for general operations

# Distance between two points in an n-dimentional space
def distance(point1, point2):
    return math.sqrt(sum((p1 - p2) ** 2 for p1, p2 in zip(point1, point2)))

# Given a set of ordered points, compute the sum of all distances betweent them
def get_distance_combination(points):
    total_distance = 0
    for i in range(len(points) - 1):
        cur_distance = distance(points[i], points[i + 1])
        total_distance = total_distance + cur_distance
    return total_distance

# Get a list of distances on every dimension
def get_distances(point1, point2):
    distances = []
    for i in range(len(point1)):
        distances.append(point1[i] - point2[i])
    return distances

# Generate a random point on the n-dimensional space given the constraints
def generate_random_point(space_side_length, num_dimensions):
    coor_list = []
    for _ in range(num_dimensions):
        coor_list.append(random.randint(0, space_side_length))
    return tuple(coor_list)

def generate_around_goal(space_side_length, num_dimensions, goal):
    radius = 6
    vectors = []
    for i in range(num_dimensions):
        vectors.append(goal[i] - random.randint(1, radius))
    
    new_goal = tuple(vectors)

    return new_goal

# Inıtialize n dimensional cubic space
def initialize_n_space(dimension_amount):
    space_dimensions = []
    start_point = [] 
    target_point = []
    for i in range(line_amount):
        space_dimensions.append(num_segments)
        start_point.append(0)
        target_point.append(num_segments)
    
    return space_dimensions, start_point, target_point

class RRT:
    def __init__(self, num_dimensions, space_side_length, collusion_dict):
        
        self.num_dimensions = num_dimensions
        self.space_side_length = space_side_length
        self.collusion_dict = collusion_dict
        
        self.total_elapsed_time = 0
        self.average_node_generation = 0
        
        self.max_distance = space_side_length ** num_dimensions + 1
        
        self.space_dimensions, self.start_point, self.target_point = \
            initialize_n_space(num_dimensions)
            
        self.node_list = [['q0', self.start_point, 'None']] 
        
        self.line_segments = []
        
        self.path_nodes, self.path_segments = [], []
        
        self.longest_node_generated = None
        self.longest_gen_time = 0
        
        self.max_nodes = 10000
        
        self.generate_tree()
        
        self.find_final_path()
        
        
    def generate_tree(self):
        total_start_time = time.time()
        total_node_gen_time = 0
        done = False
        
        # Count successful nodes
        point_count = 0
        while not done:
            
            start_time = time.time()
            point, colluded_points = self.generate_node(point_count)
            end_time = time.time()
            elapsed_time = end_time - start_time
            total_node_gen_time = total_node_gen_time + elapsed_time
            
            if elapsed_time > self.longest_gen_time:
                self.longest_gen_time = elapsed_time
                self.longest_node_generated = point
            
            point_count = point_count + 1
            
            if self.is_done(self.node_list[-1][1]) or self.max_nodes < point_count or self.total_elapsed_time > 300:
                done = True
                
        total_end_time = time.time()
        total_elapsed_time = end_time - start_time
        self.total_elapsed_time = total_elapsed_time
        self.average_node_generation = total_node_gen_time / point_count
        
        self.path_segments.append([self.node_list[-1][1], self.target_point])

        self.node_list.append(["q{}".format(len(self.node_list)), self.target_point, self.node_list[-1][0]])
        
    def is_done(self, point):
        if distance(self.target_point, point) < 10:
            return True
        return False
        
    def generate_node(self, point_count):

        point_ok = False
        node_name = "q{}".format(len(self.node_list))
        iteration = 0
        colluded_points = []
        while not point_ok:
            
            # Generate random coordinates
            if point_count % 4 == 2 and iteration < 5:
                point = generate_around_goal(self.space_side_length - 1, self.num_dimensions, self.target_point)
                
            else:    
                point = generate_random_point(self.space_side_length - 1, self.num_dimensions)
                
            print(point_count)

            # Find parent node
            parent = self.node_list[self.nearest_node(point)]

            # Get a list of distances on every dimension
            d_list = get_distances(point, parent[1])

            # Nagnitude of vector to closest node
            vec_mag = distance(point, parent[1])
            
            random_vector = random.randint(1, 10)
            
            if vec_mag == 0:
                continue

            # Get new node coordinates by adding unit vector components to parent coordinates
            points_on_the_way = []
            for n in range(20):
                temp_point = []
                for i in range(len(d_list)):
                    temp_point.append(min(self.space_side_length - 1, round(parent[1][i] + n * (d_list[i] / vec_mag))))
                points_on_the_way.append(temp_point)
                
            new_point = tuple(points_on_the_way[-1])
            
            is_invalid = False
            for i in new_point:
                if i < 0:
                    is_invalid = True
                    break
            if is_invalid:
                continue

            # If newly created node
            if not self.is_valid(points_on_the_way):
                colluded_points.append(new_point)
            else:
                point_ok = True
                
            iteration = iteration + 1
        
        self.node_list.append([node_name, new_point, parent[0]])
        self.line_segments.append([parent[1], new_point])
       
        
        
        return new_point, colluded_points

    def nearest_node(self, point):
        least_distance = self.max_distance
        least_node = 0
        # After the loop ends, keep the plot open until it's closed by the user     
        for i in range(len(self.node_list)):
            cur_dist = distance(point, self.node_list[i][1])
            if least_distance > cur_dist:
                least_distance = cur_dist
                least_node = i
        return least_node

    def is_valid(self, points):
        for i in self.collusion_dict:
            coordinates = (i[0],i[1])
            collusions = self.collusion_dict[i]
            
            for point in points: 
                point_to_check = (point[coordinates[0]], point[coordinates[1]])
            
                if collusions[point_to_check] == True:
                    return False
        
        return True
    
    def execute_bias_strategy(self, strategy):
        match strategy:
            case "CA":
                return self.complete_any()
            case "RJ":
                return self.resolve_junction()
            case "RC":
                return self.resolve_convoy()

    def find_final_path(self):
        cur_node = self.node_list[-1]
        self.path_nodes.append(cur_node)
        for i in range(len(self.node_list)):
            if self.node_list[-i-1][0] == cur_node[2]:
                self.path_nodes.insert(0, self.node_list[-i-1])
                self.path_segments.insert(0, (self.node_list[-i-1][1], cur_node[1]))
                cur_node = self.node_list[-i-1]
                
    def get_final_path(self):
        points = []
        for i in self.path_nodes:
            points.append(i[1])
        return points
    
    def show_longest_generation(self):
        print("Longest node generation was on the node:")
        print(self.longest_node_generated)
        print("And it took:")
        print(self.longest_gen_time)
        print("Average node generation time was:")
        print(self.average_node_generation)


# Line properties
x_bound = 400 # max x value of any point on line
y_bound = 400 # max y value of any point on line

min_num_stops = 3 # min number of times slope can be shifted
max_num_stops = 4 # max number of times slope can be shifted

length_min = 30 # min length of a line segment
length_max = 40 # max length of a line segment

num_segments = 360 # segment amount a line is seperated to

line_amount = 5 # amount of lines = robot amount iteration = iteration + 1

# Line initialization (Every line represents a robot's path)
lines = []

for i in range(line_amount):
    x, y = generate_curve(length_min, length_max, num_segments, min_num_stops, max_num_stops, x_bound, y_bound)
    lines.append((x,y))

# Robot variables
radius = 10 # Radius of a circular robot
num_robots = line_amount

# Fİnd all collusion pairs
all_collusions = find_all_collusions(lines, radius, num_segments)
display_collusions = find_all_collusions_old(lines, radius)

# Segment amount is identical to travel time for simplicity
total_travel_time_1 = num_segments
total_travel_time_2 = num_segments

# Visualize lines(paths)
plt.figure()
for i in range(len(lines)):
    # Plot the paths with round markers at the beginning of lines representing robots
    circle_x, circle_y = points_around_point(lines[i][0][0], lines[i][1][0], radius)
    plt.plot(lines[i][0], lines[i][1], '-o', markersize=1)
    plt.plot(lines[i][0][-1], lines[i][1][-1], 'x', markersize=15)
    plt.xlim(0, x_bound)
    plt.ylim(0, y_bound)
    plt.text(lines[i][0][0], lines[i][1][0], "path " + str(i + 1))
    plt.scatter(circle_x, circle_y, c = "gray")
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Paths')
plt.show(block = False)



start_time = time.time()
schedule_rrt = RRT(num_robots, num_segments, all_collusions)
end_time = time.time()
elapsed_time = end_time - start_time
print(f"Elapsed time while calculating schedule: {elapsed_time}")

schedule = schedule_rrt.get_final_path()

for s in schedule:
    print(s)

best_distance = distance(schedule_rrt.start_point, schedule_rrt.target_point)
obtained_distance = get_distance_combination(schedule)
distance_ratio = obtained_distance / best_distance
print (f"Best possible distance without collusions: {best_distance}")
print (f"Distance obtained by RRT: {obtained_distance}, ratio between best and obtained: {distance_ratio}")
schedule_rrt.show_longest_generation()

schedule_pairs_x = {}
schedule_pairs_y = {}
for i in range(len(lines) - 1 ):
    for k in range((i+1), len(lines)):
        current_pair_schedule_x = []
        current_pair_schedule_y = []
        for s in schedule:
            current_pair_schedule_x.append(s[i])
            current_pair_schedule_y.append(s[k])
        schedule_pairs_x[(i,k)] = current_pair_schedule_x
        schedule_pairs_y[(i,k)] = current_pair_schedule_y


# Print each path pair with collusions
fig = plt.figure()
subplot_row = num_robots - 1
subplot_column = num_robots - 1

# Counter for the subplot positions
subplot_count = 1

# Print each path pair with collisions
for i in range(len(lines) - 1):
    subplot_count = subplot_count + i
    for k in range((i + 1), len(lines)):
        current_pair = (lines[i], lines[k])
        current_collusion = display_collusions[(i, k)]

        plt.subplot(subplot_row, subplot_column, subplot_count)
        plt.plot(current_collusion[1], current_collusion[0], ',', markersize=0.5, c="r")
        plt.xlabel('Path ' + str(k + 1))
        plt.ylabel('Path ' + str(i + 1))
        plt.xlim(0, total_travel_time_1)
        plt.ylim(0, total_travel_time_2)

        plt.plot(schedule_pairs_y[(i,k)], schedule_pairs_x[(i,k)], 'o', markersize=0.5, c="g")

        # Increment the subplot count for the next pair
        subplot_count += 1

# Adjust the layout to prevent overlapping of subplots
plt.tight_layout()

# Display all subplots together
plt.show(block=False)

fig = plt.figure()
subplot_row = num_robots - 1
subplot_column = num_robots - 1

# Counter for the subplot positions
subplot_count = 1

# Print each path pair with collisions
for i in range(len(lines) - 1):
    subplot_count = subplot_count + i
    for k in range((i + 1), len(lines)):
        current_pair = (lines[i], lines[k])
        current_collusion = display_collusions[(i, k)]

        # Create the first subplot
        plt.subplot(subplot_row, subplot_column, subplot_count)
        plt.xlim(0, x_bound)
        plt.ylim(0, y_bound)

        plt.xlabel('X (Lines: ' + str(i + 1) + ', ' + str(k + 1))
        plt.ylabel('Y')

        circle_x, circle_y = points_around_point(current_pair[0][0][0], current_pair[0][1][0], radius)
        plt.plot(current_pair[0][0], current_pair[0][1], '-o', markersize=1)
        plt.scatter(circle_x, circle_y, c="gray")
        plt.text(current_pair[0][0][0], current_pair[0][1][0], "path " + str(i + 1))
        plt.scatter(current_pair[0][0][current_collusion[0]],
                    current_pair[0][1][current_collusion[0]], c="r")

        circle_x, circle_y = points_around_point(current_pair[1][0][0], current_pair[1][1][0], radius)
        plt.plot(current_pair[1][0], current_pair[1][1], '-o', markersize=1)
        plt.scatter(circle_x, circle_y, c="gray")
        plt.text(current_pair[1][0][0], current_pair[1][1][0], "path " + str(k + 1))
        plt.scatter(current_pair[1][0][current_collusion[1]],
                    current_pair[1][1][current_collusion[1]], c="y")

        subplot_count += 1

# Adjust the layout to prevent overlapping of subplots
plt.tight_layout()

# Display all subplots together
plt.show(block=False)
