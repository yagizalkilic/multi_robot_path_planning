import random
import numpy as np
import matplotlib.pyplot as plt

def generate_curve(length_min, length_max, num_segments):
    # Generate random starting points for the curve
    start_x = random.uniform(0, 100)
    start_y = random.uniform(0, 100)

    # Generate random lengths for the curve within the given range
    length = random.uniform(length_min, length_max)

    # Generate random coefficients for the quadratic curve
    a = random.uniform(-1, 1)
    b = random.uniform(-1, 1)
    s = random.choice([-1,1])


    # Generate points for the curve segments
    x = np.linspace(start_x, start_x + length, num=num_segments)
    y = a * x**2 + b * x + start_y

    return x, y

def find_collision_points(x1, y1, x2, y2, r, v1, v2):
    # Find all collision points between the two curves
    collision_points_x = []
    collision_points_y = []
    for i in range(len(x1)):
        for k in range(len(x2)):
            # Calculate the distance between the points on each curve
            distance = np.sqrt((x1[i] - x2[k])**2 + (y1[i] - y2[k])**2)
    
            # Check if the distance is less than or equal to the sum of the radii
            if distance <= 2 * r:
                # Calculate the time passed on each line when the collision occurs
                t1 = i
                t2 = k
                collision_points_x.append(t1)
                collision_points_y.append(t2)
                

    return collision_points_x, collision_points_y

# Define the minimum and maximum lengths, the number of segments, and the radius of the objects
length_min = 100
length_max = 150
num_segments = 1000
radius = 30
v1 = 1
v2 = 1

# Generate the first curve
x1, y1 = generate_curve(length_min, length_max, num_segments)

# Generate the second curve
x2, y2 = generate_curve(length_min, length_max, num_segments)

print(len(x1))

# Calculate the collision points
collision_points_x, collision_points_y = find_collision_points(x1, y1, x2, y2, radius, v1, v2)

# Plot the first curve with round markers at the beginning of lines
plt.figure()
plt.plot(x1, y1, '-o', markersize=1)
plt.text(x1[0], y1[0], "line 1")
plt.plot(x1[0], y1[0], '-o', markersize=30)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Routes')

plt.plot(x2, y2, '-o', markersize=1)
plt.text(x2[0], y2[0], "line 2")
plt.plot(x2[0], y2[0], '-o', markersize=30)

total_travel_time_1 = 1000
total_travel_time_2 = 1000

print(collision_points_x)
print(collision_points_y)

# Plot the collision points
plt.figure()
plt.plot(collision_points_x, collision_points_y, 'ro')
plt.xlabel('Time 1')
plt.ylabel('Time 2')
plt.xlim(0, total_travel_time_1)
plt.ylim(0, total_travel_time_2)
plt.title('Collision Points')

plt.show()
