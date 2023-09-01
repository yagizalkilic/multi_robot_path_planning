#ifndef PROJECT_UTILITIES_H
#define PROJECT_UTILITIES_H

#include <functional>
#include <string>
#include <vector>
#include <utility>
#include <random>
#include <iostream>
#include <fstream> 
#include <sstream>
#include <cmath>
#include <unordered_map>
#include <stdexcept> 
#include <algorithm>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <limits.h>
#include <utility>
#include <set>
#include <boost/math/common_factor.hpp>

struct pair_hash 
{
    template <class T1, class T2>
    const std::size_t operator () (const std::pair<T1, T2>& p) const 
    {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        return h1 ^ h2;
    }
};

struct Point 
{
    int dimension = 2; // Set the dimension as 2 for default
    std::vector<double> coordinates;
};

struct Node 
{
    std::string name;
    Point point;
    std::string parent;
    double cost = 0;
};


double calculate_distance(const Point& p1, const Point& p2);
int random_segment_length(int min_length, int max_length);
double get_random_double(double min_val, double max_val);
Point generate_random_point(std::vector<int> space_side_length, int num_dimensions);
Point generate_around_goal(std::vector<int> space_side_length, int radius, Point goal);
std::vector<Point> points_around_point(double x, double y, double r);
std::vector<int> get_distances(Point point1, Point point2);
std::vector<int> get_unique_numbers(std::vector<int> num_list, int start_index, int end_index);
double calculate_total_distance(std::vector<Point> physical_path, int first_index, int last_index);
double calculate_orientation(Point point1, Point point2);
std::vector<int> get_unique_neighbours(std::vector<int> num_list, int start_index, int end_index);
std::vector<int> exclude_number(std::vector<int> num_list, int number, int start_index, int end_index);
double calculate_angular_velocity(double orientation_begin, double orientation_end, double duration);
Point generate_point_between(const Point p1, const Point p2, double distance);
double calculate_max_speed_before_deacc(double max_acc, double distance);
std::pair<double, double> calculate_distance_during_acc(double start_vel, double end_vel, double acc, double dt);

#endif // PROJECT_UTILITIES_H
