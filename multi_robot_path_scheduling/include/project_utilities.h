#ifndef PROJECT_UTILITIES_H
#define PROJECT_UTILITIES_H

#include <functional>
#include <string>
#include <vector>
#include <utility>
#include <random>
#include <iostream>
#include <cmath>
#include <unordered_map>
#include <stdexcept> 
#include <algorithm>

struct pair_hash 
{
    template <class T1, class T2>
    const std::size_t operator () (const std::pair<T1, T2>& p) const {
        auto h1 = std::hash<T1>{}(p.first);
        auto h2 = std::hash<T2>{}(p.second);

        return h1 ^ h2;
    }
};

struct Point 
{
    int dimension = 2; // Set the dimension as 2 for default
    std::vector<int> coordinates;
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
Point generate_random_point(int space_side_length, int num_dimensions);
Point generate_around_goal(int space_side_length, int radius, Point goal);
std::vector<Point> points_around_point(double x, double y, double r);
std::vector<int> get_distances(Point point1, Point point2);

#endif // PROJECT_UTILITIES_H
