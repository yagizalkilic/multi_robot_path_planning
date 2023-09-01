#include "../../include/rrt.h"

RRT::RRT(int num_dimensions, std::vector<int> space_side_length, 
         std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash> dimension_pair_correspondence)
{
    this->num_dimensions = num_dimensions;
    this->space_side_length = space_side_length;
    this->dimension_pair_correspondence = dimension_pair_correspondence;

    // Initialize start_point, target_point, first_node
    initialize_n_space();

    // To determine rrt runtime
    this->max_nodes = 30000;
    this->max_seconds = 300;

    // Time tracking variables
    this->total_elapsed_time = std::chrono::duration<double>::zero();
    this->average_node_generation = std::chrono::duration<double>::zero();
    this->longest_gen_time = std::chrono::duration<double>::zero();

    // Spread specification for generating around the goal
    this->radius = 20;

    // Least distance required between the last two nodes.
    this->final_distance = 10;

    generate_tree();
    find_final_path();

}

/**
 * Generates an RRT. At each step generates a node, calculates generation times.
 * Exits if the target_point is found or the max_nodes is exceeded.
 */
void RRT::generate_tree() 
{
    // Time tracking parameters
    auto total_start_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> total_node_gen_time = std::chrono::duration<double>::zero();

    bool done = false;
    bool node_generation_possible = false;
    int point_count = 0;

    while (!done) 
    {
        auto start_time = std::chrono::steady_clock::now();

        // Generate new node
        node_generation_possible = generate_node(point_count);
        point_count++;

        auto end_time = std::chrono::steady_clock::now();

        std::chrono::duration<double> elapsed_time = end_time - start_time;
        total_node_gen_time += elapsed_time;

        if (elapsed_time > longest_gen_time) 
        {
            longest_gen_time = elapsed_time;
        }

        // We are done if the path is complete, if the max amount of nodes is traversed, if a certain
        // amount of time has passed since the start of tree generation, or if there is no new
        // node that can be successfully generated.
        if (is_done(node_list.back().point) || max_nodes < point_count || 
            total_elapsed_time.count() > max_seconds || !node_generation_possible) 
        {
            done = true;
        }
    }

    auto total_end_time = std::chrono::steady_clock::now();
    total_elapsed_time = total_end_time - total_start_time;
    average_node_generation = total_node_gen_time / point_count;

    // The final node is always: (space_side_length, ..., space_side_length)
    // This is true even if the path is incomplete.
    Node final_node;
    final_node.name = "q" + std::to_string(node_list.size());
    final_node.point = target_point;
    final_node.parent = node_list.back().name;

    node_list.push_back(final_node);
}

/**
 * Determines whether the path is completed or not.
 *
 * @param point Current point to compare with the target
 * @return true if the distance is less than the predetermined amount, false otherwise
 */
bool RRT::is_done(const Point& point) const 
{
    return calculate_distance(target_point, point) < final_distance;
}

/**
 * Generates the next_node in the RRT. Selects a random point(selects a random point around
 * the target 25% of the time), determines the closest node to that point called parent node. 
 * Determines a random vector from the parent node in the direction of the random point. Checks
 * for collisions on the vector. IF there is none pushes the new node to the tree.
 *
 * @param point_count Current count of node amount
 * @return true if generation is possible, false otherwise
 */
bool RRT::generate_node(int point_count) 
{
    bool point_ok = false;
    int iteration = 0;

    std::string node_name = "q" + std::to_string(node_list.size());

    Point new_point;
    Node parent;

    while (!point_ok) 
    {
        // If no node was generated for the determined amount of time return false.
        if ( iteration > 500 )
        {
            return false;
        }

        Point point;
        // Generate around target 25% of the time
        if (point_count % 4 == 2 && iteration < 5) 
        {
            point = generate_around_goal(space_side_length, radius, target_point);
        } 
        else 
        {
            point = generate_random_point(space_side_length, num_dimensions);
        }

        // Print current node count to the terminal for debugging.
        std::cout << point_count << std::endl;

        // Find the nearest node to the randomly generated node
        parent = node_list[nearest_node(point)];

        // If the randomly generated point is on the parent point discard the point.
        if (parent.point.coordinates == point.coordinates) 
        {
            continue;
        }

        // Get the difference between current point's and parrent point's every coordinate entry.
        auto d_list = get_distances(point, parent.point);

        // Generate the random vector.
        double vec_mag = calculate_distance(point, parent.point);
        int random_vector = std::rand() % 15 + 6;

        // Check if the generated points are out of bounds.
        bool is_out_of_bounds = false;

        if (vec_mag < 1)
        {
            continue;
        }

        // Generate points between vector and parent.
        std::vector<Point> points_on_the_way;
        for (int n = 0; n < random_vector; ++n) 
        {
            Point temp_point;
            temp_point.dimension = d_list.size();
            for (size_t i = 0; i < d_list.size(); ++i) 
            {
                int temp_axis = std::min(space_side_length[i] - 1, (int)(parent.point.coordinates[i] + n * (d_list[i] / vec_mag)));
                if (temp_axis < 0 || temp_axis > space_side_length[i])
                {
                    is_out_of_bounds = true;
                    break;
                }
                temp_point.coordinates.push_back(temp_axis);
            }
            if (is_out_of_bounds)
            {
                break;
            }
            points_on_the_way.push_back(temp_point);
        }
        if (is_out_of_bounds)
        {
            continue;
        }

        // Last generated point is the point of the new_node
        new_point = points_on_the_way.back();

        // Check if points between parent and new are restricted.
        if (is_valid(points_on_the_way)) 
        {
            point_ok = true;
        }

        // Tracks node generation amount, is necessary if every single node
        // Generated is invalid.
        iteration++;
    }

    // Generate and push the new node
    Node next_node;
    next_node.name = node_name;
    next_node.point.coordinates = new_point.coordinates;
    next_node.point.dimension = new_point.dimension;
    next_node.parent = parent.name;

    node_list.push_back(next_node);

    return true;
}

/**
 * Find the nearest node to the given point. Look at euclidian distance.
 *
 * @param point Point to find the nearest node around
 * @return least_node Node that has the least distance from the point
 */
int RRT::nearest_node(const Point& point) const 
{
    double least_distance = INT_MAX;
    int least_node = 0;

    for (size_t i = 0; i < node_list.size(); ++i) 
    {
        double cur_dist = calculate_distance(point, node_list[i].point);
        if (least_distance > cur_dist) 
        {
            least_distance = cur_dist;
            least_node = i;
        }
    }

    return least_node;
}

/**
 * Check if any given key corresponds to a restricted point.
 *
 * @param points List of points to check
 * @return true if there is no collision, false otherwise
 */
bool RRT::is_valid(std::vector<Point> points)
{
    for (const auto& entry : dimension_pair_correspondence) 
    {
        const auto& key = entry.first;  // This will give you a pair of integers
        const auto& inner_map = entry.second;  // This will give you the inner unordered map

        auto x_coord = key.first;
        auto y_coord = key.second;

        for (Point point : points) 
        {
            std::pair<int, int> cur_point = std::make_pair(point.coordinates[x_coord], point.coordinates[y_coord]);
            auto value = inner_map.find(cur_point);

            if (value->second) 
            {
                return false;
            }
        }
    }

    return true;
}

/**
 * Generates the path from the final node to the first node.
 */
void RRT::find_final_path() 
{
    auto cur_node = node_list.back();
    path_nodes.push_back(cur_node);

    // Iterate through all nodes in reverse order.
    for (int i = static_cast<int>(node_list.size()); i >= 0; --i) 
    {
        if (node_list[i].name == cur_node.parent) 
        {
            path_nodes.insert(path_nodes.begin(), node_list[i]);
            cur_node = node_list[i];
        }
    }
}

/**
 * Print time tracking parameters.
 */
void RRT::show_longest_generation() const 
{
    // Convert durations to seconds for easier display
    double longest_gen_time_seconds = longest_gen_time.count();
    double average_node_gen_seconds = average_node_generation.count();
    double total_elapsed_seconds = total_elapsed_time.count();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "The longest node generation time was: " << longest_gen_time_seconds << " seconds" << std::endl;
    std::cout << "Average node generation time was: " << average_node_gen_seconds << " seconds" << std::endl;
    std::cout << "Total elapsed time during RRT generation was: " << total_elapsed_seconds << " seconds" << std::endl;
}

/**
 * Initialize the space required by the RRT.
 */
void RRT::initialize_n_space()
{
    start_point.dimension = num_dimensions;
    start_point.coordinates.resize(num_dimensions, 0);

    target_point.dimension = space_side_length.size();
    for ( int i = 0; i < space_side_length.size(); i++ )
    {
        target_point.coordinates.push_back(space_side_length[i] - 1);
    }

    Node first_node;
    first_node.name = "q0";
    first_node.point = start_point;
    first_node.parent = "None";

    node_list.push_back(first_node);
}