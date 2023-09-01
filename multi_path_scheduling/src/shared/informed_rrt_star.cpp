#include "../../include/informed_rrt_star.h"

InformedRRTStar::InformedRRTStar(int num_dimensions, std::vector<int> space_side_length, 
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
void InformedRRTStar::generate_tree() 
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
    final_node.cost = node_list.back().cost + calculate_distance(node_list.back().point, target_point);

    node_list.push_back(final_node);
    std::cout << std::endl;;
}

/**
 * Determines whether the path is completed or not.
 *
 * @param point Current point to compare with the target
 * @return true if the distance is less than the predetermined amount, false otherwise
 */
bool InformedRRTStar::is_done(const Point& point) const 
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
bool InformedRRTStar::generate_node(int point_count) 
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
        std::cout << point_count << ", ";

        // Find the nearest node to the randomly generated node
        parent = node_list[nearest_node(point)];

        // If the randomly generated point is on the parent point discard the point.
        if (parent.point.coordinates == point.coordinates) 
        {
            continue;
        }

        // Get the difference between current point and parrent points every coordinate entry.
        auto d_list = get_distances(point, parent.point);

        // Generate the random vector.
        double vec_mag = calculate_distance(point, parent.point);
        int random_vector = std::rand() % 10 + 11;

        // Check if the generated points are out of bounds.
        bool is_out_of_bounds = false;

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
    next_node.cost = parent.cost + calculate_distance(parent.point, new_point);


    node_list.push_back(next_node);

    // Rewire the tree (RRT* extension)
    rewire_tree(next_node);

    return true;
}

/**
 * Rewires the tree by checking if any existing nodes in the tree can be reached more efficiently
 * by the newly added node. If so, the parent of those nodes is updated to the newly added node.
 *
 * @param new_node The newly added node in the tree.
 */
void InformedRRTStar::rewire_tree(Node& new_node)
{
    // Find all nodes within a certain radius from the new_node
    std::vector<int> nearby_nodes = find_k_nearest_nodes(new_node, 30);

    // Update the parent of each nearby node if it improves the cost
    for (int node_index : nearby_nodes)
    {
        Node& nearby_node = node_list[node_index];
        // Calculate the cost to reach the nearby_node via the new_node
        double dist = calculate_distance(nearby_node.point, new_node.point);
        double cost_via_new_node = new_node.cost + dist;

        // Update the parent if the new path is shorter
        if (cost_via_new_node < nearby_node.cost && new_node.parent != nearby_node.name 
             && new_node.name != nearby_node.name && is_valid_path(nearby_node.point, new_node.point) )
        {
            std::cout << "rewired" << " -- ";
            nearby_node.parent = new_node.name;
            nearby_node.cost = cost_via_new_node;
        }
    }
}

/**
 * Find the nearest node to the given point. Look at euclidian distance.
 *
 * @param point Point to find the nearest node around
 * @return least_node Node that has the least distance from the point
 */
int InformedRRTStar::nearest_node(const Point& point) const 
{
    double least_distance = INT_MAX;
    int least_node = 0;

    for (int i = 0; i < node_list.size(); ++i) 
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
bool InformedRRTStar::is_valid(std::vector<Point> points)
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
 * Check if the path between two points is collision-free (no obstacles in the way).
 * We assume that the points are within bounds (valid points in the search space).
 *
 * @param point1 The starting point of the path.
 * @param point2 The ending point of the path.
 * @return true if the path is collision-free, false otherwise.
 */
bool InformedRRTStar::is_valid_path(const Point& point1, const Point& point2) 
{
    // Calculate the direction vector between the two points
    std::vector<int> direction;
    direction = get_distances(point2, point1);

    // Calculate the number of steps to divide the path into
    int num_steps = 20; // You can adjust this value for finer collision checking

    // Check for collisions at each step along the path
    for (int step = 0; step <= num_steps; ++step)
    {
        // Calculate the intermediate point along the path
        Point intermediate_point;
        intermediate_point.dimension = point1.dimension;
        for (size_t i = 0; i < point1.coordinates.size(); ++i)
        {
            int coord = (int)(point1.coordinates[i] + (step * direction[i] / num_steps));
            intermediate_point.coordinates.push_back(coord);
        }
        std::vector<Point> single_point;
        single_point.push_back(intermediate_point);

        // Check if the intermediate point is in collision with any obstacles
        if (!is_valid(single_point))
        {
            return false; // Collision detected
        }
    }

    // No collisions detected along the path
    return true;
}

/**
 * Generates the path from the final node to the first node.
 */
void InformedRRTStar::find_final_path() 
{
    auto cur_node = node_list.back();
    path_nodes.push_back(cur_node);

    // Iterate through all nodes in reverse order.
    for (int i = static_cast<int>(node_list.size()) - 1; i >= 0; --i)
    {
        if (node_list[i].name == cur_node.parent) 
        {
            path_nodes.insert(path_nodes.begin(), node_list[i]);
            cur_node = node_list[i];
            i = static_cast<int>(node_list.size()) - 1;
            continue;
        }
    }
}

/**
 * Find all nodes that are within a given radius from a given node in the tree.
 *
 * @param node The node whose neighbors need to be found.
 * @param radius The maximum distance within which to consider nodes.
 * @return A vector of indices of nodes that fall within the given radius from the node in the node_list.
 */
std::vector<int> InformedRRTStar::find_nodes_within_radius(const Node& node, double radius) const
{
    std::vector<int> nodes_within_radius;

    for (size_t i = 0; i < node_list.size(); ++i)
    {
        if (node_list[i].name == node.name)
            continue;

        double distance = calculate_distance(node_list[i].point, node.point);
        if (distance <= radius)
        {
            nodes_within_radius.push_back(i);
        }
    }

    return nodes_within_radius;
}

/**
 * Print time tracking parameters.
 */
void InformedRRTStar::show_longest_generation() const 
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
void InformedRRTStar::initialize_n_space()
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

/**
 * Find the k nearest nodes to a given node in the tree.
 *
 * @param node The node whose neighbors need to be found.
 * @param k The number of nearest nodes to find.
 * @return A vector of indices of the k nearest nodes in the node_list.
 */
std::vector<int> InformedRRTStar::find_k_nearest_nodes(const Node& node, int k) const
{
    std::vector<std::pair<int, double>> nearest_nodes;

    for (size_t i = 0; i < node_list.size(); ++i)
    {
        if (node_list[i].name == node.name)
            continue;

        double distance = calculate_distance(node_list[i].point, node.point);
        nearest_nodes.emplace_back(i, distance);
    }

    std::sort(nearest_nodes.begin(), nearest_nodes.end(), [](const auto& a, const auto& b) {
        return a.second < b.second;
    });

    std::vector<int> result;
    for (int i = 0; i < k && i < nearest_nodes.size(); ++i)
    {
        result.push_back(nearest_nodes[i].first);
    }

    return result;
}

    std::vector<std::vector<double>> transform(double value) {
        // Define your n-dimensional transformation logic here
        std::vector<std::vector<double>> transformationMatrix;
        // Fill in the transformationMatrix based on your logic
        return transformationMatrix;
    }

void InformedRRTStar::drawEllipse(int dimensions) 
{
    double pi = 3.14159265358979323846;
    std::vector<double> t;
    for (double it = 0; it <= 2 * pi + 0.1; it += 0.1) {
        t.push_back(it);
    }

    std::vector<std::vector<int>> coordinates(dimensions);
    for (int i = 0; i < dimensions; ++i) {
        coordinates[i].resize(t.size());
    }

    std::vector<std::vector<double>> transformMatrix = transform(c_best / 2); // You'll need to define the transform method and provide c_best
}
