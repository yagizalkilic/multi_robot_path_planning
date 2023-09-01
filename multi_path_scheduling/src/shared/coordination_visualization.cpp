#include "../../include/coordination_visualization.h"

CoordinationVisualization::CoordinationVisualization(int space_boundary_x, int space_boundary_y, std::vector<int> time_boundary)
{
	this->space_boundary_x = space_boundary_x;
    this->space_boundary_y = space_boundary_y;
    this->time_boundary = time_boundary;
}

/**
 * Draws the physical space where collisions occur. Displays the paths that 
 * AGVs take as well as the possible collisions that can occur on the paths.
 * 
 * @param all_paths physical representation of the paths
 * @param all_collisions_space collision coordinates
 * @param AGV_radius radius of an AGV
 * @param canvas_name
 */
void CoordinationVisualization::draw_space(std::vector<PhysicalPath>& all_paths,
	std::vector<std::pair<int, int>> all_collisions_space, int AGV_radius, char* canvas_name)
{
    std::cout << "Drawing collision space..." << std::endl;

    TCanvas* space_canvas = new TCanvas(canvas_name, canvas_name, 800, 800);
    space_canvas->SetGrid();
    space_canvas->cd();
    TMultiGraph *mg = new TMultiGraph();

    mg->GetXaxis()->SetLimits(0, space_boundary_x); 
    mg->GetYaxis()->SetRangeUser(0, space_boundary_y); 

    // Colors to be used on graphs

    const int numColors = all_paths.size();
    int colors[numColors] = {kBlue, kGreen, kOrange, kMagenta, kGray, kPink, kAzure};

    // Generate lines and AGVs
    for (int idx = 0; idx < all_paths.size(); idx++)
    {
        const auto &path = all_paths[idx].get_final_physical_path_points();
        Int_t n = path.size();
        Double_t x[n], y[n];
        // Build the arrays with the coordinates of points
        for (Int_t i = 0; i < n; i++)
        {
            x[i] = path[i].coordinates[0];
            y[i] = path[i].coordinates[1];
        }

        // Create representations of AGVs
        TGraph *ellipseCenter = new TGraph(1, &x[0], &y[0]);
        ellipseCenter->SetMarkerStyle(24);
        ellipseCenter->SetMarkerSize(AGV_radius * 2 * space_boundary_x / 200);
        ellipseCenter->SetMarkerColor(colors[idx % numColors]);
        mg->Add(ellipseCenter, "P");

        // Add text at the position of the AGV center
        TString textLabel = TString::Format("AGV %d", (idx + 1)); 
        TText *text = new TText(x[0], y[0], textLabel);
        text->SetTextSize(0.02);
        text->SetTextAlign(22); 
        text->SetTextColor(colors[idx % numColors]);

        // Create graph for each path
        TGraph *gr3 = new TGraph(n, x, y);
        gr3->SetMarkerStyle(1);
        gr3->SetLineColor(colors[idx % numColors]);
        gr3->SetMarkerColor(colors[idx % numColors]);

        gr3->GetListOfFunctions()->Add(text);

        // Add the graph to the TMultiGraph
        mg->Add(gr3, "PL");
    }

    // Add collisions to the graph
    std::vector<int> collision_x;
    std::vector<int> collision_y;

    for( auto const &coordinate: all_collisions_space )
    {
        collision_x.push_back(coordinate.first);
        collision_y.push_back(coordinate.second);
    }

    int* collision_x_array = collision_x.data();
    int* collision_y_array = collision_y.data();

    TGraph *collisions = new TGraph(collision_x.size(), collision_x_array, collision_y_array);
    collisions->SetMarkerStyle(20);
    collisions->SetMarkerSize(0.5);
    collisions->SetLineColor(kRed);
    collisions->SetMarkerColor(kRed);
    collisions->SetMarkerColorAlpha(kRed, 0.35);

    // Add collisions to the TMultiGraph
    mg->Add(collisions, "P"); // Remove "A" to exclude line connecting points

    // Draw all graphs together, first canvas is done.
    mg->Draw("APL");

    space_canvas->GetFrame()->SetBorderSize(12);
    space_canvas->Modified();
    space_canvas->Update();

}

/**
 * For each path pair displays the collisions that can occur in the time space and draws
 * the solution that the RRT provides.
 * 
 * @param AGV_amount amount of AGVs
 * @param all_nodes nodes that represent the found path by RRT
 * @param all_collisions_time collisions that occur in time space between each pair of AGVs
 * @param canvas_name
 */
void CoordinationVisualization::draw_time(int AGV_amount, std::vector<Node> path, std::vector<Node> all_nodes,
	std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash>& all_collisions_time, char* canvas_name)
{
	std::cout << "Drawing time space..." << std::endl;

	TCanvas* time_canvas = new TCanvas(canvas_name, canvas_name, 300 * AGV_amount, 220 * AGV_amount);

    // Divide the canvas into segments that represent path pair
    time_canvas->Divide(AGV_amount - 1, AGV_amount - 1, 0.01, 0.01);

    int pad_amount = (AGV_amount - 1) * (AGV_amount - 1);

    int cd_count = 1;

    // For each segment draw a graph if there is a relevant pair, leave it empty if not
    for (int i = 0; i < AGV_amount; i++) 
    {
        for (int a = 0; a < AGV_amount; a++) 
        {
            auto key = all_collisions_time.find(std::make_pair(i,a));

            // If the pair exists in relevant time pairs
            if (key != all_collisions_time.end()) 
            {
                // Create the multi graph for time path and collisions
                TMultiGraph *mg = new TMultiGraph();

                mg->GetXaxis()->SetLimits(0, time_boundary[a]); 
                mg->GetYaxis()->SetRangeUser(0, time_boundary[i]); 

                std::string pad_name = "Collision Zone: (" + std::to_string(a+1) + ", " + std::to_string(i+1) + ")";
                const char* title = pad_name.c_str(); // Convert to const char*


                const auto &collision_zone = key->second;

                // Create vectors to hold the coordinates of colliding points
                std::vector<int> collision_x;
                std::vector<int> collision_y;

                // Iterate through the collisionMap to get the colliding points
                for (auto coordEntry = collision_zone.begin(); coordEntry != collision_zone.end(); ++coordEntry)
                {
                    if (coordEntry->second)
                    {
                        collision_x.push_back(coordEntry->first.second);
                        collision_y.push_back(coordEntry->first.first);
                    }
                }
                bool line_color = true;

                if ( collision_x.size() == 0 )
                {
                    collision_x.push_back(0);
                    collision_y.push_back(0);
                    line_color = false;
                }

                // Create graph for the colliding points in this division
                TGraph *collisionGraph = new TGraph(collision_x.size(), collision_x.data(), collision_y.data());
                collisionGraph->GetXaxis()->SetLimits(0, time_boundary[a]); 
                collisionGraph->GetYaxis()->SetRangeUser(0, time_boundary[i]); 
                collisionGraph->SetMarkerStyle(20);
                collisionGraph->SetMarkerSize(1);
                if (line_color)
                {
                    collisionGraph->SetLineColor(kRed);
                    collisionGraph->SetMarkerColor(kRed);
                }
                else
                {
                    collisionGraph->SetLineColor(kWhite);
                    collisionGraph->SetMarkerColor(kWhite);
                }

                // Start creating the graph for tree expansions
                std::vector<int> tree_expansion_x;
                std::vector<int> tree_expansion_y;

                int expansion_count = 0;

                for( Node t : all_nodes)
                {
                    tree_expansion_x.push_back(t.point.coordinates[a]);
                    tree_expansion_y.push_back(t.point.coordinates[i]);
                    expansion_count++;
                } 

                TGraph *tree_expansion_graph = new TGraph(tree_expansion_x.size(), tree_expansion_x.data(), tree_expansion_y.data());
                tree_expansion_graph->GetXaxis()->SetLimits(0, time_boundary[a]); 
                tree_expansion_graph->GetYaxis()->SetRangeUser(0, time_boundary[i]); 
                tree_expansion_graph->SetMarkerStyle(20);
                tree_expansion_graph->SetMarkerSize(1);
                tree_expansion_graph->SetMarkerColor(kGray);

                // Start creating the graph for time path
                std::vector<int> safe_path_x;
                std::vector<int> safe_path_y;

                int n = 0;

                for( Node t : path)
                {
                    safe_path_x.push_back(t.point.coordinates[a]);
                    safe_path_y.push_back(t.point.coordinates[i]);
                    n++;
                } 

                TGraph *safe_path_graph = new TGraph(safe_path_x.size(), safe_path_x.data(), safe_path_y.data());
                safe_path_graph->GetXaxis()->SetLimits(0, time_boundary[a]); 
                safe_path_graph->GetYaxis()->SetRangeUser(0, time_boundary[i]); 
                safe_path_graph->SetMarkerStyle(20);
                safe_path_graph->SetMarkerSize(1);
                safe_path_graph->SetLineColor(kBlack);
                safe_path_graph->SetMarkerColor(kBlack);

                mg->Add(tree_expansion_graph, "P");
                mg->Add(collisionGraph, "P");
                mg->Add(safe_path_graph, "PL");

                mg->SetTitle(title); 
                mg->SetName(title); 

                mg->GetXaxis()->SetTitle((std::string( "schedule " + std::to_string(a+1)) ).c_str());
                mg->GetYaxis()->SetTitle((std::string( "schedule " + std::to_string(i+1)) ).c_str());

                time_canvas->cd(cd_count);
                mg->Draw("APL");

                cd_count++;   

            } 
            // Found key is not relevant
            else if ( a != i)
            {
                std::string pad_name = "Redundant Zone: (" + std::to_string(a+1) + ", " + std::to_string(i+1) + ")";
                const char* title = pad_name.c_str(); // Convert to const char*
                cd_count++;    
            }
        }
    }

    // Display second canvas
    time_canvas->GetFrame()->SetBorderSize(12);
    time_canvas->Modified();
    time_canvas->Update();

}

void CoordinationVisualization::draw_velocity(double max_vel, int AGV_amount, double dt,
    std::vector<std::vector<std::pair<double, double>>> individual_schedules, char* canvas_name)
{
    std::cout << "Drawing velocity profiles..." << std::endl;

    TCanvas* tv_canvas = new TCanvas(canvas_name, canvas_name, 300 * AGV_amount, 220 * AGV_amount);

    // Divide the canvas into segments that represent path pair tv
    tv_canvas->Divide(AGV_amount / 2 + (AGV_amount % 2 != 0), 2, 0.01, 0.01);
    tv_canvas->SetGrid();

    int pad_amount = AGV_amount;

    int cd_count = 1;

    for (int a = 0; a < AGV_amount; a++) 
    {
        auto current_schedule = individual_schedules[a];

        TMultiGraph *mg = new TMultiGraph();

        mg->GetXaxis()->SetLimits(0, individual_schedules[a][individual_schedules[a].size() - 1].first); 
        mg->GetYaxis()->SetRangeUser(-1 * max_vel * 2, max_vel * 2); 

        std::string pad_name = "Velocities: (" + std::to_string(a+1) + ")";
        const char* title = pad_name.c_str(); // Convert to const char*

        std::vector<double> time;
        std::vector<double> velocity;
        std::vector<double> acceleration;
        std::vector<double> total_distances;
        double total_distance = 0;
        double max_total_distance = 0;

        for ( int i = 0; i < individual_schedules[a].size(); i++ )
        {
            time.push_back(individual_schedules[a][i].first);
            velocity.push_back(individual_schedules[a][i].second);
            if ( i != individual_schedules[a].size() )
            {
                acceleration.push_back((individual_schedules[a][i+1].second - individual_schedules[a][i].second) / dt);
            }
            else
            {
                acceleration.push_back(acceleration.back());
            }
            if ( i != 0 )
            {
                total_distance += (individual_schedules[a][i].first - individual_schedules[a][i-1].first) * individual_schedules[a][i].second;
                total_distances.push_back(total_distance);
            }
            else
            {
                total_distance += individual_schedules[a][i].first * individual_schedules[a][i].second;
                total_distances.push_back(total_distance);
            }
            if ( total_distance > max_total_distance )
            {
                max_total_distance = total_distance;
            }
        }
        double scale = max_vel * 2 / max_total_distance ;
        for ( int i = 0; i < total_distances.size(); i++ )
        {
            total_distances[i] = scale * total_distances[i];
        }

        TGraph *tv_graph = new TGraph(time.size(), time.data(), velocity.data());
        tv_graph->GetXaxis()->SetLimits(0, individual_schedules[a][individual_schedules[a].size() - 1].first); 
        tv_graph->GetYaxis()->SetRangeUser(0, max_vel * 10); 
        tv_graph->SetMarkerColorAlpha(1, 0.3);
        tv_graph->SetMarkerStyle(20);
        tv_graph->SetMarkerSize(0.5);

        TGraph *ta_graph = new TGraph(time.size(), time.data(), acceleration.data());
        ta_graph->GetXaxis()->SetLimits(0, individual_schedules[a][individual_schedules[a].size() - 1].first); 
        ta_graph->GetYaxis()->SetRangeUser(0, max_vel * 10); 
        ta_graph->SetMarkerStyle(20);
        ta_graph->SetMarkerSize(0.5);

        ta_graph->SetLineColor(kRed);
        ta_graph->SetMarkerColor(kRed);

        TGraph *td_graph = new TGraph(time.size(), time.data(), total_distances.data());
        td_graph->GetXaxis()->SetLimits(0, individual_schedules[a][individual_schedules[a].size() - 1].first); 
        td_graph->GetYaxis()->SetRangeUser(0, max_vel * 10); 
        td_graph->SetMarkerStyle(20);
        td_graph->SetMarkerSize(0.5);

        td_graph->SetLineColor(kGreen);
        td_graph->SetMarkerColor(kGreen);

        mg->Add(td_graph, "PL");
        mg->Add(ta_graph, "PL");
        mg->Add(tv_graph, "P");

        mg->SetTitle(title); 
        mg->SetName(title); 

        mg->GetXaxis()->SetTitle((std::string( "time " + std::to_string(a+1) + ", distance was scaled down by " + std::to_string(scale))).c_str());
        mg->GetYaxis()->SetTitle((std::string( "distance(d) / velocity(b) / acceleration(r)" + std::to_string(a+1)) ).c_str());

        tv_canvas->cd(cd_count);
        gPad->SetGrid(1,1);
        mg->Draw("APL");

        cd_count++;   

    } 
    tv_canvas->Modified();
    tv_canvas->Update();

}