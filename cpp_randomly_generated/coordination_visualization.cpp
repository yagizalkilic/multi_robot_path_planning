#include "coordination_visualization.h"

CoordinationVisualization::CoordinationVisualization(int space_boundary_x, int space_boundary_y, int time_boundary)
{
	this->space_boundary_x = space_boundary_x;
    this->space_boundary_y = space_boundary_y;
    this->time_boundary = time_boundary;
}

void CoordinationVisualization::draw_space(std::vector<std::vector<Point>>& all_paths,
	std::vector<std::pair<int, int>> all_collisions_space, int AGV_radius)
{
    std::cout << "Drawing collision space..." << std::endl;

    TCanvas* space_canvas = new TCanvas("space_canvas", "Graph Draw Options", 1000, 1000);
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
        const auto &path = all_paths[idx];
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
        ellipseCenter->SetMarkerSize(AGV_radius * 2 * space_boundary_x / 1000);
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

    space_canvas->Update();
    space_canvas->GetFrame()->SetBorderSize(12);
    space_canvas->Modified();
}

void CoordinationVisualization::draw_time(int AGV_amount, std::vector<Node> path, 
	std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash>& all_collisions_time)
{
	std::cout << "Drawing time space..." << std::endl;

	TCanvas* time_canvas = new TCanvas("time_canvas", "Collisions", 1000 * AGV_amount, 1000 * AGV_amount);

    // Divide the canvas into segments that represent path pair
    time_canvas->Divide(AGV_amount - 1, AGV_amount - 1, 0, 0);

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
                mg->GetXaxis()->SetLimits(0, time_boundary); 
                mg->GetYaxis()->SetRangeUser(0, time_boundary); 

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
                collisionGraph->GetXaxis()->SetLimits(0, time_boundary); 
                collisionGraph->GetYaxis()->SetRangeUser(0, time_boundary); 
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
                safe_path_graph->GetXaxis()->SetLimits(0, time_boundary); 
                safe_path_graph->GetYaxis()->SetRangeUser(0, time_boundary); 
                safe_path_graph->SetMarkerStyle(20);
                safe_path_graph->SetMarkerSize(1);


                mg->Add(collisionGraph);
                mg->Add(safe_path_graph);
                mg->SetTitle(title); 
                mg->SetName(title); 

                time_canvas->cd(cd_count);
                mg->Draw("AP");
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
    time_canvas->Update();
    time_canvas->GetFrame()->SetBorderSize(12);
    time_canvas->Modified();
}