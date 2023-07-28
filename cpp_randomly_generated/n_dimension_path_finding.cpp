#include "n_dimension_path_finding.h"

int main(int argc, char *argv[]) 
{
    // Initialize root
    TApplication *path_finding_app = new TApplication("path_finding", &argc, argv);

    // Coordination space properties
    int x_bound = 300; // max x value of any point on path
    int y_bound = 300; // max y value of any point on path
    int AGV_amount = 4; // amount of all_paths = robot amount iteration = iteration + 1
    int AGV_radius = 10; // radius of a circular agv
    int path_min_stops = 3; // min number of times slope can be shifted
    int path_max_stops = 4; // max number of times slope can be shifted
    int path_length_min = 50; // min length of a path segment
    int path_length_max = 70; // max length of a path segment  

    // Initialize the space information, determine paths, collisions on time and space

    std::cout << "Initializing coordination space..." << std::endl;
    auto collision_space = AGVCollisionSpace( x_bound, y_bound, AGV_amount, AGV_radius, path_min_stops, 
                                           path_max_stops, path_length_min, path_length_max );

    auto all_paths = collision_space.get_paths();
    auto all_collisions_time = collision_space.get_collision_map();
    auto all_collisions_space = collision_space.get_collision_points();
    int collision_side_dimension_length = collision_space.get_side_dimension_length();

    // Draw the collision space with paths, AGVs, and collisions

    std::cout << "Drawing collision space..." << std::endl;

    Double_t canvas_w = 1000;
    Double_t canvas_h = 1000;

    // Set space canvas with its propertis.

    TCanvas *c1 = new TCanvas("c1", "Graph Draw Options", canvas_w, canvas_h);
    c1->SetWindowSize(canvas_w + (canvas_w - c1->GetWw()), canvas_h + (canvas_h - c1->GetWh()));
    c1->SetGrid();
    c1->SetWindowSize(800,800);
    c1->cd();
    TMultiGraph *mg = new TMultiGraph();
    mg->GetXaxis()->SetLimits(0, x_bound); 
    mg->GetYaxis()->SetRangeUser(0, y_bound); 

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
        ellipseCenter->SetMarkerSize(AGV_radius * 2 * x_bound / 1000);
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

    c1->Update();
    c1->GetFrame()->SetBorderSize(12);
    c1->Modified();

    // Construct the RRT and determine the time path

    std::cout << "Constructing the RRT..." << std::endl;

    auto coordination_RRT = RRT(AGV_amount, collision_side_dimension_length, all_collisions_time);
    coordination_RRT.show_longest_generation();
    auto path = coordination_RRT.get_final_path();

    // Start drawing time canvas

    std::cout << "Drawing time space..." << std::endl;

    // Create a new TCanvas for plotting the collisions
    TCanvas *c2 = new TCanvas("c2", "Collisions", canvas_w * AGV_amount, canvas_h * AGV_amount);

    // Divide the canvas into segments that represent path pair
    c2->Divide(AGV_amount - 1, AGV_amount - 1, 0, 0);

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
                mg->GetXaxis()->SetLimits(0, collision_side_dimension_length); 
                mg->GetYaxis()->SetRangeUser(0, collision_side_dimension_length); 

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
                collisionGraph->GetXaxis()->SetLimits(0, collision_side_dimension_length); 
                collisionGraph->GetYaxis()->SetRangeUser(0, collision_side_dimension_length); 
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
                safe_path_graph->GetXaxis()->SetLimits(0, collision_side_dimension_length); 
                safe_path_graph->GetYaxis()->SetRangeUser(0, collision_side_dimension_length); 
                safe_path_graph->SetMarkerStyle(20);
                safe_path_graph->SetMarkerSize(1);


                mg->Add(collisionGraph);
                mg->Add(safe_path_graph);
                mg->SetTitle(title); 
                mg->SetName(title); 

                c2->cd(cd_count);
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
    c2->Update();
    c2->GetFrame()->SetBorderSize(12);
    c2->Modified();
    path_finding_app->Run();
    return 0;
}