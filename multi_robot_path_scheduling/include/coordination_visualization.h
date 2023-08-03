#ifndef COORDINATION_VISUALIZATION_H
#define COORDINATION_VISUALIZATION_H

#include <TCanvas.h>
#include <TGraph.h>
#include <TRandom.h>
#include <TMarker.h>
#include <TMultiGraph.h>
#include <TEllipse.h>
#include <TH2F.h>
#include <TText.h>
#include <TROOT.h>
#include <TApplication.h>
#include "TFile.h"
#include "TH1F.h"
#include "TBrowser.h"
#include "TFrame.h"
#include "TImage.h"

#include "project_utilities.h"

/**
 * Visualizer for AGV coordination problem
 *
 * Is used to visualize the physical collision space and the collision 
 * in time. Generates canvases and draws the necessary data on them.
 */
class CoordinationVisualization
{
public:
    // Constructor
    CoordinationVisualization( int space_boundary_x, int space_boundary_y, int time_boundary );

    void draw_space(std::vector<std::vector<Point>>& all_paths, 
        std::vector<std::pair<int, int>> all_collisions_space, int AGV_radius, char* canvas_name);

    void draw_time(int AGV_amount, std::vector<Node> path, std::vector<Node> all_nodes,
        std::unordered_map<std::pair<int, int>, std::unordered_map<std::pair<int, int>, bool, pair_hash>, pair_hash>& all_collisions_time, char* canvas_name);

private:
    int space_boundary_x;
    int space_boundary_y;
    int time_boundary;

    TCanvas* space_canvas;
    TCanvas* time_canvas;

};

#endif // AGVCOLLISIONSPACE_H