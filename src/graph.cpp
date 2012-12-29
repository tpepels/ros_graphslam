#include "ros/ros.h"
#include "graph.h"

using namespace std;

const int N_INF = -9999999;
const int P_INF = 9999999;

void Graph::Graph(double resolution, double range_threshold) {
	this->resolution = resolution;
	this->range_threshold = range_threshold;
};

void Graph::addNode(geometry_msgs::Pose pose, sensor_msgs::LaserScan scan){
	Node n;
	n.pose = pose;
	n.scan = scan;
	n.scan_grid = scanToOccGrid(scan, pose);
	node_list.push_back(n);
	// TODO: Match the new node's scans to previous scans and add edges accordingly
	Edge e;
	e.parent = last_node;
	e.child = n;
	edge_list.push_back(e);
    last_node = n;
}
;

// Combine the scan-information in the nodes into a complete map!
void Graph::generateMap() {

}
;

// Take a scan as input, generate a small, local occupancy grid
ScanGrid Graph::scanToOccGrid(sensor_msgs::LaserScan& scan, geometry_msgs::Pose& pose){
	ScanGrid new_grid;
	double angle_incr = scan.angle_increment;
	// First we need to know the size and bounds of the grid.
    double xmax = N_INF,xmin = P_INF,ymax = N_INF,ymin = P_INF;
    double scan_angle, scan_x, scan_y, min_angle = scan.angle_min;
    int num_scans = scan.ranges.size();
    double pose_theta = tf::getYaw(pose);
    // Get the scan positions the bound the grid
    for (int i = 0; i < num_scans; i++)
    {
        scan_angle = pose_theta + min_angle + i * angle_incr;
        scan_x = pose.x + scan.ranges[i] * cos(scan_angle);
        scan_y = pose.y + scan.ranges[i] * sin(scan_angle);

        if (ymax < scan_y)
            ymax = scan_y;
        if (xmax < scan_x)
            xmax = scan_x;
        if (ymin > scan_y)
            ymin = scan_y;
        if (xmin > scan_x)
            xmin = scan_x;
    }
    // Initialize the grid, set the bounds relative to the odometry
    new_grid.ymax = round((ymax - pose.y) / resolution);
    new_grid.ymin = round((pose.y - ymin) / resolution);
    new_grid.xmax = round((xmax - pose.x) / resolution);
    new_grid.xmin = round((pose.x - xmin) / resolution);
    // Size of the grid can be computed with the bounds
    new_grid.height = new_grid.ymax + new_grid.ymin;
    new_grid.width = new_grid.xmin + new_grid.xmax;
    // Set the grid to the correct size
    new_grid.resolution = this->resolution;
    int grid_size = new_grid.height * new_grid.width
    new_grid.grid.resize(grid_size);
    // Set each cell to empty space, as it was probably covered by the laser scanner
    for (int i= 0; i < grid_size; i++) {
        new_grid.grid[i] = 0;
    }
    //
   	double max_range = scan.range_max * range_threshold, measurement;
   	double theta, x, y;
   	int row, col;
    for (int i = 0; i < num_scans; i++)
    {
        measurement = node->scan.ranges[i];
        // Check if out of range, dont place an obstacle on the grid in this case
        if (measurement > max_range)
            continue;
        // Determine and set the location of the object in the local grid
        theta = pose_theta + min_angle + i * angle_incr;
        x = measurement * cos(theta);
        y = measurement * sin(theta);
        col = new_grid.ymin + (y / resolution);
        row = new_grid.xmin + (x / resolution);
        // 
        new_grid.grid[col * new_grid.width + row] = 100;
    }
    // It's possible to make a distinction between known and unknow space here by using scanner triangles
    // But i'm not sure if that is necessary
    return new_grid;
};
