#include "graph.h"

using namespace std;

const int N_INF = -9999999;
const int P_INF = 9999999;
const double PI = 3.141592654;

Graph::Graph(double resolution, double range_threshold) {
    ROS_INFO("Graph entering constructor");
	this->resolution = resolution;
	this->range_threshold = range_threshold;
};

void Graph::addNode(geometry_msgs::Pose pose, sensor_msgs::LaserScan scan){
    ROS_INFO("Graph entering addNode");
	Node n;
	n.robot_pose = pose;
	n.laser_scan = scan;
	n.scan_grid = scanToOccGrid(scan, pose);
	node_list.push_back(n);
	// TODO: Match the new node's scans to previous scans and add edges accordingly
	Edge e;
	e.parent = &node_list[node_list.size() - 2];
	e.child = &node_list[node_list.size() - 1];
	edge_list.push_back(e);
    last_node = &n;
    ROS_INFO("Graph finished addNode");
}
;

// Combine the scan-grids in the nodes into a complete map!
void Graph::generateMap(nav_msgs::OccupancyGrid& cur_map) {
    ROS_INFO("Graph entering generateMap");
    // First we need to know the outer-bounds of the map
    double xmax = N_INF,xmin = P_INF,ymax = N_INF,ymin = P_INF;
    Node * node;
    for (unsigned int i = 0; i < node_list.size(); i++)
    {
        node = &node_list[i];
        float x = node->robot_pose.position.x, y = node->robot_pose.position.y;
        if (xmax < (x + (node->scan_grid.xmax)) * resolution)
            xmax = (x + (node->scan_grid.xmax)) * resolution;

        if (xmin > (x - (node->scan_grid.xmin)) * resolution)
            xmin = (x - (node->scan_grid.xmin)) * resolution;

        if (ymax < (y + (node->scan_grid.ymax)) * resolution)
            ymax = (y + (node->scan_grid.ymax)) * resolution;

        if (ymin > (y - (node->scan_grid.ymin)) * resolution)
            ymin = (y - (node->scan_grid.ymin)) * resolution;
    }
    ROS_INFO("Graph map bounds: %f, %f, %f, %f", xmin, xmax, ymin, ymax);
    // Map size
    double map_height = (ymax - ymin) / resolution;
    double map_width = (xmax - xmin) / resolution;
    // Increase the size of the map slightly so we don't run into any rounding errors
    map_width += 2;
    map_height += 2;
    unsigned int map_size = (unsigned int) round(map_height * map_width);
    ROS_INFO("Graph map size: %d", map_size);
    //
    cur_map.header.frame_id = "/odom";
    cur_map.header.stamp = ros::Time().now();
    cur_map.info.height = map_height;
    cur_map.info.width = map_width;
    cur_map.info.resolution = resolution;
    //
    cur_map.data.resize(map_size);
    geometry_msgs::Pose origin;
    origin.orientation.x = 0.;
    origin.orientation.y = 0.;
    origin.orientation.z = 0.;
    origin.orientation.w = 1.;
    origin.position.x = xmin;
    origin.position.y = ymin;
    cur_map.info.origin = origin;
    // This vector counts how many times a map-position was seen in the graph
    // Later we use this to compute the certainty that a position is an obstacle
    vector<unsigned int> pos_seen;
    pos_seen.resize(map_size);
    // These will count how many times a position was blocked/free. All others will be unknown
    vector<unsigned int> pos_fr (map_size);
    pos_fr.resize(map_size);
    vector<unsigned int> pos_blocked (map_size);
    pos_blocked.resize(map_size);
    ROS_INFO("Graph combining local grids");
    // Go through all nodes' scan-grids and maintain the position information for each
    for (unsigned int i = 0; i < node_list.size(); i++)
    {
        node = &node_list[i];
        float x = node->robot_pose.position.x, y = node->robot_pose.position.y;
        int node_x = round((x - xmin) / resolution) - node->scan_grid.xmin;
        int node_y = round((y - ymin) / resolution) - node->scan_grid.ymin;
        // Go through the local map, and count the occupancies for the global map
        for (int j = 0; j < node->scan_grid.height; j++) {
            for (int k = 0; k < node->scan_grid.width; k++) {
                // Index in the global map
                int global_index = (map_width * (node_y + j) + k) + node_x;
                pos_seen[global_index]++;
                // The value of the local grid
                int local_index = (j * node->scan_grid.width) + k;
                int value = node->scan_grid.grid[local_index];
                //
                if (value == 100) // Position is blocked
                    pos_blocked[global_index]++;
                if (value == 0) // Position is unoccupied
                    pos_fr[global_index]++;
            }
        }
    }
    ROS_INFO("Graph determining occupied/free/unknown");
    // Now, we can update the global map according to what we saw in the local maps
    for(unsigned int i = 0; i < map_size; i++) {
        // The position was free more often than blocked
        if(pos_fr[i] > pos_blocked[i])
            cur_map.data[i] = 0;
        else if (pos_fr[i] < pos_blocked[i])
            cur_map.data[i] = 100;
        else if (pos_fr[i] == pos_blocked[i]) {
            // The position was as often blocked as free, use the seen to break the tie
            // TODO: What to do here? Set to unknown so it gets visited again?
            cur_map.data[i] = -1;
        } else {
            cur_map.data[i] = -1;
        }
    }
    ROS_INFO("Graph finished generateMap");
}
;

// Take a scan as input, generate a small, local occupancy grid
ScanGrid Graph::scanToOccGrid(sensor_msgs::LaserScan& scan, geometry_msgs::Pose& pose){
    ROS_INFO("Graph entering scanToOccGrid");
	ScanGrid new_grid;
	double angle_incr = scan.angle_increment;
	// First we need to know the size and bounds of the grid.
    double xmax = N_INF,xmin = P_INF,ymax = N_INF,ymin = P_INF;
    double scan_angle, min_angle = scan.angle_min;
    int num_scans = scan.ranges.size();
    double pose_theta = tf::getYaw(pose.orientation);
    // Get the scan positions the bound the grid
    for (int i = 0; i < num_scans; i++)
    {
        scan_angle = pose_theta + min_angle + i * angle_incr;
        if (ymax < pose.position.y + scan.ranges[i] * sin(scan_angle))
            ymax = pose.position.y + scan.ranges[i] * sin(scan_angle);
        if (xmax < pose.position.x + scan.ranges[i] * cos(scan_angle))
            xmax = pose.position.x + scan.ranges[i] * cos(scan_angle);
        if (ymin > pose.position.y + scan.ranges[i] * sin(scan_angle))
            ymin = pose.position.y + scan.ranges[i] * sin(scan_angle);
        if (xmin > pose.position.x + scan.ranges[i] * cos(scan_angle))
            xmin = pose.position.x + scan.ranges[i] * cos(scan_angle);
    }
    // Initialize the grid, set the bounds relative to the odometry
    new_grid.ymax = round((ymax - pose.position.y) / resolution);
    new_grid.ymin = round((pose.position.y - ymin) / resolution);
    new_grid.xmax = round((xmax - pose.position.x) / resolution);
    new_grid.xmin = round((pose.position.x - xmin) / resolution);
    // Size of the grid can be computed with the bounds
    new_grid.height = new_grid.ymax + new_grid.ymin;
    new_grid.width = new_grid.xmin + new_grid.xmax;
    // Set the grid to the correct size
    int grid_size = new_grid.height * new_grid.width;
    new_grid.grid.resize(grid_size);
    // Set each cell to unknown space, later we will it with obstacles and known space
    for (int i= 0; i < grid_size; i++) {
        new_grid.grid[i] = -1;
    }
    //
   	double max_range = scan.range_max * range_threshold, measurement;
   	double theta, x, y;
   	int row, col;
    for (int i = 0; i < num_scans; i++)
    {
        measurement = scan.ranges[i];
        // Check if out of range, dont place an obstacle on the grid in this case
        if (measurement > max_range)
            continue;
        // Determine and set the location of the object in the local grid
        theta = pose_theta + min_angle + i * angle_incr;
        x = measurement * cos(theta);
        y = measurement * sin(theta);
        col = new_grid.ymin + round(y / resolution);
        row = new_grid.xmin + round(x / resolution);
        // 
        new_grid.grid[col * new_grid.width + row] = 100;
    }
    // Use triangulation to fill the grid covered by the scan with known space
    double range, scan_dist, scan_theta, scan_end;
    int index, scan_index;
    for (int i = 0; i < new_grid.height; i++) {
        for (int j = 0; j < new_grid.width; j++) {
            index = i * new_grid.width + j;
            // We can skip positions we know are occupied
            if (new_grid.grid[index] == 100)
                continue;
            scan_theta = atan2(i - new_grid.ymin, j - new_grid.xmin) - pose_theta - scan.angle_min;
            scan_theta -= floor(scan_theta / PI / 2) * (PI * 2);
            // Check if the scan is out of bounds
            scan_index = round(scan_theta / scan.angle_increment);
            if (scan_index < 0|| scan_index >= num_scans) {
                new_grid.grid[index] = -1;
                continue;
            }
            //
            range = scan.ranges[scan_index];
            scan_dist = sqrt(pow(j - new_grid.xmin, 2) + pow(i - new_grid.ymin, 2));
            scan_end = range - (scan_dist * resolution);
            // If the end of the scan is outside the bounds of the grid, it will be unknown
            // For instance, it will be behind a wall. Otherwise, it is in between the robot and the wall.
            if(scan_end > 0)
                new_grid.grid[index] = 0;
            else
                new_grid.grid[index] = -1;
        }
    }
    ROS_INFO("Graph finished scanToOccGrid");
    return new_grid;
};
