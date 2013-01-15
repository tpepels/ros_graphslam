#ifndef GRAPH_H
#define GRAPH_H

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf/transform_listener.h"
#include <Eigen/Core>
//
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
//
#include "g2o/types/slam2d/se2.h"
#include "g2o/types/slam2d/edge_se2.h"
#include "g2o/types/slam2d/vertex_se2.h"
//
#include "graphnodes.h"
#include "scanmatcher.h"

using namespace std;
using namespace geometry_msgs;
using namespace Eigen;

class Graph {

	public:
		vector<Node*> node_list;
		vector<Edge*> edge_list;
		Node * last_node;
		//
		Graph(double resolution, double range_threshold);
		~Graph();
		void addNode(Pose pose, const sensor_msgs::LaserScan::ConstPtr& scan);
		void generateMap(nav_msgs::OccupancyGrid& cur_map);
		void solve(unsigned int iterations);
	private:
		//
		unsigned int idCounter;
		double resolution, range_threshold;
		ScanGrid scanToOccGrid(const sensor_msgs::LaserScan::ConstPtr& scan, GraphPose& pose);
		void addNearbyConstraints(int close_limit, int step_size, double dist_limit, double min_dist_delta, double min_angle_delta);
};
#endif
