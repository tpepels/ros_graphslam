#ifndef GRAPHSLAM_H
#define GRAPHSLAM_H
#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
//
#include "graph.h"
#include "scanmatcher.h"
#include "graphnodes.h"

#ifndef PI
#define PI 3.14159265359
#endif

using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;

class GraphSlam {
public:
	GraphSlam(ros::NodeHandle& nh);
	~GraphSlam();
	void spin();
private:
	ros::Subscriber laserScan_Sub, odometry_Sub;
	ros::Publisher map_publish, pose_publish, graph_publish, pose_publisher;
	tf::TransformListener tf_listener;
	// The last pose and corresponding scan
	Pose prev_graph_odom;
	LaserScan::ConstPtr cur_scan, cur_sm_scan;
	double resolution, min_node_dist, min_node_rot, range_t, min_sm_dist, min_sm_rot;
	int solve_after_nodes, solve_iterations;
	//
	bool odom_updated, scan_updated, first_scan;
	// For scanmatching
	GraphPose cur_sm_pose, prev_sm_pose;
	bool sm_odom_updated, sm_pose_updated;
	Pose prev_sm_odom;
	ScanMatcher matcher;
	//
	Graph* graph;
	//
	float distance(float x1, float x2, float y1, float y2);
	float rot_distance(float theta1, float theta2);
	void laserScan_callback(const LaserScan::ConstPtr& msg);
	void odom_callback(const nav_msgs::Odometry& msg);
	void drawPoses();
	void drawScans();
	void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);
};
#endif