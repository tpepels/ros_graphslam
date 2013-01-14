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

#ifndef PI
#define PI 3.14159265359
#endif

using namespace std;
using namespace geometry_msgs;

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
	Pose cur_pose;
	sensor_msgs::LaserScan::ConstPtr cur_scan;
	double resolution, min_dist, min_rot, range_t;
	int solve_after_nodes, solve_iterations;
	//
	bool odom_updated, scan_updated, first_scan;
	Graph* graph;
	//
	Pose getFramePose(string frame, string fixed_frame, ros::Time stamp);
	void laserScan_callback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void odom_callback(const nav_msgs::Odometry& msg);
	void drawPoses();
	void drawScans();
};
#endif