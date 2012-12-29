#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
//
#include "graph.h"

using namespace std;

class GraphSlam {
public:
	GraphSlam(ros::NodeHandle& nh);
	~GraphSlam();
	void spin();
private:
	ros::Subscriber laserScan_Sub, odometry_Sub;
	ros::Publisher map_publish, pose_publish, graph_publish;
	// The last pose and corresponding scan
	geometry_msgs::Pose cur_pose;
	sensor_msgs::LaserScan cur_scan;
	//
	bool odom_updated, scan_updated, first_scan;
	Graph graph;
	//
	void laserScan_callback(const sensor_msgs::LaserScan& msg);
	void odom_callback(const nav_msgs::Odometry& msg);
	float pose_distance(Pose* p1, Pose* p2);
	void drawPoses(Graph& graph);
	void drawScans(Graph& graph);
};