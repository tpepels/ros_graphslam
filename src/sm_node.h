#ifndef SMNODE_H
#define SMNODE_H
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/transform_listener.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
//
#include "scanmatcher.h"
#include "graphnodes.h"

using namespace std;
using namespace geometry_msgs;
using namespace sensor_msgs;

class SMNode {
public:
	SMNode(ros::NodeHandle& nh);
	void spin();
private:
	ros::Subscriber laserScan_Sub, odometry_Sub;
	ros::Publisher pose_publisher;
	tf::TransformListener tf_listener;
	// The last pose and corresponding scan
	GraphPose cur_sm_pose, est_sm_pose, prev_sm_pose;
	LaserScan::ConstPtr cur_sm_scan;
	bool first_scan, sm_odom_updated, sm_pose_updated;
	// For scanmatching
	Pose prev_sm_odom, cur_odom;
	ScanMatcher matcher;

	float distance(float x1, float x2, float y1, float y2);
	float rot_distance(float theta1, float theta2);
	void laserScan_callback(const LaserScan::ConstPtr& msg);
	void odom_callback(const nav_msgs::Odometry& msg);
	Pose getFramePose(string frame, string fixed_frame, ros::Time stamp);
};
#endif