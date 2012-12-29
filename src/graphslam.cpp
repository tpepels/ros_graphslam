#include "ros/ros.h"
#include "graphslam.h"

using namespace std;

const int MIN_DIST = 0.5;
//
GraphSlam::GraphSlam(ros::NodeHandle& nh) {
	// Subscribe to odom an laser scan messages
	laserScan_Sub = nh.subscribe("base_scan", 100, &GraphSlam::laserScan_callback, this);
	odometry_Sub = nh.subscribe("odom", 100, &GraphSlam::odom_callback, this);
	//
	map_publish = nh.advertise<nav_msgs::OccupancyGrid> ("/map", 1, false);
	pose_publish = nh.advertise<geometry_msgs::PoseArray>("/scan_node", 1);
	//
	odom_updated = false;
	scan_updated = false;
	first_scan = true;
	// Set the initial pose to 0,0,0
	cur_pose.x = 0.;
	cur_pose.y = 0.;
	cur_pose.t = 0.;
}
;

float GraphSlam::pose_distance(Pose* p1, Pose* p2) {
	return (sqrt((p2->x-p1->x)*(p2->x-p1->x) + (p2->y-p1->y) * (p2->y-p1->y)));
}

void GraphSlam::laserScan_callback(const sensor_msgs::LaserScan& msg){
	scan_updated = true;
	cur_scan = msg;
	// This means the robot is at the origin.
	if(!odom_updated && first_scan) {
		odom_updated = true;
	}
	first_scan = false;
};

void GraphSlam::odom_callback(const nav_msgs::Odometry& msg){
	if(scan_updated && cur_pose != null) {
		if(pose_distance(&cur_pose, &pose) >= MIN_DIST) {
			cur_pose = msg;
			odom_updated = true;
		}
	}
};

void GraphSlam::spin() {
	ros::Rate rate(10); // Specify the FSM loop rate in Hz
	while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		// Check if messages have been received, first_scan is for the scan at the origin
		if(odom_updated && scan_updated) {
			// Call the graph-slam update here
		}
		//
		rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
	}
}
;

int main(int argc, char **argv){
	ros::init(argc, argv, "GraphSlam");
	ros::NodeHandle n;
	GraphSlam g_slam(n);
	ROS_INFO("INFO!");
	g_slam.spin(); // Execute FSM loop
	return 0;
}
;