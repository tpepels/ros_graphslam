#include "ros/ros.h"
#include "graphslam.h"

using namespace std;

const float MIN_DIST = 0.5, MIN_ROT = 0.5;
const float PI = 3.141592654;
//
GraphSlam::GraphSlam(ros::NodeHandle& nh) {
	// Subscribe to odom an laser scan messages
	laserScan_Sub = nh.subscribe("base_scan", 100, &GraphSlam::laserScan_callback, this);
	odometry_Sub = nh.subscribe("odom", 100, &GraphSlam::odom_callback, this);
	//
	map_publish = nh.advertise<nav_msgs::OccupancyGrid> ("/map", 1, false);
	pose_publish = nh.advertise<geometry_msgs::PoseArray>("/pose", 1);
	graph_publish = nh.advertise<visualization_msgs::Marker>("/graph_vis", 1);
	//
	odom_updated = false;
	scan_updated = false;
	first_scan = true;
	// Set the initial pose to 0,0,0
	cur_pose.x = 0.;
	cur_pose.y = 0.;
	//
	graph = new Graph(0.05, 0.9);
}
;

~GraphSlam() {
	delete graph;
}
;


void GraphSlam::laserScan_callback(const sensor_msgs::LaserScan& msg){
	scan_updated = true;
	cur_scan = msg;
	// This means the robot is at the origin.
	if(!odom_updated && first_scan) {
		odom_updated = true;
	}
	first_scan = false;
}
;

float GraphSlam::pose_distance(geometry_msgs::Pose* p1, geometry_msgs::Pose* p2) {
	return (sqrt(pow(p2->x - p1->x), 2) + pow((p2->y - p1->y), 2));
}
;

float GraphSlam::rotation_distance(geometry_msgs::Pose* p1, geometry_msgs::Pose* p2) {
	float dist = tf::getYaw(p1.orientation) - tf::getYaw(p2.orientation);
	// Flip the distance if negative
	if (dist < 0) {
		dist = -dist;
	}

	// Return distance between PI and -PI
	if (dist > PI) {
		return 2 * PI - dist;
	}
	return dist;
}
;

void GraphSlam::odom_callback(const nav_msgs::Odometry& msg){
	if(scan_updated && cur_pose != null) {
		if(pose_distance(&cur_pose, &pose) >= MIN_DIST || rotation_distance(&cur_pose, &pose) <= MIN_ROT) {
			cur_pose = msg;
			odom_updated = true;
		}
	}
}
;

void GraphSlam::spin() {
	ros::Rate rate(10); // Specify the FSM loop rate in Hz
	while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		// Check if messages have been received, first_scan is for the scan at the origin
		if(odom_updated && scan_updated) {
			graph.addNode(cur_pose, cur_scan);
			graph.generateMap();
			map_publish.publish(graph.map);
			// Call the graph-slam update here
			odom_updated = false;
			scan_updated = false;
		}
		//
		rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
	}
}
;

void GraphSlam::drawPoses(){
	geometry_msgs::PoseArray poses;
	visualization_msgs::Marker nodes_message;
	visualization_msgs::Marker edges_message;
	poses.header.frame_id = "/odom";
	poses.header.stamp = ros::Time();
	nodes_message.header.frame_id = "/odom";
	nodes_message.header.stamp = ros::Time()::now();
	edges_message.header.frame_id = "/odom";
	edges_message.header.stamp = ros::Time()::now();
	//
	nodes_message.scale.x = 0.05;
	nodes_message.scale.y = 0.05;
	nodes_message.color.r = 1.0;
	nodes_message.color.a = 1.0;
	nodes_message.type = visualization_msgs::Marker::POINTS;
	nodes_message.ns = "nodes";
	//
	edges_message.scale.x = 0.05;
	edges_message.scale.y = 0.05;
	edges_message.color.g = 1.0;
	edges_message.color.a = 1.0;
	edges_message.type = visualization_msgs::Marker::LINE_LIST;
	edges_message.ns = "edges";
	//
	for(unsigned int i = 0; i < graph->node_list.size(); i++) {
		geometry_msgs::Pose new_pose;
		new_pose.x = graph.node_list[i].robot_pose.x;
		new_pose.y = graph.node_list[i].robot_pose.y;
		new_pose.orientation = graph.node_list[i].orientation;
        poses.poses.push_back(new_pose);
        //
		geometry_msgs::Point point;
		point = new_pose.position;
		nodes_message.points.push_back(point);
	}
	pose_publish.publish(poses);	
	graph_publish.publish(nodes_message);
	//
	for(unsigned int i = 0; i < graph->edge_list.size(); i++) {
		geometry_msgs::Point start;
		Pose pose = graph.edge_list.parent->pose;
		start.x = pose.x;
		start.y = pose.y;
		edges_message.points.push_back(start);
		//
		geometry_msgs::Point end;
		pose = graph.edge_list[i].child->pose;
		end.x = pose.x;
		end.y = pose.y;
		edges_message.points.push_back(end);
	}
	//
	graph_publish.publish(edges_message);
}
;

void GraphSlam::drawScans(){
	visualization_msgs::Marker scan_message;
	scan_message.header.frame_id = "/odom";
	scan_message.header.stamp = ros::Time()::now();
	scan_message.scale.x = 0.005;
	scan_message.scale.y = 0.005;
	scan_message.ns = "scans";
	scan_message.type = visualization_msgs::Marker::POINTS;
	scan_message.color.g = 1.0;
	scan_message.color.a = 1.0;
	//
	
	for(unsigned int i = 0; i < nodes.size(); i++) {
		float theta = tf::getYaw(graph.node_list[i].robot_pose.orientation);
		//
		float minimal_angle = theta + graph.node_list[i].laser_scan.angle_min;
		float current_angle = minimal_angle;
		float maximal_angle = theta + graph.node_list[i].laser_scan.angle_max;
		//
		float angle_increment = graph.node_list[i].laser_scan.angle_increment;
		float max_range = graph.node_list[i].laser_scan.range_max;
		//
		for(unsigned int j = 0; current_angle <= maximal_angle - angle_increment && j < 180; j = j + 1, current_angle = current_angle + angle_increment) {
			float range = nodes[i].scan.ranges[j];
			if(range == max_range){
				continue;
			}
			//
			float x = graph.node_list[i].pose.x + cos(current_angle) * range;
			float y = graph.node_list[i].pose.y + sin(current_angle) * range;
			//
			geometry_msgs::Point point;
			point.x = x;
			point.y = y;
			scan_message.points.push_back(point);
		}
	}
	graph_publish.publish(scan_message);
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