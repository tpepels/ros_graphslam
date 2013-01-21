#include "graphslam.h"

using namespace std;
using namespace geometry_msgs;

//
GraphSlam::GraphSlam(ros::NodeHandle& nh) {
	// Subscribe to odom an laser scan messages
	laserScan_Sub = nh.subscribe("base_scan", 1, &GraphSlam::laserScan_callback, this);
	odometry_Sub = nh.subscribe("odom", 1, &GraphSlam::odom_callback, this);
	//
	map_publish = nh.advertise<nav_msgs::OccupancyGrid> ("/map", 1, false);
	pose_publish = nh.advertise<geometry_msgs::PoseArray>("/pose", 1);
	graph_publish = nh.advertise<visualization_msgs::Marker>("/graph_vis", 1);
	pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/last_pose", 1);
	// Check if parameters are set correctly
	if (!nh.hasParam("graphslam/resolution"))
    	ROS_WARN("No param named 'graphslam/resolution'");
	//
	nh.param("graphslam/resolution", resolution, 0.05);
    nh.param("graphslam/solve_iterations", solve_iterations, 20);
    // After this distance, a new node will be added to the graph
    nh.param("graphslam/min_node_dist", min_node_dist, 0.3);
    nh.param("graphslam/min_node_rot", min_node_rot, 0.45);
    // This is the distance used for the scanmatching correction
    nh.param("graphslam/min_sm_dist", min_sm_dist, 0.05);
    nh.param("graphslam/min_sm_rot", min_sm_rot, 0.15);
    //
    nh.param("graphslam/solve_after_nodes", solve_after_nodes, 10);
    nh.param("graphslam/laser_range_t", range_t, 0.95);
	//
	odom_updated = false;
	scan_updated = false;
	first_scan = true;
	// Set the initial pose for the graph and scanmatcher to 0,0,0
	cur_sm_pose.x = 0.;
	cur_sm_pose.y = 0.;
	cur_sm_pose.theta = 0;
	//
	prev_graph_odom.position.x = 0;
	prev_graph_odom.position.y = 0;
	prev_graph_odom.orientation = tf::createQuaternionMsgFromYaw(0);
	prev_sm_odom = prev_graph_odom;
	//cur_sm_odom = prev_sm_odom;
	//
	graph = new Graph(resolution, range_t);
	ROS_INFO("GraphSlam Constructor finished");
}
;

GraphSlam::~GraphSlam() {
	delete graph;
}
;

void GraphSlam::laserScan_callback(const LaserScan::ConstPtr& msg){	
	// Store the first reference scan for later use
	if(first_scan)
		cur_sm_scan = msg;
	// Check if we will perform scanmatching
	if(!first_scan && sm_odom_updated) {
		//ROS_INFO("Graphslam scanmatching!");
		double mean[3];
		double error;
		LaserScan scan = *msg;
		LaserScan ref_scan = *cur_sm_scan;		
		//
		bool result = matcher.scanMatch(scan, cur_sm_pose, ref_scan, prev_sm_pose, mean, error);
		//ROS_INFO("SM Error %f", error);
		if(result) {
			ROS_INFO("Estimated pose: x %f y: %f t: %f", cur_sm_pose.x, cur_sm_pose.y, cur_sm_pose.theta);
			ROS_INFO("ScanMatched pose: x %f y: %f t: %f", mean[0], mean[1], mean[2]);
			cur_sm_pose.x = mean[0];
			cur_sm_pose.y = mean[1];
			cur_sm_pose.theta = mean[2];
		} else {
			// In this case, cur_sm_pose will have the value updated by the odometry
			// ROS_WARN("Scanmatching in GraphSlam failed.");
			ROS_WARN("Using estimated pose: x %f y: %f t: %f", cur_sm_pose.x, cur_sm_pose.y, cur_sm_pose.theta);
		}
		// 
		cur_sm_scan = msg;
		sm_odom_updated = false;
		sm_pose_updated = true;
	}
	// Check if a node will get added to the graph
	if(odom_updated || first_scan) {
		//ROS_INFO("Odom and scan updated!");
		scan_updated = true;
		cur_scan = msg;
		// This means the robot is at the origin.
		if(!odom_updated && first_scan) {
			odom_updated = true;
		}
		first_scan = false;
	}
}
;

void GraphSlam::rosToGraphPose(const Pose& ros_pose, GraphPose& g_pose) {
	g_pose.x = ros_pose.position.x;
	g_pose.y = ros_pose.position.y;
	g_pose.theta = tf::getYaw(ros_pose.orientation);
}
;

void GraphSlam::odom_callback(const nav_msgs::Odometry& msg){
	float new_x = msg.pose.pose.position.x, new_y = msg.pose.pose.position.y;
	float new_theta = tf::getYaw(msg.pose.pose.orientation);
	// Check if we're going to add a node to the graph
	float g_dist = distance(prev_graph_odom.position.x, new_x, prev_graph_odom.position.y, new_y);
	float g_rot_dist = rot_distance(new_theta, tf::getYaw(prev_graph_odom.orientation));
	//
	if(g_dist >= min_node_dist || abs(g_rot_dist) >= min_node_rot) {
		odom_updated = true;
		prev_graph_odom = msg.pose.pose;
	}
	// check if we will do some scanmatching
	float prev_theta = tf::getYaw(prev_sm_odom.orientation);
	float sm_dist = distance(prev_sm_odom.position.x, new_x, prev_sm_odom.position.y, new_y);
	float sm_rot_dist = rot_distance(new_theta, prev_theta);
	// If the new distance is large enough update the current pose based on the scanmatching result
	if(odom_updated || sm_dist >= min_sm_dist || abs(sm_rot_dist) >= min_sm_rot) {
		// ROS_INFO("Odom changed by: xy: %f, theta: %f", sm_dist, sm_rot_dist);
		// Store the previous pose as reference
		prev_sm_pose = cur_sm_pose;
		// Apply the odometry motion model to get an initial estimate of the new position
		float drot1 = atan2(new_y - prev_sm_odom.position.y, new_x - prev_sm_odom.position.x) - prev_theta;
		float drot2 = sm_rot_dist - drot1;

		cur_sm_pose.x += sm_dist * cos(prev_theta + drot1);
		cur_sm_pose.y += sm_dist * sin(prev_theta + drot1);
		cur_sm_pose.theta += drot1 + drot2;

		prev_sm_odom = msg.pose.pose;
		sm_odom_updated = true;
		// cur_sm_odom = msg.pose.pose;
	}
}
;

void GraphSlam::spin() {
	ros::Rate rate(10); // Specify the FSM loop rate in Hz
	while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		// Check if messages have been received
		if(odom_updated && scan_updated) {
			// ROS_INFO("GraphSlam odom and scan updated!");
			graph->addNode(cur_sm_pose, cur_scan);
			nav_msgs::OccupancyGrid cur_map;
			graph->generateMap(cur_map);
			//
			if(graph->node_list.size() > 2 && graph->node_list.size() % solve_after_nodes == 0)
				graph->solve(solve_iterations);
			// ROS_INFO("GraphSlam Map generated");
			map_publish.publish(cur_map);
			// Publish the graph, scans and poses
			this->drawPoses();
			this->drawScans();
			odom_updated = false;
			scan_updated = false;
  			// ROS_INFO("Published last known pose: x: %f, y %f, t: %f", last_pose.x, last_pose.y, last_pose.theta);
		}
		if(sm_pose_updated) {
			// Publish the lase scanmatching pose
			geometry_msgs::PoseStamped p;
			p.header.stamp = ros::Time().now();
			p.header.frame_id = "/odom";
			p.pose.position.x = cur_sm_pose.x;
			p.pose.position.y = cur_sm_pose.y;
			p.pose.orientation = tf::createQuaternionMsgFromYaw(cur_sm_pose.theta);
			pose_publisher.publish(p);
			sm_pose_updated = false;
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
	nodes_message.header.stamp = ros::Time().now();
	edges_message.header.frame_id = "/odom";
	edges_message.header.stamp = ros::Time().now();
	//
	edges_message.scale.x = 0.01;
	edges_message.scale.y = 0.01;
	edges_message.color.g = 1.0;
	edges_message.color.a = 1.0;
	edges_message.type = visualization_msgs::Marker::LINE_LIST;
	edges_message.ns = "edges";
	//
	nodes_message.scale.x = 0.05;
	nodes_message.scale.y = 0.05;
	nodes_message.color.r = 1.0;
	nodes_message.color.a = 1.0;
	nodes_message.type = visualization_msgs::Marker::POINTS;
	nodes_message.ns = "nodes";
	//
	// Publish all poses in the graph!
	for(unsigned int i = 0; i < graph->node_list.size(); i++) {
		geometry_msgs::Pose new_pose;
		new_pose.position.x = graph->node_list[i]->graph_pose.x;
		new_pose.position.y = graph->node_list[i]->graph_pose.y;
		new_pose.orientation = tf::createQuaternionMsgFromYaw(graph->node_list[i]->graph_pose.theta);
        poses.poses.push_back(new_pose);
        // Add some nice points to display the graph
        geometry_msgs::Point point;
		point = new_pose.position;
		nodes_message.points.push_back(point);
	}
	pose_publish.publish(poses);
	graph_publish.publish(nodes_message);

	unsigned int node_list_size = graph->node_list.size();
	// unsigned int edge_list_size = graph->edge_list.size();
	// Publish the edges between all poses
	for(unsigned int i = 0; i < graph->edge_list.size(); i++) {
		geometry_msgs::Point start;
		//
		GraphPose *parent_pose, *child_pose;
		unsigned int edge_parent_id = graph->edge_list[i]->parent_id;
		// ROS_INFO("Parent id: %d", edge_parent_id);
		for(unsigned int k = 0; k < node_list_size; k++){
			if(graph->node_list[k]->id == edge_parent_id){
				parent_pose = &(graph->node_list[k]->graph_pose);
				// ROS_INFO("Found parent! id %d", graph->node_list[k]->id);
			}
		}
		// ROS_INFO("PARENT X: %f, Y: %f", parent_pose->x, parent_pose->y);
		start.x = parent_pose->x;
		start.y = parent_pose->y;
		//
		geometry_msgs::Point end;
		unsigned int edge_child_id = graph->edge_list[i]->child_id;
		// ROS_INFO("Child id: %d", edge_child_id);
		for(unsigned int k = 0; k < node_list_size; k++){
			if(graph->node_list[k]->id == edge_child_id){
				child_pose = &(graph->node_list[k]->graph_pose);
				// ROS_INFO("Found child! id %d", graph->node_list[k]->id);
			}
		}
		//
		// ROS_INFO("CHILD X: %f, Y: %f", child_pose->x, child_pose->y);
		end.x = child_pose->x;
		end.y = child_pose->y;
		// ROS_INFO("Added edge from x %f y %f, to x %f y %f", start.x, start.y, end.x, end.y);
		//
		edges_message.points.push_back(start);
		edges_message.points.push_back(end);
		child_pose = 0;
		parent_pose = 0;
	}
	//
	graph_publish.publish(edges_message);
}
;

void GraphSlam::drawScans(){
	visualization_msgs::Marker scan_message;
	scan_message.header.frame_id = "/odom";
	scan_message.header.stamp = ros::Time().now();
	scan_message.scale.x = 0.005;
	scan_message.scale.y = 0.005;
	scan_message.ns = "scans";
	scan_message.type = visualization_msgs::Marker::POINTS;
	scan_message.color.g = 1.0;
	scan_message.color.a = 1.0;
	//
	
	for(unsigned int i = 0; i < graph->node_list.size(); i++) {
		float theta = graph->node_list[i]->graph_pose.theta;
		//
		float minimal_angle = theta + graph->node_list[i]->laser_scan.angle_min;
		float current_angle = minimal_angle;
		float maximal_angle = theta + graph->node_list[i]->laser_scan.angle_max;
		//
		float angle_increment = graph->node_list[i]->laser_scan.angle_increment;
		float max_range = graph->node_list[i]->laser_scan.range_max;
		//
		for(unsigned int j = 0; current_angle <= maximal_angle - angle_increment && j < 180; j = j + 1, current_angle = current_angle + angle_increment) {
			float range = graph->node_list[i]->laser_scan.ranges[j];
			if(range == max_range){
				continue;
			}
			//
			float x = graph->node_list[i]->graph_pose.x + cos(current_angle) * range;
			float y = graph->node_list[i]->graph_pose.y + sin(current_angle) * range;
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

float GraphSlam::distance(float x1, float x2, float y1, float y2) {
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
;

float GraphSlam::rot_distance(float theta1, float theta2) {
	float rot_dist = theta1 - theta2;
    if (rot_dist >= PI) {
        rot_dist -= 2 * PI;
    } else if (rot_dist < -PI) {
        rot_dist += 2 * PI;
    }
	return rot_dist;
}
;

void GraphSlam::createTfFromXYTheta(double x, double y, double theta, tf::Transform& t) {
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}
;

int main(int argc, char **argv){
	ros::init(argc, argv, "GraphSlam");
	ros::NodeHandle n;
	GraphSlam g_slam(n);
	// ROS_INFO("INFO!");
	g_slam.spin(); // Execute FSM loop
	return 0;
}
;
