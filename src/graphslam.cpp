#include "graphslam.h"

using namespace std;
using namespace geometry_msgs;

//
GraphSlam::GraphSlam(ros::NodeHandle& nh) {
	// Subscribe to odom an laser scan messages
	laserScan_Sub = nh.subscribe("base_scan", 10, &GraphSlam::laserScan_callback, this);
	odometry_Sub = nh.subscribe("odom", 10, &GraphSlam::odom_callback, this);
	//
	map_publish = nh.advertise<nav_msgs::OccupancyGrid> ("/map", 1, false);
	pose_publish = nh.advertise<geometry_msgs::PoseArray>("/pose", 1);
	graph_publish = nh.advertise<visualization_msgs::Marker>("/graph_vis", 1);
	pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("/last_pose", 1);
	//
	nh.param("resolution", resolution, 0.05);
    nh.param("solve_iterations", solve_iterations, 10);
    nh.param("min_dist", min_dist, 0.25);
    nh.param("min_rot", min_rot, 0.3);
    nh.param("solve_after_nodes", solve_after_nodes, 10);
    nh.param("laser_range_t", range_t, 0.9);
	//
	odom_updated = false;
	scan_updated = false;
	first_scan = true;
	// Set the initial pose to 0,0,0
	cur_pose.position.x = 0.;
	cur_pose.position.y = 0.;
	cur_pose.orientation = tf::createQuaternionMsgFromYaw(0);
	//
	graph = new Graph(resolution, range_t);
	ROS_INFO("GraphSlam Constructor finished");
}
;

GraphSlam::~GraphSlam() {
	delete graph;
}
;

Pose GraphSlam::getFramePose(string frame, string fixed_frame, ros::Time stamp) {
	// Wait for the transform to be available
	tf_listener.waitForTransform(frame, fixed_frame, stamp, ros::Duration(1.0));
	tf::Pose pose;
	pose.setOrigin(tf::Vector3(0, 0, 0));
	pose.setRotation(tf::createQuaternionFromYaw(0));
	// Transform message
	geometry_msgs::PoseStamped pose_stamped;
	pose_stamped.header.frame_id = frame;
	pose_stamped.header.stamp = stamp;
	tf::poseTFToMsg(pose, pose_stamped.pose);
	// Transform
	geometry_msgs::PoseStamped result_msg;
	tf_listener.transformPose(fixed_frame, pose_stamped, result_msg);
	// Get the pose
	tf::Stamped<tf::Pose> tf_pose;
	tf::poseStampedMsgToTF(result_msg, tf_pose);

	Pose result;
	result.position.x = tf_pose.getOrigin().getX();
	result.position.y = tf_pose.getOrigin().getY();
	result.orientation = tf::createQuaternionMsgFromYaw(tf::getYaw(tf_pose.getRotation()));

	return result;
}

void GraphSlam::laserScan_callback(const sensor_msgs::LaserScan& msg){
	scan_updated = true;
	cur_scan = msg;
	// This means the robot is at the origin.
	if(!odom_updated && first_scan) {
		ROS_INFO("GraphSlam first scan");
		odom_updated = true;
	}
	first_scan = false;
}
;

void GraphSlam::odom_callback(const nav_msgs::Odometry& msg){
	if(scan_updated) {
		Pose new_pose = getFramePose("base_link", "odom", msg.header.stamp);
		float new_x = new_pose.position.x, new_y = new_pose.position.y;
		float distance = sqrt(pow(cur_pose.position.x - new_x, 2) + pow(cur_pose.position.y - new_y, 2));
		float rot_dist = abs(tf::getYaw(cur_pose.orientation) - tf::getYaw(msg.pose.pose.orientation));
		if (rot_dist > PI) {
			rot_dist = 2 * PI - rot_dist;
		}
		if(distance >= min_dist || rot_dist >= min_rot) {
			ROS_INFO("GraphSlam odom dist ok!");
			cur_pose = new_pose;
			odom_updated = true;
		}
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
			graph->addNode(cur_pose, cur_scan);
			//
			nav_msgs::OccupancyGrid cur_map;
			graph->generateMap(cur_map);
			//
			if(graph->node_list.size() > 2 && graph->node_list.size() % solve_after_nodes == 0)
				graph->solve(solve_iterations);
			// ROS_INFO("GraphSlam Map generated");
			map_publish.publish(cur_map);
			this->drawPoses();
			// ROS_INFO("GraphSlam Map published");
			// Call the graph-slam update here
			odom_updated = false;
			scan_updated = false;
			//
			GraphPose last_pose = graph->last_node->graph_pose;
			geometry_msgs::PoseStamped p;
			p.header.stamp = ros::Time().now();
  			p.header.frame_id = "/odom";
  			p.pose.position.x = last_pose.x;
  			p.pose.position.y = last_pose.y;
  			p.pose.orientation = tf::createQuaternionMsgFromYaw(last_pose.theta);
  			pose_publisher.publish(p);
  			ROS_INFO("Published last known pose: x: %f, y %f, t: %f", last_pose.x, last_pose.y, last_pose.theta);
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
	// Publish all poses in the graph!
	for(unsigned int i = 0; i < graph->node_list.size(); i++) {
		geometry_msgs::Pose new_pose;
		new_pose.position.x = graph->node_list[i].graph_pose.x;
		new_pose.position.y = graph->node_list[i].graph_pose.y;
		new_pose.orientation = tf::createQuaternionMsgFromYaw(graph->node_list[i].graph_pose.theta);
        poses.poses.push_back(new_pose);
	}
	pose_publish.publish(poses);

	// Publish the edges between all poses
	// ROS_INFO("Starting drawing edges.");
	for(unsigned int i = 0; i < graph->edge_list.size(); i++) {
		
		unsigned int node_list_size = graph->node_list.size();
		unsigned int edge_list_size = graph->edge_list.size();
		geometry_msgs::Point start;
		//
		GraphPose * pose;
		if(i > 0) { // First node has no parent
			unsigned int edge_parent_id = graph->edge_list[i].parent_id;
			if(i < (edge_list_size / 2.)){
				for(unsigned int k = 0; k < node_list_size; k++){
					if(graph->node_list[k].id == edge_parent_id){
						// ROS_INFO("MATCH PARENT");
						pose = &(graph->node_list[i].graph_pose);
					}
				}
			}else{
				for(int k = node_list_size - 1; k >= 0; k--){
					if(graph->node_list[k].id == edge_parent_id){
						// ROS_INFO("MATCH PARENT");
						pose = &(graph->node_list[i].graph_pose);
					}
				}
			}
			// ROS_INFO("PARENT X: %d, Y: %d", pose->x, pose->y);
			start.x = pose->x;
			start.y = pose->y;
		} else {
			start.x = 0;
			start.y = 0;
		}
		//
		geometry_msgs::Point end;
		unsigned int edge_child_id = graph->edge_list[i].child_id;
		if(i < (edge_list_size / 2.)){
			for(unsigned int k = 0; k < node_list_size; k++){
				if(graph->node_list[k].id == edge_child_id){
					// ROS_INFO("MATCH CHILD");
					pose = &(graph->node_list[i].graph_pose);
				}
			}
		}else{
			for(int k = node_list_size - 1; k >= 0; k--){
				if(graph->node_list[k].id == edge_child_id){
					// ROS_INFO("MATCH CHILD");
					pose = &(graph->node_list[i].graph_pose);
				}
			}
		}
		//
		// ROS_INFO("CHILD X: %d, Y: %d", pose->x, pose->y);
		end.x = pose->x;
		end.y = pose->y;
		//
		edges_message.points.push_back(start);
		edges_message.points.push_back(end);
		pose = 0;
	}
	// ROS_INFO("Finish drawing edges.");
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
		float theta = graph->node_list[i].graph_pose.theta;
		//
		float minimal_angle = theta + graph->node_list[i].laser_scan.angle_min;
		float current_angle = minimal_angle;
		float maximal_angle = theta + graph->node_list[i].laser_scan.angle_max;
		//
		float angle_increment = graph->node_list[i].laser_scan.angle_increment;
		float max_range = graph->node_list[i].laser_scan.range_max;
		//
		for(unsigned int j = 0; current_angle <= maximal_angle - angle_increment && j < 180; j = j + 1, current_angle = current_angle + angle_increment) {
			float range = graph->node_list[i].laser_scan.ranges[j];
			if(range == max_range){
				continue;
			}
			//
			float x = graph->node_list[i].graph_pose.x + cos(current_angle) * range;
			float y = graph->node_list[i].graph_pose.y + sin(current_angle) * range;
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
