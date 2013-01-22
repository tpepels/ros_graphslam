#include "sm_node.h"
//
SMNode::SMNode(ros::NodeHandle& nh) {
	// Subscribe to odom an laser scan messages
	laserScan_Sub = nh.subscribe("base_scan", 1, &SMNode::laserScan_callback, this);
	odometry_Sub = nh.subscribe("odom", 1, &SMNode::odom_callback, this);
	//
	pose_publisher = nh.advertise<PoseStamped>("/last_pose", 1);
	//
	first_scan = true;
	// Set the initial pose for the graph and scanmatcher to 0,0,0
	cur_sm_pose.x = 0.;
	cur_sm_pose.y = 0.;
	cur_sm_pose.theta = 0;
	//
	prev_sm_odom.position.x = 0;
	prev_sm_odom.position.y = 0;
	prev_sm_odom.orientation = tf::createQuaternionMsgFromYaw(0);
}
;

void SMNode::laserScan_callback(const LaserScan::ConstPtr& msg){	
	// Check if we will perform scanmatching
	if(!first_scan && sm_odom_updated) {
		//ROS_INFO("Graphslam scanmatching!");
		double mean[3], error;
		LaserScan scan = *msg, ref_scan = *cur_sm_scan;		
		bool result = matcher.scanMatch(scan, cur_sm_pose, ref_scan, prev_sm_pose, mean, error);
		// ROS_INFO("SM Error %f", error);
		if(result) {
			// ROS_INFO("Estimated pose: x %f y: %f t: %f", cur_sm_pose.x, cur_sm_pose.y, cur_sm_pose.theta);
			// ROS_INFO("ScanMatched pose: x %f y: %f t: %f", mean[0], mean[1], mean[2]);
			cur_sm_pose.x = mean[0];
			cur_sm_pose.y = mean[1];
			cur_sm_pose.theta = mean[2];
		} else {
			// In this case, cur_sm_pose will have the value updated by the odometry
			// ROS_WARN("Scanmatching in GraphSlam failed.");
			ROS_WARN("Using estimated pose: x %f y: %f t: %f", cur_sm_pose.x, cur_sm_pose.y, cur_sm_pose.theta);
		}
		// Check if we should update the reference scan to a new frame
		if(distance(cur_sm_pose.x, prev_sm_pose.x, cur_sm_pose.y, prev_sm_pose.y) > 0.1 
			|| abs(rot_distance(cur_sm_pose.theta, prev_sm_pose.theta)) > 0.1) {
			prev_sm_pose = cur_sm_pose;
			cur_sm_scan = msg;
		}
		sm_odom_updated = false;
		sm_pose_updated = true;
	}
	// Store the first reference scan for later use
	if(first_scan) {
		first_scan = false;
		cur_sm_scan = msg;
	}
}
;

void SMNode::odom_callback(const nav_msgs::Odometry& msg){
	float new_x = msg.pose.pose.position.x, new_y = msg.pose.pose.position.y;
	float prev_theta = tf::getYaw(prev_sm_odom.orientation);
	float sm_dist = distance(prev_sm_odom.position.x, new_x, prev_sm_odom.position.y, new_y);
	// Apply the odometry motion model to get an initial estimate of the new position
	float drot1 = atan2(new_y - prev_sm_odom.position.y, new_x - prev_sm_odom.position.x) - prev_theta;
	float drot2 = tf::getYaw(msg.pose.pose.orientation) - prev_theta - drot1;
	//
	cur_sm_pose.x += sm_dist * cos(prev_theta + drot1);
	cur_sm_pose.y += sm_dist * sin(prev_theta + drot1);
	cur_sm_pose.theta += drot1 + drot2;
	// Ready to do some scanmatching!
	prev_sm_odom = msg.pose.pose;
	sm_odom_updated = true;
}
;

float SMNode::distance(float x1, float x2, float y1, float y2) {
	return sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
}
;

float SMNode::rot_distance(float theta1, float theta2) {
	float rot_dist = theta1 - theta2;
    if (rot_dist >= PI) {
        rot_dist -= 2 * PI;
    } else if (rot_dist < -PI) {
        rot_dist += 2 * PI;
    }
	return rot_dist;
}
;

void SMNode::spin() {
	//ros::Rate rate(100); // Specify the FSM loop rate in Hz
	while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
		ros::spinOnce(); // Need to call this function often to allow ROS to process incoming messages
		if(sm_pose_updated) {
			// Publish the lase scanmatching pose
			PoseStamped p;
			p.header.stamp = ros::Time().now();
			p.header.frame_id = "/odom";
			p.pose.position.x = cur_sm_pose.x;
			p.pose.position.y = cur_sm_pose.y;
			p.pose.orientation = tf::createQuaternionMsgFromYaw(cur_sm_pose.theta);
			pose_publisher.publish(p);
			sm_pose_updated = false;
		}
		//
		//rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
	}
}
;

int main(int argc, char **argv){
	ros::init(argc, argv, "sm_node");
	ros::NodeHandle n;
	SMNode sm_node(n);
	sm_node.spin(); // Execute FSM loop
	return 0;
}
;