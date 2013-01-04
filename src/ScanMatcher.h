#ifndef SCANMATCHER_H
#define SCANMATCHER_H


class SensorModel {

 public:
  ScanMatcher();
  void initialize(ros::NodeHandle *nh, tf::TransformListener *listener);
  void setReferenceNodes(vector<GraphNode*> reference_nodes, GraphNode* fixed_node);
  double scanMatch(GraphNode* test_node, bool use_odom=true, bool visualize=true); // returns score of max_pose
  void getMean(RobotPose* mean);
  void getCovariance(double* cov);
  RobotPose getMaxPose();

}


#endif
