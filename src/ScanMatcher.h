#ifndef SCANMATCHER_H
#define SCANMATCHER_H


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

using namespace std;
using namespace geometry_msgs;

//scan matching with a beam based sensor model
class ScanMatcher {

 public:
  ScanMatcher();
  void initialize( Pose initialpose);
  //void setReferenceNodes(vector<GraphNode*> reference_nodes, GraphNode* fixed_node);
  double scanMatch(
    const sensor_msgs::LaserScan& scan,
    const nav_msgs::OccupancyGrid& map,
    const Pose sensor_pose   ); // returns score of max_pose

  //Applying motion model based on the difference in positions of the nodes.
  //This motion is applied to the current position of the particles
  void applyMotionModel( double deltaX, double deltaY, double deltaT );
  //applying motion model where each particle starts at position 1. And goes to position 2
  void applyMotionModel( Pose position1, Pose position2 );

  void getMean(Pose* mean);
  void getCovariance(double* cov);
  Pose getMaxPose();

  private:
  geometry_msgs::PoseArray particleCloud;
  double rotation[100];

};


#endif
