#ifndef SCANMATCHER_H
#define SCANMATCHER_H
#include "ros/ros.h"
#include "graphnodes.h"
//
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Pose.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseStamped.h"
//
#include <Eigen/Core>
//
#include <csm/csm_all.h>
#include <gsl/gsl_matrix.h> //Hier zit covariance in

#ifndef PI
#define PI 3.14159265359
#endif

using namespace std;

class ScanMatcher {
 public:
  ScanMatcher();
  bool scanMatch(sensor_msgs::LaserScan& scan_to_match, GraphPose& new_pose, sensor_msgs::LaserScan& reference_scan, GraphPose& ref_pose,  double change_x, double change_y, double change_theta, double mean[3], double covariance[][3]);
 private:
  sm_params input;
  sm_result output;
  tf::Transform new_pose_t, ref_pose_t;
  //
  void createTfFromXYTheta(double x, double y, double theta, tf::Transform& t);
  void convertScantoDLP(sensor_msgs::LaserScan& scan, LDP& ldp);
  bool processScan(LDP& ldp, LDP& ref_ldp, double change_x, double change_y, double change_theta, double mean[], double covariance[][3]);
};
#endif