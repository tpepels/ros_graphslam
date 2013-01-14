#ifndef SCANMATCHER_H
#define SCANMATCHER_H
#include "ros/ros.h"
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
  bool scanMatch(sensor_msgs::LaserScan& scan_to_match, sensor_msgs::LaserScan& reference_scan, double change_x, double change_y, double change_theta, double mean[3], double covariance[][3]);
 private:
  sm_params input;
  sm_result output;
  //
  void convertScantoDLP(sensor_msgs::LaserScan& scan, LDP& ldp);
  bool processScan(LDP& ldp, LDP& ref_ldp, double change_x, double change_y, double change_theta, double mean[], double covariance[][3]);
};
#endif