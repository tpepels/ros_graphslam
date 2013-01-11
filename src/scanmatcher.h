#ifndef SCANMATCHER_H
#define SCANMATCHER_H
#include "graph.h"
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
#include <csm/csm_all.h>
#include <gsl/gsl_matrix.h> //Hier zit covariance in

#define PI 3.14159265359

using namespace std;

class ScanMatcher {
 public:
  ScanMatcher();
  bool scanMatch(sensor_msgs::LaserScan& scan, ros::Time time, double mean[], double covariance[][3]);
 private:
  sm_params input;
  sm_result output;
  //
  LDP previous_ldp;
  //Pose of the base frame in fixed frame;
  tf::Transform fixed_to_base;
  //Pose of the last keyframe scan in fixed frame
  tf::Transform fixed_to_base_keyframe;
  //Used to convert poses between base of the robot and the laser scanner.
  tf::Transform laser_to_base;
  tf::Transform base_to_laser;
  //
  ros::Time last_time;
  bool initialized;
  tf::TransformListener tf_listener;
  string base_frame, fixed_frame;
  double keyframe_distance_linear, keyframe_distance_angular;

  bool setBasetoLaserTransform(const std::string& frame_id);
  void convertScantoDLP(sensor_msgs::LaserScan& scan, LDP& ldp);
  bool newKF(const tf::Transform& d);
  bool processScan(LDP& ldp, ros::Time time, double change_x, double change_y, double change_theta, double mean[], double covariance[][3]);
};
#endif