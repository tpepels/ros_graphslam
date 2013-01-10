#include "scanmatcher.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <math.h>

ScanMatcher::ScanMatcher() {
  fixed_to_base.setIdentity();
  fixed_to_base_keyframe.setIdentity();
  //
  input.laser[0] = 0.0;
  input.laser[1] = 0.0;
  input.laser[2] = 0.0;
  //
  input.max_angular_correction_deg = 45.0;
  input.max_linear_correction = 0.50;
  input.max_iterations = 10;
  input.epsilon_xy = 0.000001;
  input.epsilon_theta = 0.000001;
  input.max_correspondence_dist = 0.3;
  input.sigma = 0.010;
  input.use_corr_tricks = 1;
  input.restart = 0;
  input.restart_threshold_mean_error = 0.01;
  input.restart_dt = 1.0;
  input.restart_dtheta = 0.1;
  input.clustering_threshold = 0.25;
  input.orientation_neighbourhood = 20;
  input.use_point_to_line_distance = 1;
  input.do_alpha_test = 0;
  input.do_alpha_test_thresholdDeg = 20.0;
  input.outliers_maxPerc = 0.90;
  input.outliers_adaptive_order = 0.7;
  input.outliers_adaptive_mult = 2.0;
  input.do_visibility_test = 0;
  input.outliers_remove_doubles = 1;
  input.do_compute_covariance = 1;
  input.debug_verify_tricks = 0;
  input.use_ml_weights = 0;
  input.use_sigma_weights = 0;
};

void ScanMatcher::convertScantoDLP(const sensor_msgs::LaserScan::ConstPtr& scan, LDP& ldp){
  unsigned int numberOfScans = scan->ranges.size();
  ldp = ld_alloc_new(numberOfScans);
  //
  for(unsigned int i = 0; i < numberOfScans; i++){
    //Set range to -1 if if it exceeds the bounds of the laser scanner.
    double range = scan->ranges[i];
    if(range > scan->range_min && range < scan->range_max){
      ldp->valid[i] = 1;
      ldp->readings[i] = range;
    }else{
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;
    }
    //Set angle
    ldp->theta[i] = scan->angle_min + i * scan->angle_increment;
    ldp->cluster[i] = -1;
  }
  //
  ldp->min_theta = ldp->theta[0];
  ldp->max_theta = ldp->theta[numberOfScans-1];
  //
  ldp->odometry[0] = 0.0;
  ldp->odometry[1] = 0.0;
  ldp->odometry[2] = 0.0;
  //
  ldp->true_pose[0] = 0.0;
  ldp->true_pose[1] = 0.0;
  ldp->true_pose[2] = 0.0;
};

void ScanMatcher::processScan(LDP& ldp, ros::Time& time){
  //Reset variables of previous_ldp
  previous_ldp->odometry[0] = 0.0;
  previous_ldp->odometry[1] = 0.0;
  previous_ldp->odometry[2] = 0.0;
  //
  previous_ldp->estimate[0] = 0.0;
  previous_ldp->estimate[1] = 0.0;
  previous_ldp->estimate[2] = 0.0;
  //
  previous_ldp->true_pose[0] = 0.0;
  previous_ldp->true_pose[1] = 0.0;
  previous_ldp->true_pose[2] = 0.0;
  //
  input.laser_ref = previous_ldp;
  input.laser_sens = ldp;
  //Estimate the change in pose since the last scan.
  double deltaT = (time - last_time).toSec();
  double change_x, change_y, change_theta;
  estimatePoseChange(change_x, change_y, change_theta);
  //Create TF from this estimate
  tf::Transform& change;
  change.setOrigin(tf::Vector3(change_x, change_y, 0.0));
  tf::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, change_theta);
  change.setRotation(quaternion);
  //Change since the last keyframe, in the fixed frame.
  change = change * (fixed_to_base * fixed_to_base_keyframe.inverse());
  //
  
};

void ScanMatcher::scanMatch(const sensor_msgs::LaserScan::ConstPtr& scan, ros::Time& time){
  last_time = scan->header.stamp;
  convertScantoDLP(scan, previous_ldp);
  processScan(previous_ldp, scan->header.stamp);
};