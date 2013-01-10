#include "scanmatcher.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <math.h>

ScanMatcher::ScanMatcher(){
  fixed_to_base.setIdentity();
  fixed_to_base_keyframe.setIdentity;
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
  input.restart_treshold_mean_error = 0.01;
  input.restart_dt = 1.0;
  input.resstart_dtheta = 0.1;
  input.clustering_treshold = 0.25;
  input.orientation_neighbourhood = 20;
  input.use_point_to_line_distance = 1;
  input.do_alpha_test = 0;
  input.do_alpah_test_thresholdDeg = 20.0;
  input.outliers_maxPerc = 0.90;
  input.outliers_adaptive_order = 0.7;
  input.outliers_adaptive_mult = 2.0;
  input.do_visibility_test = 0;
  input.outliers_remove_doubles = 1;
  input.do_compute_covariance = 1;
  input.debug_verify_tricks = 0;
  input..use_ml_weights = 0;
  input.use_sigma_weights = 0;
};

void ScanMatcher::convertScantoDLP(const sensor_msgs::LaserScan::ConstPtr& scan, LDP& ldp){
  unsigned int numberOfScans = scan->ranges.size();
  ldp = ld_alloc_new(numberOfScans);
  //
  for(unsigned int i = 0; i < numberOfScans; i++){
    //Set range to -1 if if it exceeds the bounds of the laser scanner.
    double range = scan->ranges[i];
    if(r > scan->range_min && r < scan->range_max){
      ldp->valid[i] = 1;
      ldp->readings[i] = r;
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
  ldp->max_theta = ldp->theta[n-1];
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

};

void ScanMatcher::scanMatch(const sensor_msgs::LaserScan::ConstPtr& scan, ros::Time& time){
  convertScantoDLP(scan, previous_ldp);
  processScan(previous_ldp, scan->header.stamp);
};