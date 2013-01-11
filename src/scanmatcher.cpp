#include "scanmatcher.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <math.h>

ScanMatcher::ScanMatcher() {
  initialized = false;
  //
  base_frame = "base_link";
  fixed_frame = "odom";
  keyframe_distance_linear = 0.10;
  keyframe_distance_angular = 10.0 * (PI / 180.0);
  //
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
}
;

bool ScanMatcher::setBasetoLaserTransform(const std::string& frame_id){
  ros::Time t = ros::Time::now();

  tf::StampedTransform base_to_laser_tf;
  try
  {
    tf_listener.waitForTransform(
      base_frame, frame_id, t, ros::Duration(1.0));
    tf_listener.lookupTransform (
      base_frame, frame_id, t, base_to_laser_tf);
  }
  catch (tf::TransformException ex)
  {
    ROS_WARN("Could not get initial transform, %s", ex.what());
    return false;
  }
  base_to_laser = base_to_laser_tf;
  laser_to_base = base_to_laser.inverse();

  return true;
};

void ScanMatcher::convertScantoDLP(sensor_msgs::LaserScan& scan, LDP& ldp){
  unsigned int numberOfScans = scan.ranges.size();
  ldp = ld_alloc_new(numberOfScans);
  //
  for(unsigned int i = 0; i < numberOfScans; i++) {
    //Set range to -1 if if it exceeds the bounds of the laser scanner.
    double range = scan.ranges[i];
    if(range > scan.range_min && range < scan.range_max) {
      ldp->valid[i] = 1;
      ldp->readings[i] = range;
    } else {
      ldp->valid[i] = 0;
      ldp->readings[i] = -1;
    }
    //Set angle
    ldp->theta[i] = scan.angle_min + i * scan.angle_increment;
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
}
;

bool ScanMatcher::newKF(const tf::Transform& transform){
  if (tf::getYaw(transform.getRotation()) > keyframe_distance_angular){
    return true;
  }
  //
  double x = transform.getOrigin().getX();
  double y = transform.getOrigin().getY();
  //
  if (pow(x, 2) + pow(y, 2) > pow(keyframe_distance_linear, 2)){
    return true;
  }
  return false;
};

bool ScanMatcher::processScan(LDP& ldp, ros::Time time, double change_x, double change_y, double change_theta, double mean[], Matrix3d covariance){
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
  //Create TF from the odometry pose estimate
  tf::Transform change;
  change.setOrigin(tf::Vector3(change_x, change_y, 0.0));
  tf::Quaternion quaternion;
  quaternion.setRPY(0.0, 0.0, change_theta);
  change.setRotation(quaternion);
  //Change since the last keyframe, in the fixed frame.
  change = change * (fixed_to_base * fixed_to_base_keyframe.inverse());
  //Change of the laser's position in the fixed frame
  tf::Transform change_laser;
  change_laser = laser_to_base * fixed_to_base.inverse() * change * fixed_to_base * base_to_laser;
  //Almost done, set initial estimate of input
  input.first_guess[0] = change_laser.getOrigin().getX();
  input.first_guess[1] = change_laser.getOrigin().getY();
  input.first_guess[2] = tf::getYaw(change_laser.getRotation());
  //Finally, perform scan matching.
  sm_icp(&input, &output);
  //
  tf::Transform correct_change;
  if(output.valid){
    //Get the result in a transform.
    tf::Transform correct_change_laser;
    correct_change_laser.setOrigin(tf::Vector3(change_x, change_y, 0.0));
    tf::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, change_theta);
    correct_change_laser.setRotation(quaternion);
    //Calculate change from this transform
    correct_change = base_to_laser * correct_change_laser * laser_to_base;
    //Update pose in fixed frame
    fixed_to_base = fixed_to_base_keyframe * correct_change;
    //Set mean pose
    mean[0] = fixed_to_base.getOrigin().getX();
    mean[1] = fixed_to_base.getOrigin().getY();
    mean[2] = tf::getYaw(fixed_to_base.getRotation());
    //Set covariance
    unsigned int rows = output.cov_x_m->size1, cols = output.cov_x_m->size2;
    for(unsigned int i = 0; i < rows; i++){
      for(unsigned int j = 0; j < cols; j++){
        covariance(i, j) = gsl_matrix_get(output.cov_x_m, i, j);
      }
    }
  }else{
    correct_change.setIdentity();
    ROS_WARN("Solution was not found.");
  }
  //Update variables
  if(newKF(correct_change)){
    ld_free(previous_ldp);
    previous_ldp = ldp;
    fixed_to_base_keyframe = fixed_to_base;
  }else{
    ld_free(ldp);
  }
  last_time = time;
  //Return
  if(output.valid){
    return true;
  }else{
    return false;
  }
};

bool ScanMatcher::scanMatch(sensor_msgs::LaserScan& scan_to_match, sensor_msgs::LaserScan& reference_scan, ros::Time time, double change_x, double change_y, double change_theta, double mean[3], Matrix3d covariance){
  if(!initialized){
    while(!setBasetoLaserTransform(scan_to_match.header.frame_id))
    {
      ROS_WARN("Error while waiting for transform.");
      return false;
    }  
    initialized = true;
  }
  convertScantoDLP(reference_scan, previous_ldp);
  last_time = scan_to_match.header.stamp;
  LDP current_ldp;
  convertScantoDLP(scan_to_match, current_ldp);
  bool result = processScan(current_ldp, scan_to_match.header.stamp, change_x, change_y, change_theta, mean, covariance);
  return result;
};
