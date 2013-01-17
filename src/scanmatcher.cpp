#include "scanmatcher.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <math.h>

ScanMatcher::ScanMatcher() {
  //
  input.laser[0] = 0.0;
  input.laser[1] = 0.0;
  input.laser[2] = 0.0;
  // General input parameters
  input.sigma = 0.01;
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
  ldp->max_theta = ldp->theta[numberOfScans - 1];
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

bool ScanMatcher::processScan(LDP& ldp, LDP& ref_ldp, double change_x, double change_y, double change_theta, double mean[], double covariance[][3]){
  input.laser_ref = ref_ldp;
  input.laser_sens = ldp;
  // ROS_INFO("SM change_x: %f, change_y: %f, change_t: %f", change_x, change_y, change_theta);
  //Set initial estimate of input
  input.first_guess[0] = change_x;
  input.first_guess[1] = change_y;
  input.first_guess[2] = change_theta; 
  // ROS_INFO("SM first_guess_x: %f, first_guess_y: %f, change_t: %f", input.first_guess[0], input.first_guess[1], input.first_guess[2]);
  //Finally, perform scan matching.
  sm_icp(&input, &output);
  //
  if(output.valid){
    tf::Transform output_t;
    createTfFromXYTheta(output.x[0], output.x[1], output.x[2], output_t);
    new_pose_t = ref_pose_t * output_t;
    //Set mean pose
    mean[0] = new_pose_t.getOrigin().getX();
    mean[1] = new_pose_t.getOrigin().getY();
    mean[2] = tf::getYaw(new_pose_t.getRotation());
    // ROS_INFO("SM mean_x: %2.4f, mean_y: %2.4f, mean_t: %2.4f, error: %2.4f", mean[0], mean[1], mean[2], output.error);
    //
    if(input.do_compute_covariance == 1) {
      //Set covariance
      unsigned int rows = output.cov_x_m->size1, cols = output.cov_x_m->size2;
      for(unsigned int i = 0; i < cols; i++) {
        for(unsigned int j = 0; j < rows; j++) {
          covariance[i][j] = gsl_matrix_get(output.cov_x_m, i, j);
          // ROS_INFO("Covariance[%d]: %f", i+j, covariance[i][j]);
        }
      }
    }
  } else {
    ROS_WARN("Solution was not found.");
  }
  //
  ld_free(ref_ldp);
  ld_free(ldp);
  //Return
  if(output.valid) {
    return true;
  } else {
    return false;
  }
}
;

void ScanMatcher::createTfFromXYTheta(double x, double y, double theta, tf::Transform& t) {
  t.setOrigin(tf::Vector3(x, y, 0.0));
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, theta);
  t.setRotation(q);
}
;

bool ScanMatcher::graphScanMatch(LaserScan& scan_to_match, GraphPose& new_pose, LaserScan& reference_scan, GraphPose& ref_pose, double mean[3], double covariance[][3]) {
  LDP ref_ldp;
  convertScantoDLP(reference_scan, ref_ldp);
  LDP current_ldp;
  convertScantoDLP(scan_to_match, current_ldp);
  // Transforms for the new pose and reference pose
  createTfFromXYTheta(new_pose.x, new_pose.y, new_pose.theta, new_pose_t);
  createTfFromXYTheta(ref_pose.x, ref_pose.y, ref_pose.theta, ref_pose_t);
  // All scans should be between this interval
  input.min_reading = scan_to_match.range_min;
  input.max_reading = scan_to_match.range_max;
  // Allow more distance grom the solution as the scan-matching distance is higher
  input.max_iterations = 20;
  input.epsilon_xy = 0.0001;
  input.epsilon_theta = 0.0001;
  input.do_compute_covariance = 1;
  input.max_angular_correction_deg = 90.0;
  input.max_linear_correction = 0.5;
  input.max_correspondence_dist = 0.5;
  //Calculate change in position
  double dx = new_pose.x - ref_pose.x;
  double dy = new_pose.y - ref_pose.y;
  double dt = new_pose.theta - ref_pose.theta;
  //
  if (dt >= PI) {
      dt -= 2 * PI;
  } else if (dt < -PI) {
      dt += 2 * PI;
  }
  bool result = processScan(current_ldp, ref_ldp, dx, dy, dt, mean, covariance);
  return result;
};

bool ScanMatcher::scanMatch(LaserScan& scan_to_match, GraphPose& new_pose, LaserScan& reference_scan, GraphPose& ref_pose, double mean[3]) {
  LDP ref_ldp;
  convertScantoDLP(reference_scan, ref_ldp);
  LDP current_ldp;
  convertScantoDLP(scan_to_match, current_ldp);
  // Transforms for the new pose and reference pose
  createTfFromXYTheta(new_pose.x, new_pose.y, new_pose.theta, new_pose_t);
  createTfFromXYTheta(ref_pose.x, ref_pose.y, ref_pose.theta, ref_pose_t);
  // All scans should be between this interval
  input.min_reading = scan_to_match.range_min;
  input.max_reading = scan_to_match.range_max;
  input.max_iterations = 10;
  input.epsilon_xy = 0.000001;
  input.epsilon_theta = 0.000001;
  input.do_compute_covariance = 0;
  input.max_angular_correction_deg = 30.0;
  input.max_linear_correction = 0.2;
  input.max_correspondence_dist = 0.3;
  //
  //Calculate change in position
  double dx = new_pose.x - ref_pose.x;
  double dy = new_pose.y - ref_pose.y;
  double dt = new_pose.theta - ref_pose.theta;
  //
  if (dt >= PI) {
      dt -= 2 * PI;
  } else if (dt < -PI) {
      dt += 2 * PI;
  }
  double covariance[3][3];
  bool result = processScan(current_ldp, ref_ldp, dx, dy, dt, mean, covariance);
  return result;
};