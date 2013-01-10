//
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

using namespace std;

class ScanMatcher {
 public:
  ScanMatcher();
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

  bool setBasetoLaserTransform(const std::string& frame_id);
  void convertScantoDLP(const sensor_msgs::LaserScan::ConstPtr& scan, LDP& ldp);
  void processScan(LDP& ldp, ros::Time time);
  void scanMatch(const sensor_msgs::LaserScan::ConstPtr& scan, ros::Time time);
};
