#include "ScanMatcher.h"
#include <iostream>
#include <stdlib.h>
#include <cmath>
#include <math.h>

// Used for simulateRangeScan
#include "occupancy_grid_utils/ray_tracer.h"


float ranf(){         /* ranf() is uniform in 0..1 */
return (rand() / double(RAND_MAX));
}

float box_muller(float m, float s)	/* normal random variate generator */
{				        /* mean m, standard deviation s */
	float x1, x2, w, y1;
	static float y2;
	static int use_last = 0;

	if (use_last)		        /* use value from previous call */
	{
		y1 = y2;
		use_last = 0;
	}
	else
	{
		do {
			x1 = 2.0 * ranf() - 1.0;
			x2 = 2.0 * ranf() - 1.0;
			w = x1 * x1 + x2 * x2;
		} while ( w >= 1.0 );

		w = sqrt( (-2.0 * log( w ) ) / w );
		y1 = x1 * w;
		y2 = x2 * w;
		use_last = 1;
	}

	return( m + y1 * s );
}


ScanMatcher::ScanMatcher() {
 
  const int particleCount = 100;
  //initialize particles used in monte carlo 
  particleCloud.poses.resize( particleCount );
  // map params. TODO: these should be changed to our map parameters

 /*
  map_resolution = 0.02;
  

  grid_dim_x = 40;
  grid_dim_y = 25;
  grid_dim_theta = 30;

  grid_res_xy = map_resolution;
  grid_res_theta = 0.03;
  
  map_margin_buffer = 2.0; // meters
  motion_model_weight = 0.0;
  */  

  //pose_grid = new PoseGrid2(grid_dim_theta, grid_dim_x, grid_dim_y);
  //pose_grid->setResolution(grid_res_xy, grid_res_theta);
}


void ScanMatcher::initialize( Pose initialpose){

    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      //particles with large variance centered on initial pose
      particleCloud.poses[i].position.x = initialpose.position.x + box_muller(0,1.0);
      particleCloud.poses[i].position.y = initialpose.position.y + box_muller(0,1.0);
      particleCloud.poses[i].orientation = initialpose.orientation;
      double yaw_angle = tf::getYaw(initialpose.orientation);//Get initial rotation
      rotation[i] = yaw_angle;
    }

}


//This motion is applied to the current position of the particles
void ScanMatcher::applyMotionModel( double deltaX, double deltaY, double deltaT ){

    if (deltaX > 0 or deltaY > 0)
      ROS_DEBUG( "applying odometry: %f %f %f", deltaX, deltaY, deltaT );

    //float aValue[4] = {0.0,0.0,0.0,0.0}; //the a values used in the odometry model
    double variance_odom = 1.0;
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      particleCloud.poses[i].position.x += box_muller(deltaX,variance_odom*deltaX);
      particleCloud.poses[i].position.y +=  box_muller(deltaY,variance_odom*deltaY); 
      rotation[i] += deltaT;
      geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(rotation[i]);   
      particleCloud.poses[i].orientation = odom_quat;   
      //particleCloud.poses[i].orientation.w += deltaT; 
      //particleCloud.poses[i].orientation.normalize(); 
    }

}


//applying motion model where each particle starts at position 1. And goes to position 2
void ScanMatcher::applyMotionModel( Pose position1, Pose position2 ){


}


//Scan matching with a beam based model
double ScanMatcher::scanMatch(
    const sensor_msgs::LaserScan& scan,
    const nav_msgs::OccupancyGrid& map,
    const Pose sensor_pose   ){

    /* This method is the beginning of an implementation of a beam
     * sensor model */   
    for (unsigned int i = 0; i < particleCloud.poses.size(); ++i)
    {
      geometry_msgs::Pose sensor_pose;      
      sensor_pose =  particleCloud.poses[i];

      /* If the laser and centre of the robot weren't at the same
       * position, we would first apply the tf from /base_footprint
       * to /base_laser here. */
      sensor_msgs::LaserScan::Ptr simulatedScan;
      try{
        simulatedScan
          = occupancy_grid_utils::simulateRangeScan
          ( map, sensor_pose, scan, true );
      }
      catch (occupancy_grid_utils::PointOutOfBoundsException)
      {
        continue;
      }
     }


 return 0.0;

}
