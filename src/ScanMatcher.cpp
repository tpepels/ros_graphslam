#include "ScanMatcher.h"

ScanMatcher::ScanMatcher() {
  // map params. TODO: these should be changed to our map parameters
  map_resolution = 0.02;
  

  grid_dim_x = 40;
  grid_dim_y = 25;
  grid_dim_theta = 30;

  grid_res_xy = map_resolution;
  grid_res_theta = 0.03;
  
  map_margin_buffer = 2.0; // meters
  motion_model_weight = 0.0;
  
  //pose_grid = new PoseGrid2(grid_dim_theta, grid_dim_x, grid_dim_y);
  //pose_grid->setResolution(grid_res_xy, grid_res_theta);
}

