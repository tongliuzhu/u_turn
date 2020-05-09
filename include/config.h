#pragma once
#include <math.h>
/**
 * @brief Config param for quick demo usage
 *
 * TODO refactor and move config data to config files
 */

const double kMeter2Pixel = 20.;
const bool kUsingDPMap = false;

struct MapBoundaryConfig {
  const double map_width; // meter
  double map_height; // meter defined by lane width and lane gap; 2*lane_width +
                     // lane_gap
  const double lane_width; // meter
  const double lane_gap;   // meter

  MapBoundaryConfig(double map_wid = 50., double lane_wid = 4.,
                    double lane_gap = 1.)
      : map_width(map_wid), lane_width(lane_wid), lane_gap(lane_gap) {
    map_height = 2. * lane_width + lane_gap;
  }
};

struct HybridAstarConfig {
  double xy_grid_resolution = 0.2;
  double phi_grid_resolution = 5. * M_PI / 180.;
  int next_node_num = 10;
  double step_size = 0.25;
  double traj_forward_penalty = 1.;
  double traj_back_penalty = 3.;
  double traj_gear_switch_penalty = 3.;
  double traj_steer_penalty = 0.6;
  double traj_steer_change_penalty = 0.2;
  double node_radius = 0.1; // collision margin meter
  double delta_t = 0.1;     // sec
  HybridAstarConfig() {}
};

struct VehicleParam {
  double length;
  double width;
  double back_to_gear;
  double max_steer_angle;
  double steer_ratio;
  double wheel_base;
  VehicleParam(double len = 4.7, double wid = 2.0, double b2g = 1.0,
               double msa = 540.0 * M_PI / 180., double sr = 18.,
               double wb = 3.0)
      : length(len), width(wid), back_to_gear(b2g), max_steer_angle(msa),
        steer_ratio(sr), wheel_base(wb) {}
};
