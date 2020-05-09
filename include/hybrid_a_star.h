#pragma once

#include <algorithm>
#include <chrono>
#include <iterator>
#include <memory>
#include <queue>
#include <string>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "grid_search.h"
#include "node3d.h"
#include "reeds_shepp_path.h"
#include "vec2d.h"

struct HybridAStartResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> kappa;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
};

/**
 * @class HybridAStar
 *
 * @brief Implements a class of HybridAStar search.
 */
class HybridAStar {
public:
  explicit HybridAStar(const HybridAstarConfig &open_space_conf);
  virtual ~HybridAStar() = default;
  // Hybrid a star search
  bool Plan(double sx, double sy, double sphi, double ex, double ey,
            double ephi, const std::vector<double> &XYbounds,
            std::vector<ObstacleInfo> &ob_vec, HybridAStartResult *result);
  // Uturn plan method
  void UturnPlan(const HybridAStartResult &start_lane_center,
                 const HybridAStartResult &target_lane_center,
                 const std::vector<double> &XYbounds,
                 std::vector<ObstacleInfo> &ob_vec, HybridAStartResult *result);
  bool TrajectoryPartition(const HybridAStartResult &result,
                           std::vector<HybridAStartResult> *partitioned_result);

private:
  // RS path direct search solution
  bool AnalyticExpansion(std::shared_ptr<Node3d> current_node);
  // check collision and validity
  bool ValidityCheck(std::shared_ptr<Node3d> node);
  // check Reeds Shepp path collision and validity
  bool RSPCheck(const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end);
  // load the whole RSP as nodes and add to the close set
  std::shared_ptr<Node3d>
  LoadRSPinCS(const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
              std::shared_ptr<Node3d> current_node);
  std::shared_ptr<Node3d>
  Next_node_generator(std::shared_ptr<Node3d> current_node,
                      size_t next_node_index);
  void CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                         std::shared_ptr<Node3d> next_node);
  double TrajCost(std::shared_ptr<Node3d> current_node,
                  std::shared_ptr<Node3d> next_node);
  double HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node);
  bool GetResult(HybridAStartResult *result);
  bool GetTemporalProfile(HybridAStartResult *result);
  bool GenerateSpeedAcceleration(HybridAStartResult *result);
  double Distance(const ObstacleInfo &ob, const double &x, const double &y,
                  const double &phi) const;
  bool CollisionDetection(const std::vector<ObstacleInfo> &ob_vec,
                          const double &x, const double &y,
                          const double &phi) const;
  bool OutOfBoundaryCheck(const double &x, const double &y,
                          const double &phi) const;

private:
  HybridAstarConfig hybrid_astar_config_;
  VehicleParam vehicle_param_;
  size_t next_node_num_ = 0;
  double max_wheel_angle_ = 0.0;
  double step_size_ = 0.0;
  double xy_grid_resolution_ = 0.0;
  double delta_t_ = 0.0;
  double traj_forward_penalty_ = 0.0;
  double traj_back_penalty_ = 0.0;
  double traj_gear_switch_penalty_ = 0.0;
  double traj_steer_penalty_ = 0.0;
  double traj_steer_change_penalty_ = 0.0;
  double heu_rs_forward_penalty_ = 0.0;
  double heu_rs_back_penalty_ = 0.0;
  double heu_rs_gear_switch_penalty_ = 0.0;
  double heu_rs_steer_penalty_ = 0.0;
  double heu_rs_steer_change_penalty_ = 0.0;
  double collision_margin_ = 0.2; // meter
  std::vector<double> XYbounds_;
  std::shared_ptr<Node3d> start_node_;
  std::shared_ptr<Node3d> end_node_;
  std::shared_ptr<Node3d> final_node_;

  struct cmp {
    bool operator()(const std::pair<std::string, double> &left,
                    const std::pair<std::string, double> &right) const {
      return left.second >= right.second;
    }
  };
  std::priority_queue<std::pair<std::string, double>,
                      std::vector<std::pair<std::string, double>>, cmp>
      open_pq_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> close_set_;
  std::unique_ptr<ReedShepp> reed_shepp_generator_;
  std::vector<ObstacleInfo> ob_vec_;
  std::unique_ptr<GridSearch> grid_a_star_heuristic_generator_;
};