#include "hybrid_a_star.h"

HybridAStar::HybridAStar(const HybridAstarConfig &open_space_conf) {
  reed_shepp_generator_ =
      std::make_unique<ReedShepp>(vehicle_param_, hybrid_astar_config_);
  grid_a_star_heuristic_generator_ =
      std::make_unique<GridSearch>(hybrid_astar_config_);
  next_node_num_ = hybrid_astar_config_.next_node_num;
  max_wheel_angle_ =
      vehicle_param_.max_steer_angle / vehicle_param_.steer_ratio;
  step_size_ = hybrid_astar_config_.step_size;
  xy_grid_resolution_ = hybrid_astar_config_.xy_grid_resolution;
  delta_t_ = hybrid_astar_config_.delta_t;
  traj_forward_penalty_ = hybrid_astar_config_.traj_forward_penalty;
  traj_back_penalty_ = hybrid_astar_config_.traj_back_penalty;
  traj_gear_switch_penalty_ = hybrid_astar_config_.traj_gear_switch_penalty;
  traj_steer_penalty_ = hybrid_astar_config_.traj_steer_penalty;
  traj_steer_change_penalty_ = hybrid_astar_config_.traj_steer_change_penalty;

  ob_vec_.clear();
}

bool HybridAStar::AnalyticExpansion(std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
  if (!reed_shepp_generator_->ShortestRSP(current_node, end_node_,
                                          reeds_shepp_to_check)) {
    // std::cout << "ShortestRSP failed" << std::endl;
    return false;
  }

  if (!RSPCheck(reeds_shepp_to_check)) {
    return false;
  }

  std::cout << "Reach the end configuration with Reed Shepp" << std::endl;
  // load the whole RSP as nodes and add to the close set
  final_node_ = LoadRSPinCS(reeds_shepp_to_check, current_node);
  return true;
}

bool HybridAStar::RSPCheck(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end) {
  std::shared_ptr<Node3d> node = std::shared_ptr<Node3d>(
      new Node3d(reeds_shepp_to_end->x, reeds_shepp_to_end->y,
                 reeds_shepp_to_end->phi, XYbounds_, hybrid_astar_config_));
  return ValidityCheck(node);
}

bool HybridAStar::ValidityCheck(std::shared_ptr<Node3d> node) {
  size_t node_step_size = node->GetStepSize();
  const auto &traversed_x = node->GetXs();
  const auto &traversed_y = node->GetYs();
  const auto &traversed_phi = node->GetPhis();

  // The first {x, y, phi} is collision free unless they are start and end
  // configuration of search problem
  size_t check_start_index = 0;
  if (node_step_size == 1) {
    check_start_index = 0;
  } else {
    check_start_index = 1;
  }
  for (size_t i = check_start_index; i < node_step_size; ++i) {
    if (traversed_x[i] > XYbounds_[1] || traversed_x[i] < XYbounds_[0] ||
        traversed_y[i] > XYbounds_[3] || traversed_y[i] < XYbounds_[2]) {
      std::cout << "b: " << XYbounds_[0] << " " << XYbounds_[1] << " "
                << XYbounds_[2] << " " << XYbounds_[3] << std::endl;
      std::cout << "ERROR: point outside of bounds!" << std::endl;
      return false;
    }
    if (CollisionDetection(ob_vec_, traversed_x[i], traversed_y[i],
                           traversed_phi[i])) {
      return false;
    }
    if (OutOfBoundaryCheck(traversed_x[i], traversed_y[i], traversed_phi[i])) {
      return false;
    }
  }
  return true;
}

std::shared_ptr<Node3d> HybridAStar::LoadRSPinCS(
    const std::shared_ptr<ReedSheppPath> reeds_shepp_to_end,
    std::shared_ptr<Node3d> current_node) {
  std::shared_ptr<Node3d> end_node = std::shared_ptr<Node3d>(
      new Node3d(reeds_shepp_to_end->x, reeds_shepp_to_end->y,
                 reeds_shepp_to_end->phi, XYbounds_, hybrid_astar_config_));
  end_node->SetPre(current_node);
  close_set_.insert(std::make_pair(end_node->GetIndex(), end_node));
  return end_node;
}

std::shared_ptr<Node3d>
HybridAStar::Next_node_generator(std::shared_ptr<Node3d> current_node,
                                 size_t next_node_index) {
  double steering = 0.0;
  size_t index = 0;
  double traveled_distance = 0.0;
  if (next_node_index < static_cast<double>(next_node_num_) / 2) {
    steering =
        -max_wheel_angle_ +
        (2 * max_wheel_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(next_node_index);
    traveled_distance = step_size_;
  } else {
    index = next_node_index - next_node_num_ / 2;
    steering =
        -max_wheel_angle_ +
        (2 * max_wheel_angle_ / (static_cast<double>(next_node_num_) / 2 - 1)) *
            static_cast<double>(index);
    traveled_distance = -step_size_;
  }
  // take above motion primitive to generate a curve driving the car to a
  // different grid
  double arc = std::sqrt(2) * xy_grid_resolution_;
  std::vector<double> intermediate_x;
  std::vector<double> intermediate_y;
  std::vector<double> intermediate_phi;
  double last_x = current_node->GetX();
  double last_y = current_node->GetY();
  double last_phi = current_node->GetPhi();
  intermediate_x.push_back(last_x);
  intermediate_y.push_back(last_y);
  intermediate_phi.push_back(last_phi);
  for (size_t i = 0; i < arc / step_size_; ++i) {
    const double next_x = last_x + traveled_distance * std::cos(last_phi);
    const double next_y = last_y + traveled_distance * std::sin(last_phi);
    const double next_phi = util::NormalizeAngle(
        last_phi +
        traveled_distance / vehicle_param_.wheel_base * std::tan(steering));
    intermediate_x.push_back(next_x);
    intermediate_y.push_back(next_y);
    intermediate_phi.push_back(next_phi);
    last_x = next_x;
    last_y = next_y;
    last_phi = next_phi;
  }

  if (OutOfBoundaryCheck(intermediate_x.back(), intermediate_y.back(),
                         intermediate_phi.back())) {
    return nullptr;
  }

  std::shared_ptr<Node3d> next_node = std::shared_ptr<Node3d>(
      new Node3d(intermediate_x, intermediate_y, intermediate_phi, XYbounds_,
                 hybrid_astar_config_));
  next_node->SetPre(current_node);
  next_node->SetDirec(traveled_distance > 0.0);
  next_node->SetSteer(steering);
  return next_node;
}

void HybridAStar::CalculateNodeCost(std::shared_ptr<Node3d> current_node,
                                    std::shared_ptr<Node3d> next_node) {
  next_node->SetTrajCost(current_node->GetTrajCost() +
                         TrajCost(current_node, next_node));
  // evaluate heuristic cost
  double optimal_path_cost = 0.0;
  optimal_path_cost += HoloObstacleHeuristic(next_node);
  next_node->SetHeuCost(optimal_path_cost);
}

double HybridAStar::TrajCost(std::shared_ptr<Node3d> current_node,
                             std::shared_ptr<Node3d> next_node) {
  // evaluate cost on the trajectory and add current cost
  double piecewise_cost = 0.0;
  if (next_node->GetDirec()) {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_forward_penalty_;
  } else {
    piecewise_cost += static_cast<double>(next_node->GetStepSize() - 1) *
                      step_size_ * traj_back_penalty_;
  }
  if (current_node->GetDirec() != next_node->GetDirec()) {
    piecewise_cost += traj_gear_switch_penalty_;
  }
  piecewise_cost += traj_steer_penalty_ * std::abs(next_node->GetSteer());
  piecewise_cost += traj_steer_change_penalty_ *
                    std::abs(next_node->GetSteer() - current_node->GetSteer());
  return piecewise_cost;
}

double HybridAStar::HoloObstacleHeuristic(std::shared_ptr<Node3d> next_node) {
  // return 0;
  // currently use RS length for heuristic
  double rs_cost = 0;
  std::shared_ptr<ReedSheppPath> reeds_shepp_to_check =
      std::make_shared<ReedSheppPath>();
  if (reed_shepp_generator_->ShortestRSP(next_node, end_node_,
                                         reeds_shepp_to_check)) {
    rs_cost = reeds_shepp_to_check->total_length;
  } else {
    // std::cout << "Warning: RS calculation no solution!\n";
    rs_cost = 0.;
  }

  // add a* cost
  double astar_cost = 0;
  if (kUsingDPMap) {
    astar_cost = grid_a_star_heuristic_generator_->CheckDpMap(
        next_node->GetX(), next_node->GetY());
  }
  return std::max(rs_cost, astar_cost);
}

bool HybridAStar::GetResult(HybridAStartResult *result) {
  std::shared_ptr<Node3d> current_node = final_node_;
  std::vector<double> hybrid_a_x;
  std::vector<double> hybrid_a_y;
  std::vector<double> hybrid_a_phi;
  while (current_node->GetPreNode() != nullptr) {
    std::vector<double> x = current_node->GetXs();
    std::vector<double> y = current_node->GetYs();
    std::vector<double> phi = current_node->GetPhis();
    if (x.empty() || y.empty() || phi.empty()) {
      std::cout << "result size check failed" << std::endl;
      return false;
    }
    if (x.size() != y.size() || x.size() != phi.size()) {
      std::cout << "states sizes are not equal" << std::endl;
      return false;
    }
    std::reverse(x.begin(), x.end());
    std::reverse(y.begin(), y.end());
    std::reverse(phi.begin(), phi.end());
    x.pop_back();
    y.pop_back();
    phi.pop_back();
    hybrid_a_x.insert(hybrid_a_x.end(), x.begin(), x.end());
    hybrid_a_y.insert(hybrid_a_y.end(), y.begin(), y.end());
    hybrid_a_phi.insert(hybrid_a_phi.end(), phi.begin(), phi.end());
    current_node = current_node->GetPreNode();
  }
  hybrid_a_x.push_back(current_node->GetX());
  hybrid_a_y.push_back(current_node->GetY());
  hybrid_a_phi.push_back(current_node->GetPhi());
  std::reverse(hybrid_a_x.begin(), hybrid_a_x.end());
  std::reverse(hybrid_a_y.begin(), hybrid_a_y.end());
  std::reverse(hybrid_a_phi.begin(), hybrid_a_phi.end());
  (*result).x = hybrid_a_x;
  (*result).y = hybrid_a_y;
  (*result).phi = hybrid_a_phi;

  if (!GetTemporalProfile(result)) {
    std::cout << "GetSpeedProfile from Hybrid Astar path fails" << std::endl;
    return false;
  }

  if (result->x.size() != result->y.size() ||
      result->x.size() != result->v.size() ||
      result->x.size() != result->phi.size()) {
    std::cout << "state sizes not equal, "
              << "result->x.size(): " << result->x.size() << "result->y.size()"
              << result->y.size() << "result->phi.size()" << result->phi.size()
              << "result->v.size()" << result->v.size();
    return false;
  }

  if (result->a.size() != result->steer.size() ||
      result->x.size() - result->a.size() != 1) {
    std::cout << "control sizes not equal or not right" << std::endl;
    std::cout << " acceleration size: " << result->a.size();
    std::cout << " steer size: " << result->steer.size();
    std::cout << " x size: " << result->x.size();
    return false;
  }
  // make v, a, steer and kappa the same length as x,y, phi
  if (result->x.size() >= 2) {
    result->v.resize(result->x.size());
    result->a.resize(result->x.size());
    result->steer.resize(result->x.size());
    result->kappa.resize(result->x.size());
    result->v.at(result->x.size() - 1) = result->v.at(result->x.size() - 2);
    result->a.at(result->x.size() - 1) = result->a.at(result->x.size() - 2);
    result->steer.at(result->x.size() - 1) =
        result->steer.at(result->x.size() - 2);
    result->kappa.at(result->x.size() - 1) =
        result->kappa.at(result->x.size() - 2);
  }

  return true;
}

bool HybridAStar::GenerateSpeedAcceleration(HybridAStartResult *result) {
  // Sanity Check
  if (result->x.size() < 2 || result->y.size() < 2 || result->phi.size() < 2) {
    std::cout << "result size check when generating speed and acceleration fail"
              << std::endl;
    return false;
  }
  const size_t x_size = result->x.size();

  // load velocity from position
  // initial and end speed are set to be zeros
  result->v.push_back(0.0);
  for (size_t i = 1; i + 1 < x_size; ++i) {
    double discrete_v = (((result->x[i + 1] - result->x[i]) / delta_t_) *
                             std::cos(result->phi[i]) +
                         ((result->x[i] - result->x[i - 1]) / delta_t_) *
                             std::cos(result->phi[i])) /
                            2.0 +
                        (((result->y[i + 1] - result->y[i]) / delta_t_) *
                             std::sin(result->phi[i]) +
                         ((result->y[i] - result->y[i - 1]) / delta_t_) *
                             std::sin(result->phi[i])) /
                            2.0;
    result->v.push_back(discrete_v);
  }
  result->v.push_back(0.0);

  // load acceleration from velocity
  for (size_t i = 0; i + 1 < x_size; ++i) {
    const double discrete_a = (result->v[i + 1] - result->v[i]) / delta_t_;
    result->a.push_back(discrete_a);
  }

  // load steering from phi
  for (size_t i = 0; i + 1 < x_size; ++i) {
    double discrete_steer = (result->phi[i + 1] - result->phi[i]) *
                            vehicle_param_.wheel_base / step_size_;
    if (result->v[i] > 0.0) {
      discrete_steer = std::atan(discrete_steer);
    } else {
      discrete_steer = std::atan(-discrete_steer);
    }
    result->steer.push_back(discrete_steer);
    result->kappa.emplace_back(
        std::tan(result->steer.at(i) / vehicle_param_.wheel_base));
  }
  return true;
}

bool HybridAStar::GetTemporalProfile(HybridAStartResult *result) {
  std::vector<HybridAStartResult> partitioned_results;
  if (!TrajectoryPartition(*result, &partitioned_results)) {
    std::cout << "TrajectoryPartition fail" << std::endl;
    return false;
  }
  HybridAStartResult stitched_result;
  for (const auto &result : partitioned_results) {
    std::copy(result.x.begin(), result.x.end() - 1,
              std::back_inserter(stitched_result.x));
    std::copy(result.y.begin(), result.y.end() - 1,
              std::back_inserter(stitched_result.y));
    std::copy(result.phi.begin(), result.phi.end() - 1,
              std::back_inserter(stitched_result.phi));
    std::copy(result.v.begin(), result.v.end() - 1,
              std::back_inserter(stitched_result.v));
    std::copy(result.a.begin(), result.a.end(),
              std::back_inserter(stitched_result.a));
    std::copy(result.steer.begin(), result.steer.end(),
              std::back_inserter(stitched_result.steer));
    std::copy(result.kappa.begin(), result.kappa.end(),
              std::back_inserter(stitched_result.kappa));
  }
  stitched_result.x.push_back(partitioned_results.back().x.back());
  stitched_result.y.push_back(partitioned_results.back().y.back());
  stitched_result.phi.push_back(partitioned_results.back().phi.back());
  stitched_result.v.push_back(partitioned_results.back().v.back());
  *result = stitched_result;
  return true;
}

bool HybridAStar::Plan(double sx, double sy, double sphi, double ex, double ey,
                       double ephi, const std::vector<double> &XYbounds,
                       std::vector<ObstacleInfo> &ob_vec,
                       HybridAStartResult *result) {
  FunctionTimer plan("Total Plan");
  std::chrono::high_resolution_clock::time_point astar_start_time =
      std::chrono::high_resolution_clock::now();
  // clear containers
  open_set_.clear();
  close_set_.clear();
  open_pq_ = decltype(open_pq_)();
  final_node_ = nullptr;

  // load XYbounds
  XYbounds_ = XYbounds;

  // set obstacles
  ob_vec_.clear();
  for (size_t i = 0; i < ob_vec.size(); ++i) {
    ob_vec_.push_back(ob_vec[i]);
  }

  // load nodes
  start_node_.reset(
      new Node3d({sx}, {sy}, {sphi}, XYbounds_, hybrid_astar_config_));
  end_node_.reset(
      new Node3d({ex}, {ey}, {ephi}, XYbounds_, hybrid_astar_config_));
  if (!ValidityCheck(start_node_)) {
    std::cout << "start_node in collision with obstacles" << std::endl;
    return false;
  }
  if (!ValidityCheck(end_node_)) {
    std::cout << "end node x: " << end_node_->GetX()
              << " y: " << end_node_->GetX() << std::endl;
    for (size_t i = 0; i < ob_vec_.size(); ++i) {
      std::cout << "ob x: " << ob_vec_[i].obstacle_x
                << " ob y: " << ob_vec_[i].obstacle_y
                << " ob_r: " << ob_vec_[i].obstacle_radius << std::endl;
    }
    std::cout << "end_node in collision with obstacles" << std::endl;
    return false;
  }
  // load open set, pq
  open_set_.insert(std::make_pair(start_node_->GetIndex(), start_node_));
  open_pq_.push(
      std::make_pair(start_node_->GetIndex(), start_node_->GetCost()));

  // generate dp all in pixel dimension
  std::chrono::high_resolution_clock::time_point dp_map_time =
      std::chrono::high_resolution_clock::now();
  if (kUsingDPMap) {
    grid_a_star_heuristic_generator_->GenerateDpMap(ex, ey, XYbounds_, ob_vec_);
  }
  std::cout << "DP map time is: "
            << std::chrono::duration<double, std::milli>(
                   std::chrono::high_resolution_clock::now() - dp_map_time)
                   .count()
            << std::endl;
  // Hybrid A* begins
  size_t explored_node_num = 0;
  // std::chrono::high_resolution_clock::time_point astar_start_time =
  // std::chrono::high_resolution_clock::now();
  double heuristic_time = 0.0;
  double rs_time = 0.0;
  std::chrono::high_resolution_clock::time_point start_time;
  std::chrono::high_resolution_clock::time_point end_time;
  while (!open_pq_.empty()) {
    // take out the lowest cost neighboring node
    const std::string current_id = open_pq_.top().first;
    open_pq_.pop();
    std::shared_ptr<Node3d> current_node = open_set_[current_id];
    // check if an analystic curve could be connected from current
    // configuration to the end configuration without collision. if so, search
    // ends.
    start_time = std::chrono::high_resolution_clock::now();

    if (AnalyticExpansion(current_node)) {
      break;
    }
    end_time = std::chrono::high_resolution_clock::now();
    rs_time += std::chrono::duration<double, std::milli>(end_time - start_time)
                   .count();
    close_set_.insert(std::make_pair(current_node->GetIndex(), current_node));
    for (size_t i = 0; i < next_node_num_; ++i) {
      std::shared_ptr<Node3d> next_node = Next_node_generator(current_node, i);
      // boundary check failure handle
      if (next_node == nullptr) {
        continue;
      }
      // check if the node is already in the close set
      if (close_set_.find(next_node->GetIndex()) != close_set_.end()) {
        continue;
      }
      // collision check
      if (!ValidityCheck(next_node)) {
        continue;
      }
      if (open_set_.find(next_node->GetIndex()) == open_set_.end()) {
        explored_node_num++;
        start_time = std::chrono::high_resolution_clock::now();
        CalculateNodeCost(current_node, next_node);
        end_time = std::chrono::high_resolution_clock::now();
        heuristic_time +=
            std::chrono::duration<double, std::milli>(end_time - start_time)
                .count();
        open_set_.emplace(next_node->GetIndex(), next_node);
        open_pq_.emplace(next_node->GetIndex(), next_node->GetCost());
      }
    }
  }
  if (final_node_ == nullptr) {
    std::cout << "Hybrid A searching return null ptr(open_set ran out)"
              << std::endl;
    return false;
  }
  if (!GetResult(result)) {
    std::cout << "GetResult failed" << std::endl;
    return false;
  }
  std::cout << "explored node num is " << explored_node_num << std::endl;
  std::cout << "heuristic time is " << heuristic_time << " ms" << std::endl;
  std::cout << "reed shepp time is " << rs_time << " ms" << std::endl;
  std::cout << "hybrid astar total time is "
            << std::chrono::duration<double, std::milli>(
                   std::chrono::high_resolution_clock::now() - astar_start_time)
                   .count()
            << std::endl;
  return true;
}

bool HybridAStar::TrajectoryPartition(
    const HybridAStartResult &result,
    std::vector<HybridAStartResult> *partitioned_result) {
  const auto &x = result.x;
  const auto &y = result.y;
  const auto &phi = result.phi;
  if (x.size() != y.size() || x.size() != phi.size()) {
    std::cout
        << "states sizes are not equal when do trajectory partitioning of "
           "Hybrid A Star result";
    return false;
  }

  size_t horizon = x.size();
  partitioned_result->clear();
  partitioned_result->emplace_back();
  auto *current_traj = &(partitioned_result->back());
  double heading_angle = phi.front();
  const Vec2d init_tracking_vector(x[1] - x[0], y[1] - y[0]);
  double tracking_angle = init_tracking_vector.Angle();
  bool current_gear =
      std::abs(util::NormalizeAngle(tracking_angle - heading_angle)) < (M_PI_2);
  for (size_t i = 0; i < horizon - 1; ++i) {
    heading_angle = phi[i];
    const Vec2d tracking_vector(x[i + 1] - x[i], y[i + 1] - y[i]);
    tracking_angle = tracking_vector.Angle();
    bool gear = std::abs(util::NormalizeAngle(tracking_angle - heading_angle)) <
                (M_PI_2);
    if (gear != current_gear) {
      current_traj->x.push_back(x[i]);
      current_traj->y.push_back(y[i]);
      current_traj->phi.push_back(phi[i]);
      partitioned_result->emplace_back();
      current_traj = &(partitioned_result->back());
      current_gear = gear;
    }
    current_traj->x.push_back(x[i]);
    current_traj->y.push_back(y[i]);
    current_traj->phi.push_back(phi[i]);
  }
  current_traj->x.push_back(x.back());
  current_traj->y.push_back(y.back());
  current_traj->phi.push_back(phi.back());

  const auto start_timestamp = std::chrono::system_clock::now();

  // Retrieve v, a, steer and kappa from path
  for (auto &result : *partitioned_result) {
    if (!GenerateSpeedAcceleration(&result)) {
      std::cout << "GenerateSpeedAcceleration fail";
      return false;
    }
  }

  const auto end_timestamp = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = end_timestamp - start_timestamp;
  std::cout << "speed profile total time: " << diff.count() * 1000.0 << " ms."
            << std::endl;
  return true;
}

/// added for test only and need reformulate
double HybridAStar::Distance(const ObstacleInfo &ob, const double &x,
                             const double &y, const double &phi) const {
  double d = pow(ob.obstacle_x - x, 2) + pow(ob.obstacle_y - y, 2);
  return sqrt(d);
}

// TODO refactor
bool HybridAStar::CollisionDetection(const std::vector<ObstacleInfo> &ob_vec,
                                     const double &x, const double &y,
                                     const double &phi) const {
  const double shift_distance =
      vehicle_param_.length / 2.0 - vehicle_param_.back_to_gear;
  Point2D center(x + shift_distance * std::cos(phi),
                 y + shift_distance * std::sin(phi));

  std::vector<Point2D> corners = std::move(util::BoxCenter2Corners(
      center, phi, vehicle_param_.length, vehicle_param_.width));

  for (size_t i = 0; i < ob_vec.size(); ++i) {
    for (size_t j = 0; j < corners.size(); ++j) {
      // collision detection with obstacles
      if (Distance(ob_vec[i], corners[j].x, corners[j].y, phi) <=
          ob_vec[i].obstacle_radius + collision_margin_) {
        return true; // collision
      }
    }
  }
  return false; // no collision
}

bool HybridAStar::OutOfBoundaryCheck(const double &x, const double &y,
                                     const double &phi) const {
  const double shift_distance =
      vehicle_param_.length / 2.0 - vehicle_param_.back_to_gear;
  Point2D center(x + shift_distance * std::cos(phi),
                 y + shift_distance * std::sin(phi));

  std::vector<Point2D> corners = std::move(util::BoxCenter2Corners(
      center, phi, vehicle_param_.length, vehicle_param_.width));

  for (size_t i = 0; i < corners.size(); ++i) {
    if (corners.at(i).x < XYbounds_.at(0) + collision_margin_ ||
        corners.at(i).x > XYbounds_.at(1) - collision_margin_ ||
        corners.at(i).y < XYbounds_.at(2) + collision_margin_ ||
        corners.at(i).y > XYbounds_.at(3) - collision_margin_) {
      return true; // out of boundary
    }
  }
  return false; // within boundary
}

void HybridAStar::UturnPlan(const HybridAStartResult &start_lane_center,
                            const HybridAStartResult &target_lane_center,
                            const std::vector<double> &XYbounds,
                            std::vector<ObstacleInfo> &ob_vec,
                            HybridAStartResult *result) {
  // since we have no idea about the size of the two lane center
  // i use a simple method to choose start and end position
  // you can customize it according to your application
  if (!start_lane_center.x.empty() && !start_lane_center.y.empty() &&
      !start_lane_center.phi.empty() && !target_lane_center.x.empty() &&
      !target_lane_center.y.empty() && !target_lane_center.phi.empty()) {
    double sx = start_lane_center.x.back();
    double sy = start_lane_center.y.back();
    double sphi = start_lane_center.phi.back();

    double ex = target_lane_center.x.back();
    double ey = target_lane_center.y.back();
    double ephi = target_lane_center.phi.back();

    Plan(sx, sy, sphi, ex, ey, ephi, XYbounds, ob_vec, result);
  } else {
    std::cout << "Input lane center empty, please check!" << std::endl;
  }
}
