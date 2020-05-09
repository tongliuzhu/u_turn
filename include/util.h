#pragma once

#include <chrono>
#include <iostream>
#include <math.h>
#include <vector>
/**
 * This is ugly file and only for quick demo usage
 *
 * @brief Implements some common used functions and data structs;
 * TODO refactor
 */

// time evaluation of functions
struct FunctionTimer {
  std::chrono::high_resolution_clock::time_point t2, t1;
  std::string function_name;
  FunctionTimer(std::string name) : function_name(name) {
    t1 = std::chrono::high_resolution_clock::now();
    t2 = std::chrono::high_resolution_clock::now();
  }

  ~FunctionTimer() {
    t2 = std::chrono::high_resolution_clock::now();
    std::cout << function_name << " takes "
              << std::chrono::duration<double, std::milli>(t2 - t1).count()
              << " ms\n";
  }
};

// simple round obstacle for quick demo
struct ObstacleInfo {
  double obstacle_x;
  double obstacle_y;
  double obstacle_radius;
  ObstacleInfo(const double x, const double y, const double o_r)
      : obstacle_x(x), obstacle_y(y), obstacle_radius(o_r) {}
  // copy constructor
  ObstacleInfo(const ObstacleInfo &other) {
    if (&other == this)
      return;
    this->obstacle_x = other.obstacle_x;
    this->obstacle_y = other.obstacle_y;
    this->obstacle_radius = other.obstacle_radius;
  }
  // assign operator
  ObstacleInfo &operator=(const ObstacleInfo &other) {
    if (&other == this)
      return *this;
    this->obstacle_x = other.obstacle_x;
    this->obstacle_y = other.obstacle_y;
    this->obstacle_radius = other.obstacle_radius;
    return *this;
  }

  ObstacleInfo(ObstacleInfo &&a)
      : obstacle_x(a.obstacle_x), obstacle_y(a.obstacle_y),
        obstacle_radius(a.obstacle_radius) {}
  ObstacleInfo &operator=(ObstacleInfo &&a) {
    obstacle_x = a.obstacle_x;
    obstacle_y = a.obstacle_y;
    obstacle_y = a.obstacle_y;
    return *this;
  }
};

struct Point2D {
  double x;
  double y;
  Point2D(double x = 0, double y = 0) : x(x), y(y) {}
};

namespace util {
inline double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

inline std::pair<double, double> Cartesian2Polar(double x, double y) {
  double r = std::sqrt(x * x + y * y);
  double theta = std::atan2(y, x);
  return std::make_pair(r, theta);
}

inline std::vector<Point2D> BoxCenter2Corners(const Point2D &center,
                                              const double &ego_phi,
                                              const double &ego_length,
                                              const double &ego_width) {
  std::vector<Point2D> corners;
  double half_length_ = ego_length / 2.;
  double half_width_ = ego_width / 2.;
  double cos_heading_ = std::cos(ego_phi);
  double sin_heading_ = std::sin(ego_phi);
  const double dx1 = cos_heading_ * half_length_;
  const double dy1 = sin_heading_ * half_length_;
  const double dx2 = sin_heading_ * half_width_;
  const double dy2 = -cos_heading_ * half_width_;
  corners.clear();
  Point2D c1(center.x + dx1 + dx2, center.y + dy1 + dy2);
  Point2D c2(center.x + dx1 - dx2, center.y + dy1 - dy2);
  Point2D c3(center.x - dx1 - dx2, center.y - dy1 - dy2);
  Point2D c4(center.x - dx1 + dx2, center.y - dy1 + dy2);
  corners.emplace_back(c1);
  corners.emplace_back(c2);
  corners.emplace_back(c3);
  corners.emplace_back(c4);
  return corners;
}
};
