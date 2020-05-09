#include <math.h>
#include <opencv2/opencv.hpp>

#include "hybrid_a_star.h"
using namespace cv;
// This is simple test case, similar to gtest, as no thrid party lib allowed, so
// not using gtest
// Mainly for visulization and the setup is quite simple

class HybridATest {
public:
  void SetUp(const MapBoundaryConfig &map_config) {
    XYbounds_.push_back(0.0);
    XYbounds_.push_back(map_config.map_width);
    XYbounds_.push_back(0.0);
    XYbounds_.push_back(map_config.map_height);
    hybrid_test_ =
        std::unique_ptr<HybridAStar>(new HybridAStar(hybrid_astar_config_));
  }

  void UturnTestRun(const HybridAStartResult &cur_lane_center,
                    const HybridAStartResult &target_lane_center,
                    std::vector<ObstacleInfo> &ob_vec, cv::Mat &img) {
    HybridAStartResult result;
    hybrid_test_->UturnPlan(cur_lane_center, target_lane_center, XYbounds_,
                            ob_vec, &result);
    // below all for visulize
    for (size_t i = 0; i < result.x.size(); ++i) {
      // std::cout << " x: " << result.x.at(i) << " y: " << result.y.at(i)
      //           << " phi: " << result.phi.at(i)
      //           << " kappa: " << result.kappa.at(i) << std::endl;

      // draw img pos
      cv::Point circle_center_cur;
      circle_center_cur.x = result.x.at(i) * kMeter2Pixel;
      circle_center_cur.y = result.y.at(i) * kMeter2Pixel;

      // only for dwawing bad code
      VehicleParam vehicle_param;
      double ego_length = vehicle_param.length;
      double ego_width = vehicle_param.width;
      double ego_phi = result.phi.at(i);
      double shift_distance = ego_length / 2.0 - vehicle_param.back_to_gear;
      Point2D center(result.x.at(i) + shift_distance * std::cos(ego_phi),
                     result.y.at(i) + shift_distance * std::sin(ego_phi));

      std::vector<Point2D> corners =
          util::BoxCenter2Corners(center, ego_phi, ego_length, ego_width);
      std::vector<cv::Point> contour;
      contour.resize(corners.size());
      for (size_t i = 0; i < corners.size(); ++i) {
        contour[i].x = corners[i].x * kMeter2Pixel;
        contour[i].y = corners[i].y * kMeter2Pixel;
      }
      std::vector<std::vector<cv::Point>> contours;
      contours.push_back(contour);

      cv::circle(img, circle_center_cur, 2, Scalar(36, 127, 255), -1);
      cv::drawContours(img, contours, 0, cv::Scalar(255, 0, 0), 1);
      imshow("Test", img); // show img
      waitKey(100);
    }
  }

protected:
  std::unique_ptr<HybridAStar> hybrid_test_;
  HybridAstarConfig hybrid_astar_config_;
  std::vector<double> XYbounds_;
};

int main() {
  // image definition and visulization
  MapBoundaryConfig map_config;
  const int img_size_width =
      map_config.map_width * kMeter2Pixel; // cv map x direction meter
  const int img_size_height =
      map_config.map_height * kMeter2Pixel; // y direction meter
  Mat img(img_size_height, img_size_width, CV_8UC3, Scalar(255, 255, 255));

  // lanes center start and end position
  const double cur_lane_center_y =
      map_config.lane_width * 1.5 + map_config.lane_gap;
  const double target_lane_center_y = map_config.lane_width * 0.5;
  const double road_center_y =
      map_config.lane_width + map_config.lane_gap * 0.5;
  const double cur_lane_center_phi = 0.;
  const double target_lane_center_phi = -M_PI;

  // use lanes center start and end position to generate center lines
  HybridAStartResult cur_lane_center_vec, target_lane_center_vec,
      road_center_vec;
  constexpr size_t input_lane_center_size = 5;

  Scalar cur_lane_color(255, 255, 0);
  Scalar target_lane_color(255, 0, 255);
  double temp_cur_lane_center_x = 0.;
  double temp_target_lane_center_x = 0.;
  for (size_t i = 0; i < input_lane_center_size; ++i) {
    // calculate the two center lanes
    temp_cur_lane_center_x =
        map_config.map_width / 4. +
        map_config.map_width / 4. * i /
            (input_lane_center_size - 1); // only x pos change
    cur_lane_center_vec.x.emplace_back(temp_cur_lane_center_x);
    cur_lane_center_vec.y.emplace_back(cur_lane_center_y);
    cur_lane_center_vec.phi.emplace_back(cur_lane_center_phi);
    cv::circle(img, Point(temp_cur_lane_center_x * kMeter2Pixel,
                          cur_lane_center_y * kMeter2Pixel),
               3, cur_lane_color, -1);

    target_lane_center_vec.x.emplace_back(
        temp_cur_lane_center_x); // same as current lane
    target_lane_center_vec.y.emplace_back(target_lane_center_y);
    target_lane_center_vec.phi.emplace_back(target_lane_center_phi);
    cv::circle(img, Point(temp_cur_lane_center_x * kMeter2Pixel,
                          target_lane_center_y * kMeter2Pixel),
               3, target_lane_color, -1);

    // road center, in general there are obstacles and come from perception;
    // here you can customize
    road_center_vec.x.emplace_back(
        temp_cur_lane_center_x); // same as current lane
    road_center_vec.y.emplace_back(road_center_y);
    road_center_vec.phi.emplace_back(cur_lane_center_phi); // does not
                                                           // matter
  }

  // // draw two lanes
  cv::line(img, Point(0., 0.), Point(img_size_width / 2., 0.),
           target_lane_color, 3);
  cv::line(img, Point(0., map_config.lane_width * kMeter2Pixel),
           Point(img_size_width / 2., map_config.lane_width * kMeter2Pixel),
           target_lane_color, 3);

  cv::line(img, Point(0., (map_config.lane_width + map_config.lane_gap) *
                              kMeter2Pixel),
           Point(img_size_width / 2.,
                 (map_config.lane_width + map_config.lane_gap) * kMeter2Pixel),
           cur_lane_color, 3);
  cv::line(
      img, Point(0., (2. * map_config.lane_width + map_config.lane_gap) *
                         kMeter2Pixel),
      Point(img_size_width / 2.,
            (2. * map_config.lane_width + map_config.lane_gap) * kMeter2Pixel),
      cur_lane_color, 3);

  // here start the tests
  HybridATest hyt;

  // set obstacles; here are road centers; you can customize it
  std::vector<ObstacleInfo> ob_vec;
  for (size_t i = 0; i < road_center_vec.x.size(); ++i) {
    ob_vec.emplace_back(ObstacleInfo(
        road_center_vec.x.at(i), road_center_vec.y.at(i),
        std::max(map_config.lane_gap / 2., 0.3))); // minimum radius is 0.3
    cv::circle(img, Point(ob_vec[i].obstacle_x * kMeter2Pixel,
                          ob_vec[i].obstacle_y * kMeter2Pixel),
               ob_vec[i].obstacle_radius * kMeter2Pixel, Scalar(0, 0, 0), -1);
  }

  imshow("Test", img); // show img
  waitKey(100);

  hyt.SetUp(map_config);
  hyt.UturnTestRun(cur_lane_center_vec, target_lane_center_vec, ob_vec, img);
  waitKey(0);
  return 0;
}
