// simple test cases

#include "reeds_shepp_path.h"
#include "node3d.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

#if 0
class reeds_shepp
{
public:
    virtual void SetUp(const int &height, const int &width)
    {
        reedshepp_test = std::unique_ptr<ReedShepp>(new ReedShepp(vehicle_param_, hybrid_astar_config_));
        XYbounds_.push_back(0.0);
        XYbounds_.push_back(double(height));
        XYbounds_.push_back(0.0);
        XYbounds_.push_back(double(width));
    }
    void check(std::shared_ptr<Node3d> &start_node, std::shared_ptr<Node3d> &end_node,
               std::shared_ptr<ReedSheppPath> &optimal_path, cv::Mat &img)
    {
        for (size_t i = 0; i < (*optimal_path).x.size(); ++i)
        {
            std::cout << "x: " << (*optimal_path).x.at(i)*kMeter2Pixel << " y: " 
            << (*optimal_path).y.at(i)*kMeter2Pixel << " phi: " << (*optimal_path).phi.at(i) << " gear: " << (*optimal_path).gear.at(i) << std::endl;
            // draw img x_pos verses time
            cv::Point circle_center_cur;
            circle_center_cur.x = (*optimal_path).x.at(i)*kMeter2Pixel;
            circle_center_cur.y = (*optimal_path).y.at(i)*kMeter2Pixel;

            cv::circle(img, circle_center_cur, 2, Scalar(255, 0, 0), -1);
            imshow("Test", img); // show img
            waitKey(20);
        }
        for (size_t i = 0; i < (*optimal_path).segs_lengths.size(); ++i)
        {
            std::cout << "segs_lengths: " << (*optimal_path).segs_lengths.at(i) << " segs_types: " << (*optimal_path).segs_types.at(i)
                      << std::endl;
        }
        std::cout << " total_length: " << (*optimal_path).total_length << std::endl;
    }

    void RSTest(const double &s_x, const double &s_y, const double &s_phi,
                const double &e_x, const double &e_y, const double &e_phi, cv::Mat &img)
    {
        std::shared_ptr<Node3d> start_node =
            std::shared_ptr<Node3d>(new Node3d(s_x, s_y, s_phi, XYbounds_, hybrid_astar_config_));
        std::shared_ptr<Node3d> end_node =
            std::shared_ptr<Node3d>(new Node3d(e_x, e_y, e_phi, XYbounds_, hybrid_astar_config_));
        std::shared_ptr<ReedSheppPath> optimal_path = std::shared_ptr<ReedSheppPath>(new ReedSheppPath());
        if (!reedshepp_test->ShortestRSP(start_node, end_node, optimal_path))
        {
            std::cout << "generating short RSP not successful";
        }
        else
        {
            check(start_node, end_node, optimal_path, img);
        }        
    }

protected:
    VehicleParam vehicle_param_;
    HybridAstarConfig hybrid_astar_config_;
    std::unique_ptr<ReedShepp> reedshepp_test;
    std::vector<double> XYbounds_;
};

int main()
{
    // image definition
    
    const int img_size_width = 1200; // x direction
    const int img_size_height = 600; // y direction
    double s_x = 10.;
    double s_y = 10.;
    double s_phi = -45. * M_PI / 180.;
    double e_x = 10.3;
    double e_y = 10.4;
    double e_phi = -180. * M_PI / 180.; // angle is [-180, 180)
    Mat img(img_size_height, img_size_width, CV_8UC3, Scalar(255, 255, 255));
    imshow("Test", img);

    const int pixel_scale_width = 1;
    const int pixel_scale_height = 1;
    Scalar ref_color = Scalar(255, 255, 0);
    Scalar real_color = Scalar(255, 0, 255);

    putText(img, "start position", Point(s_x*kMeter2Pixel, s_y*kMeter2Pixel), 1, 1, ref_color);
    putText(img, "goal position", Point(e_x*kMeter2Pixel, e_y*kMeter2Pixel), 1, 1, real_color);
    // draw start and end pos
    cv::circle(img, Point(s_x*kMeter2Pixel, s_y*kMeter2Pixel), 3, ref_color, -1);
    cv::circle(img, Point(e_x*kMeter2Pixel, e_y*kMeter2Pixel), 3, real_color, -1);
    imshow("Test", img); // show img
    waitKey(1000);

    reeds_shepp rs_test;
    rs_test.SetUp(img_size_height, img_size_width);
    rs_test.RSTest(s_x, s_y, s_phi, e_x, e_y, e_phi, img);
    waitKey(0);
    return 0;
}
#endif