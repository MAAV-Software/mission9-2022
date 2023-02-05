#ifndef UTILITIES_H
#define UTILITIES_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <tuple>
#include <cstdlib>
#include <math.h>
#include <string>
#include <cassert>
#include <exception>
using namespace cv;
using namespace std;

class Utilities
{
  public:
    static vpImagePoint cvPointToVPImagePoint(cv::Point& point){
      vpImagePoint new_point;
      new_point.set_i(point.x);
      new_point.set_j(point.y);

      return new_point;
    }

    static std::vector<vpImagePoint> cvPointsToVPImagePoints (std::vector<cv::Point>& points){
      std::vector<vpImagePoint> visp_points;
      for (auto p: points){
        vpImagePoint visp_point = cvPointToVPImagePoint(p);
        visp_points.push_back(visp_point);
      }
      return visp_points;
    }

};
#endif