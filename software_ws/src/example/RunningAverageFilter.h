#ifndef RUNNING_AVERAGE_FILTER_H
#define RUNNING_AVERAGE_FILTER_H

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
#include <queue>
using namespace cv;
using namespace std;


class RunningAverageFilter
{
  private:
    int maxSize;
    queue<cv::Point> myRunningPoints; 
    cv::Point runningTotal;
  public:

  RunningAverageFilter(int s) : maxSize{s}, runningTotal{cv::Point(0,0)}{ }

  void update_filter(cv::Point point){
    myRunningPoints.push(point);
    runningTotal.x += point.x;
    runningTotal.y += point.y;

    
    if(myRunningPoints.size() > maxSize) {
      cv::Point popped_point = myRunningPoints.front();
      myRunningPoints.pop();
      runningTotal.x -= popped_point.x;
      runningTotal.y -= popped_point.y;
      
    }
  }

  cv::Point get_running_average(){
    int num_vals = myRunningPoints.size();
    return cv::Point(runningTotal.x / num_vals, runningTotal.y / num_vals);
  }
  
};
#endif