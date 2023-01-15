#ifndef IMAGE_CONVERTER_H
#define IMAGE_CONVERTER_H

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
using namespace cv;
using namespace std;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber depth_image_sub_;

  image_transport::Publisher image_pub_;

  cv::Mat depth_mask;
  bool is_depth_mask = false;
  int x_offset = 0;
  int y_offset = 0;
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1,
      &ImageConverter::imageCb, this);
    depth_image_sub_ = it_.subscribe("/camera/depth/image_raw", 1,
      &ImageConverter::depthImageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    // cv::namedWindow("source");
  }

  ~ImageConverter()
  {
    cv::destroyWindow("source");
  }
  int get_x_offset()
  {
    return x_offset;
  }
  int get_y_offset()
  {
    return y_offset;
  }
  void depthImageCb(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    double MAX_DISTANCE_THRESHOLD = 5.0; // in meters


    cv::Mat image = cv_ptr->image;

    
    inRange(image, 0, MAX_DISTANCE_THRESHOLD, this->depth_mask);
    // cv::imshow("Depth Map", this->depth_mask);

    is_depth_mask = true;

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
	cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Run our algorithm on the image...

    cv::Mat image = cv_ptr->image;
    // Do depth thresholding...
    Mat depth_result = image;
    if (is_depth_mask){
        
        cv::bitwise_and(image, image, depth_result, this->depth_mask);
        // cv::imshow("Depth Thresholding", this->depth_mask);

        // vector<vector<cv::Point> > contours;
        // vector<cv::Vec4i> hierarchy;
        // findContours(this->depth_mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
        // int bottommost = contours[0][0].x;
        // int topmost = contours[0][0].x;
        // int leftmost = contours[0][0].x;
        // int rightmost =  contours[0][0].x;
        // for(int i = 0; i < contours.size(); i ++){
        //   for(int k = 0; k <contours[0].size(); k++){
        //     if(contours[i][k].x > leftmost){
        //       leftmost = contours[i][k].x;
        //     }
        //     if(contours[i][k].x < rightmost){
        //       rightmost = contours[i][k].x;
        //     }
        //     if(contours[i][k].y > bottommost){
        //       bottommost = contours[i][k].y;
        //     }
        //     if(contours[i][k].y > topmost){
        //       topmost = contours[i][k].y;
        //     }
        //   }
        // }
        // image = image(cv::Rect(leftmost, topmost, rightmost-leftmost, bottommost-topmost));
        // cv::imshow("Post Depth Thresholding", image);
    }




    // Do HSV Thresholding
    Mat hsv;
    cvtColor(depth_result, hsv, COLOR_BGR2HSV);
    Mat mask;
    inRange(hsv, Scalar(90,50,70), Scalar(128,255,255), mask);
    Mat result;
    image.copyTo(result, mask);
    // cv::imshow("Post Depgh and hsv", result);

    // vector<vector<cv::Point> > contours;
    // vector<cv::Vec4i> hierarchy;
    // Mat gray;
    // cv::cvtColor(result, gray, CV_BGR2GRAY);
    // Canny(gray, gray, 255/3, 255);
    // findContours(gray, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    // if(contours.size() > 0)
    // {
    // int bottommost = 200;
    // int topmost = 200;
    // int leftmost = 200;
    // int rightmost = 200;
    // for(int i = 0; i < contours.size(); i ++){
    //   for(int k = 0; k <contours[0].size(); k++){
    //     if(contours[i][k].x > 0 && contours[i][k].y > 0 && contours[i][k].x < 512 && contours[i][k].y < 512){
    //       if(contours[i][k].x < leftmost){
    //         leftmost = contours[i][k].x;
    //       }
    //       if(contours[i][k].x > rightmost){
    //         rightmost = contours[i][k].x;
    //       }
    //       if(contours[i][k].y > bottommost){
    //         bottommost = contours[i][k].y;
    //       }
    //       if(contours[i][k].y < topmost){
    //         topmost = contours[i][k].y;
    //       }
    //   }
    //   }
    // }
    // cout << leftmost << " " << topmost << " " << rightmost << " " << bottommost << endl;
    // try
    // {
    //   mask = mask(cv::Rect(leftmost, topmost, rightmost-leftmost, bottommost-topmost));
    // }
    // catch(...)
    // {
    //   cout << "im unhappy" << endl;
    // }
    // Mat drawing = image;
    // // Scalar color = Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
    // for( size_t i = 0; i< contours.size(); i++ )
    // {
    //     Scalar color = Scalar(0, 255, 0);
    //     drawContours( drawing, contours, (int)i, color, 2, LINE_8, hierarchy, 0 );
    // }
    // cv::imshow("Post Depth Thresholding", drawing);
    // }




    //Fill in the mask...
    Mat im_floodfill = mask.clone();
    floodFill(im_floodfill, cv::Point(0, 0), Scalar(255));

    Mat im_floodfill_inv;
    bitwise_not(im_floodfill, im_floodfill_inv);

    Mat im_out = (mask | im_floodfill_inv);




    // Canny Edge Detection
	Mat edges; 
    //Canny(result, edges, 255/3, 255);
    Canny(im_out, edges, 255/3, 255);
    // cv::Mat depth_and_canny = edges;
    // if (is_depth_mask){
    //     // Mat depth_result;
    //     cv::bitwise_and(edges, edges, edges, this->depth_mask);
    // }
    //find the bounding lines
    vector<Vec2f *> myFinalLines;
    myFinalLines = find_bounding_lines(edges);
	if (myFinalLines[0] == nullptr || myFinalLines[1] == nullptr || myFinalLines[2] == nullptr || myFinalLines[3] == nullptr){
        // cv::imshow("source", edges);
        cv::waitKey(3);
        sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
        return;
	}
	Vec2f left = *myFinalLines[0];
    Vec2f top = *myFinalLines[1];
    Vec2f right = *myFinalLines[2];
    Vec2f bottom = *myFinalLines[3];


    // Display lines for debugging purposes
    for(int i = 0; i < myFinalLines.size(); i++)
    {
        // Vec2f r_theta = lines[i];
        float r = (*myFinalLines[i])[0], theta = (*myFinalLines[i])[1];

        double a = cos(theta); 
        double b = sin(theta);
        double x0 = a*r; 
        double y0 = b*r;
        int x1 = int(x0 + 1000*(-b)); 
        int y1 = int(y0 + 1000*(a));
        int x2 = int(x0 - 1000*(-b)); 
        int y2 = int(y0 - 1000*(a));
        line(image, Point(x1, y1), Point(x2, y2), Scalar(0, 0, 255), 2, LINE_8);
    }
    delete myFinalLines[0];
    delete myFinalLines[1];
    delete myFinalLines[2];
    delete myFinalLines[3];

    Point TL_corner = hough_inter(top, left);
    Point TR_corner = hough_inter(top, right);
    Point BL_corner = hough_inter(bottom, left);
    Point BR_corner = hough_inter(bottom, right);
   
    
    circle(image, TL_corner, 10, Scalar(255, 0, 0), FILLED, LINE_8);
    circle(image, TR_corner, 10, Scalar(255, 0, 0), FILLED, LINE_8);
    circle(image, BL_corner, 10, Scalar(255, 0, 0), FILLED, LINE_8);
    circle(image, BR_corner, 10, Scalar(255, 0, 0), FILLED, LINE_8);


    // Plot Center 
    Point center = Point(int((TL_corner.x + TR_corner.x + BL_corner.x + BR_corner.x)/4.0), int((TL_corner.y + TR_corner.y + BL_corner.y + BR_corner.y)/4.0));
    // circle(image, center, 5, Scalar(0, 120, 255), FILLED, LINE_8);


    // Get area
    double area = quadrilateral_area(TL_corner, TR_corner, BR_corner, BL_corner);
    
    // Get offsets
    int height = image.cols;
    int width = image.rows;
    // width, height, _ = image.shape
    Point image_center = Point(int(height/2), int(width/2));
    circle(image, image_center, 10, Scalar(255, 255, 0), FILLED, LINE_8);

     //attempting to filter the points to ony use well spaced out ones, points must be at least 100 away
    int threshold_too_close = 50;
    if(!(abs(TL_corner.x - TR_corner.x) < threshold_too_close || 
    abs(TL_corner.y - BL_corner.y) < threshold_too_close || abs(TR_corner.y - BR_corner.y) < threshold_too_close
    || abs(BL_corner.x - BR_corner.x) < threshold_too_close))
    {
      x_offset = center.x - image_center.x;
      y_offset = center.y - image_center.y;
      circle(image, center, 5, Scalar(0, 120, 255), FILLED, LINE_8);
    }
    // // Get Vanishing Point
    // Point vanish_pt = hough_inter(top, bottom);
    
    // cout << "CONTROL PARAMETERS:";
    // cout << "  quad area: " << to_string(area) << endl;
    // cout << "  x offset: " << to_string(x_offset) << endl;
    // cout << "  y offset: " << to_string(y_offset) << endl;
    // cout << "  vanish pt: (" << vanish_pt.x << "," << vanish_pt.y << ")";


    // Update GUI Window
    cv::imshow("source", image);
    // cv::imshow("masked", mask);

    cv::waitKey(3);
    
    sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(std_msgs::Header(), "mono8", image).toImageMsg();
    // Output modified video stream
    image_pub_.publish(msg_out);
    
  }

    Point hough_inter(Vec2f line1, Vec2f line2){
      float rho1 = line1[0];
      float theta1 = line1[1];
      float rho2 = line2[0];
      float theta2 = line2[1];
      float ct1=cosf(theta1);     //matrix element a
      float st1=sinf(theta1);     //b
      float ct2=cosf(theta2);     //c
      float st2=sinf(theta2);     //d
      float d=ct1*st2-st1*ct2;        //determinative (rearranged matrix for inverse)
      if(d!=0.0f) {   
              int x=(int)((st2*rho1-st1*rho2)/d);
              int y=(int)((-ct2*rho1+ct1*rho2)/d);
              return Point(x, y);
      } else { //lines are parallel and will NEVER intersect!
              return Point(0,0);
      }
  }
  double quadrilateral_area(Point p1,Point p2,Point p3,Point p4)
  {
      return 0.5*((p1.x*p2.y + p2.x*p3.y + p3.x*p4.y + p4.x*p1.y) - (p2.x*p1.y + p3.x*p2.y + p4.x*p3.y + p1.x*p4.y));
  }
  // Given theta in radians
  bool close_to_verticle(float theta)
  {
      // verticle lines are integer multiples of pi/2
      double PI_2 = M_PI / 2.0;
      
      double num = (abs(theta) / PI_2);
      int closest_pi_over_2 = round(num);
      
      double threshold = 0.05;
      if(abs(num - closest_pi_over_2) < threshold)
      {
          // cout << " occored" << endl;
          return true;
      }
      return false;
  }
  // Given theta in radians
  bool close_to_horizontal(float theta)
  {
      // horizontal lines are integer multiples of pi
      double PI = M_PI;
      double num = (abs(theta) / PI);
      int closest_pi = round(num);
      
      double threshold = 0.1;
      if(abs(num - closest_pi) < threshold)
      {
          return true;
      }

      return false;
  }
  // Finds the lines that bound the mast
  // If image parameter is supplied, it will draw the lines on that image
  // returns (r, theta) pairs in order of LTRB
  vector<Vec2f *> find_bounding_lines(Mat edges)
  {
      Vec2f * leftLine = nullptr;
      Vec2f * rightLine = nullptr;
      Vec2f * topLine = nullptr;
      Vec2f * bottomLine = nullptr;
      vector<Vec2f> lines;
      HoughLines(edges, lines, 1, M_PI/180.0, 50.0, 0, 0, 0, M_PI);
      // imwrite("reached.png", edges);

      for(int i = 0; i < lines.size(); i++)
      {
          // Vec2f r_theta = lines[i];
          float r = lines[i][0], theta = lines[i][1];
          // cout << to_string(r) << " " << to_string(theta) << endl;
          // arr = np.array(r_theta[0], dtype=np.float64)
          // r, theta = arr
          
          // check if lines are sortve vertical or sortve horizontal and then see if they beat the extremes
          if(close_to_horizontal(theta)){
                      // cout << to_string(r) << " " << to_string(theta) << endl;
              if(!leftLine){
                  leftLine = new Vec2f(r, theta);}
              else if(abs(r) < abs((*leftLine)[0])){
                  leftLine = new Vec2f(r, theta);}
              
              if(!rightLine){
                  rightLine = new Vec2f(r, theta);}
              else if (abs(r) > abs((*rightLine)[0])){
                  rightLine = new Vec2f(r, theta);}
          }
          else if(close_to_verticle(theta)){
              if(!topLine){
                  topLine = new Vec2f(r, theta);}
              else if(abs(r) < abs((*topLine)[0])){
                  topLine = new Vec2f(r, theta);}
                  
              if(!bottomLine){
                  bottomLine = new Vec2f(r, theta);}
              else if(abs(r) > abs((*bottomLine)[0])){
                  bottomLine = new Vec2f(r, theta);}
          }
      }
      vector<Vec2f *> myFinal;
      myFinal.push_back(leftLine);
      myFinal.push_back(topLine);
      myFinal.push_back(rightLine);
      myFinal.push_back(bottomLine);
      return myFinal;
      // left top right bottom
  }

};
#endif