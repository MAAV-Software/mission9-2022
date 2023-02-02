/**
**  Simple ROS Node
**/
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "video_pub_cpp");
    ros::NodeHandle nh;  // Default handler for nodes in ROS
 
    // 0 reads from your default camera
    const int CAMERA_INDEX = 0;
    cv::VideoCapture capture(CAMERA_INDEX); 
    if (!capture.isOpened()) {
      ROS_ERROR_STREAM("Failed to open camera with index " << CAMERA_INDEX << "!");
      ros::shutdown();
    }
     
    // Image_transport is responsible for publishing and subscribing to Images
    image_transport::ImageTransport it(nh);
     
    // Publish to the /camera topic
    image_transport::Publisher pub_frame = it.advertise("camera", 1);
     
    cv::Mat frame;//Mat is the image class defined in OpenCV
    sensor_msgs::ImagePtr msg;
 
    ros::Rate loop_rate(10);
 
    while (nh.ok()) {
 
      // Load image
      capture >> frame; 
    
      // Check if grabbed frame has content
      if (frame.empty()) {
        ROS_ERROR_STREAM("Failed to capture image!");
        ros::shutdown();
      }
 
      // Convert image from cv::Mat (OpenCV) type to sensor_msgs/Image (ROS) type and publish
      msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
      pub_frame.publish(msg);
      /*
      Cv_bridge can selectively convert color and depth information. In order to use the specified
      feature encoding, there is a centralized coding form:
        Mono8: CV_8UC1, grayscale image
        Mono16: CV_16UC1, 16-bit grayscale image
        Bgr8: CV_8UC3 with color information and the order of colors is BGR order
        Rgb8: CV_8UC3 with color information and the order of colors is RGB order
        Bgra8: CV_8UC4, BGR color image with alpha channel
        Rgba8: CV_8UC4, CV, RGB color image with alpha channel
      */
      //cv::imshow("camera", image);
      cv::waitKey(1); // Display image for 1 millisecond
 
      ros::spinOnce();
      loop_rate.sleep();
    }  
 
    // Shutdown the camera
    capture.release();
}