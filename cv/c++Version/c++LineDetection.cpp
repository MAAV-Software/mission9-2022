// #include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <tuple>
#include <cstdlib>
#include <math.h>
#include <string>
// #include <opencv2/highgui/highgui.hpp>
// #include "opencv2/imgproc/imgproc.hpp"
// #include <fstream> 
// #include <opencv2/videoio.hpp>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
// #include <opencv2/calib3d/calib3d.hpp>
// #include <opencv2/video/tracking.hpp>
// #include <opencv2/video/background_segm.hpp>

using namespace std;
using namespace cv;

Point hough_inter(Vec2f line1, Vec2f line2)
{//https://stackoverflow.com/questions/383480/intersection-of-two-lines-defined-in-rho-theta-parameterization/383527#383527
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
    // auto A = np.array([[math.cos(theta1), math.sin(theta1)], 
    //               [math.cos(theta2), math.sin(theta2)]]);
    // auto b = np.array([rho1, rho2]);
    // ans = np.linalg.lstsq(A, b, rcond=None)[0];
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
            // cout << to_string(theta) << endl;
        // cout << " occccored" << endl;

        return true;
    }
        // cout << to_string(theta) << endl;

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
    HoughLines(edges, lines, 1, M_PI/180.0, 60.0);
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
Mat algorithm(int frame)
{
//     int frame = 25;
    string fileName = "mast_imgs/frame_"+to_string(frame)+".jpg";

    // string fileName = "mast_imgs/frame_68.jpg";

    Mat image = imread(fileName, IMREAD_COLOR);
    // if(image.empty())
    // {
    //     std::cout << "Could not read the image: " << fileName << std::endl;
    //     return image;
    //     // return 1;
    // }
    // Rect R(Point(300, 300), Point(200, 800));
    // image = image(R);
    image = image(Range(300, image.rows-200), Range(800, image.cols-300));
    // image = image[300:-200, 800:-300, :] 
    //#simulatin depth filter of the camera
    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, gray, Size(5, 5), 0);


    // Do HSV Thresholding
    Mat hsv;
    cvtColor(image, hsv, COLOR_BGR2HSV);
    // lower_blue = np.array([60,35,140])
    // upper_blue = np.array([190,255,255])
    Mat mask;
    inRange(hsv, Scalar(60,35,140), Scalar(190,255,255), mask);
    Mat result;
    image.copyTo(result, mask);
    // Mat result = cv2.bitwise_and(image, image, mask = mask)

    // Canny Edge Detection
    Mat edges; 
    Canny(result, edges, 255/3, 255);
    // imwrite("edges.png", edges);
    vector<Vec2f *> myFinalLines;
    myFinalLines = find_bounding_lines(edges);
    // imwrite("reached.png", edges);

    Vec2f left = *myFinalLines[0];
    Vec2f top = *myFinalLines[1];
    Vec2f right = *myFinalLines[2];
    Vec2f bottom = *myFinalLines[3];
    // cout << myFinalLines.size();
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
    circle(image, center, 5, Scalar(0, 120, 255), FILLED, LINE_8);
    
    // Get area
    double area = quadrilateral_area(TL_corner, TR_corner, BR_corner, BL_corner);
    
    // Get offsets
    int height = image.cols;
    int width = image.rows;
    // width, height, _ = image.shape
    Point image_center = Point(int(height/2), int(width/2));
    circle(image, image_center, 10, Scalar(255, 255, 0), FILLED, LINE_8);
    
    int x_offset = center.x - image_center.x;
    int y_offset = center.y - image_center.y;
    
    // Get Vanishing Point
    Point vanish_pt = hough_inter(top, bottom);
    
    cout << "CONTROL PARAMETERS:";
    cout << "  quad area: " << to_string(area) << endl;
    cout << "  x offset: " << to_string(x_offset) << endl;
    cout << "  y offset: " << to_string(y_offset) << endl;
    cout << "  vanish pt: (" << vanish_pt.x << "," << vanish_pt.y << ")";
    
    return image;
}
int main()
{
    Mat image = algorithm(68);
    imwrite("detection.png", image);
}