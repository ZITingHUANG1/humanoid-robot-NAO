//Face detection
// Giovanni Cortigiani, Ziting Huang, Bernhard Glas

/*
 * This module handles the Face detection functions useful for checking the attention of a visitor
 */

#pragma once
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/objdetect.hpp"
#include <vector>
#include <numeric>
#include <cmath>
#include <iostream>

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW_FD = "FaceDetection Top Camera";

//order comparison for the set of shirt coulors
struct Fmp{
    bool operator()(const Vec3b& a, const Vec3b& b){
        if(int(a[0])!= int(b[0])){
            return   int(a[0]) < int(b[0]);
        }
        else if(int(a[1])!= int(b[1])){
            return   int(a[1]) < int(b[1]);
        }
        else{
            return   int(a[2]) < int(b[2]);
        }
    }
};

class FaceDetection
{

private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;


    // image member variables
    Mat frame;
    Mat frame_gray;

    // face detection member variables
    CascadeClassifier face_cascade;
    std::vector<Rect> faces;

    int count;
    int count_max;


public:


    FaceDetection()
        : it_(nh_)
    {
        // Subscribe to input video feed from top camera
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
                                   &FaceDetection::imageCb1, this);

//        cv::namedWindow(OPENCV_WINDOW_FD);

        String face_cascade_name = "/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_frontalface_alt.xml";

        // Load the cascades
        if( !face_cascade.load( face_cascade_name ) )
        {
            cout << "--(!)Error loading face cascade\n";

        };
        count = 0;
        ros::param::get("/Naovid/facedetection/count_max", count_max);
    }


    ~FaceDetection()
    {
//        cv::destroyWindow(OPENCV_WINDOW_FD);
    }

    //handle top camera images
    void imageCb1(const sensor_msgs::ImageConstPtr& msg)
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

        frame = cv_ptr->image;

        // Update GUI Window
//        cv::imshow(OPENCV_WINDOW_FD, cv_ptr->image);
        cv::waitKey(3);
    }


    //Face Detection
    void detectAndDisplay(Mat frame)
    {
        cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
        equalizeHist( frame_gray, frame_gray );

        //-- Detect faces
        face_cascade.detectMultiScale( frame_gray, faces );
        for ( size_t i = 0; i < faces.size(); i++ )
        {
            Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
            ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4 );
            Mat faceROI = frame_gray( faces[i] );
        }
    }

    // This function returns true if a face is detected, false otherwise
    bool checkAttention(){

        detectAndDisplay(frame);

        //        if (faces.empty()) count++;
        //        else count = 0;

        //        if (count > count_max) return false;
        //        else return true;

        if (faces.empty()) return false;

        else return true;


    }

};

