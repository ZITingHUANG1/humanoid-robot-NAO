//Aruco detection and walking
// Giovanni Cortigiani, Ziting Huang, Bernhard Glas

/*
 * This module contains the functions related to the aruco detection and walking towards it.
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
#include <opencv2/aruco.hpp>
#include <opencv2/imgcodecs.hpp>
#include "Naovid/Aruco.h"
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include <string.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeed.h>
#include <naoqi_bridge_msgs/Bumper.h>
#include <naoqi_bridge_msgs/HeadTouch.h>
#include <naoqi_bridge_msgs/HandTouch.h>
#include <naoqi_bridge_msgs/JointAnglesWithSpeedAction.h>
#include <std_srvs/Empty.h>
#include <boost/algorithm/string.hpp>
#include <boost/date_time.hpp>
#include <naoqi_bridge_msgs/SpeechWithFeedbackActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <naoqi_bridge_msgs/SetSpeechVocabularyActionGoal.h>
#include <std_msgs/ColorRGBA.h>
#include <naoqi_bridge_msgs/WordRecognized.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include "actionlib/client/simple_action_client.h"

using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW_AM = "Aruco Marker Top Camera";

class ArucoMarker
{

private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    Mat frame;
    double distance_x;
    double distance_y;

    // Publisher to nao walking
    ros::Publisher walk_pub;

    // Client for stoping walk
    ros::ServiceClient walk_stop_srv;

    // Client for initiate pose for walking
    ros::ServiceClient needs_start_walk_pose_srv;

    // Subscriber for foot contact
    ros::Subscriber footContact_sub;

    //member variables for aruco detection
    double marker_torso[3];  //contains the position of the aruco marker with id 99 with respect to torso frame

    //member variables for walk
    bool m_foot_contact = false; //true when the foot is in contact with the floor

public:

    ArucoMarker()
        : it_(nh_)
    {
        // Subscribe to input video feed from top camera
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
                                   &ArucoMarker::imageCb1, this);

        walk_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_walk_srv");

        needs_start_walk_pose_srv=nh_.serviceClient<std_srvs::Empty>("/needs_start_walk_pose_srv");

        walk_pub=nh_.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);

        ros::param::get("/Naovid/aruco/distance_x", distance_x);
        ros::param::get("/Naovid/aruco/distance_y", distance_y);

//        cv::namedWindow(OPENCV_WINDOW_AM);
    }

    ~ArucoMarker()
    {
//        cv::destroyWindow(OPENCV_WINDOW_AM);
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
     /*   cv::imshow(OPENCV_WINDOW_AM, cv_ptr->image);
        cv::waitKey(3);*/

    }

    //footContactCallback function
    void footContactCallback(const std_msgs::BoolConstPtr& contact)
    {
        m_foot_contact = contact->data;
    }



    //Aruco detection functions
    bool Arucodetection(){

        //Detecting Markers
        Mat inputImage = frame;
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

        //Draw detected markers
        cv::Mat outputImage = inputImage.clone();
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
        cv::imshow("arucoid", outputImage);
        cv::waitKey(3);

        //SETTING PARAMETERS
        float datad[5] = {-0.0481869853715082,  0.0201858398559121,
                           0.0030362056699177, -0.00172241952442813, 0};
        float datac[9] = {278.236008818534, 0,                156.194471689706,
                          0,                279.380102992049, 126.007123836447,
                          0,                0,                1};

        cv::Mat cameraP(3,3,CV_32FC1, datac);
        cv::Mat dist(5,1,CV_32FC1, datad);

        //camera parameters
        cv::Mat cameraMatrix, distCoeffs;

        cameraMatrix = cameraP;
        distCoeffs = dist;

        //Estimate Pose Single Markers
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.085, cameraMatrix, distCoeffs, rvecs, tvecs);

        // Computation of aruco positions with respect to torso frame if we find aruco Marker 99
        if (!markerIds.empty()){
            ros::ServiceClient aruco_client = nh_.serviceClient<Naovid::Aruco>("aruco");
            Naovid::Aruco srv_aruco;

            for(int i=0; i<markerIds.size(); i++) {

                if(markerIds[i] == 99){

//                    cout << "marker id = 99" << endl;
                    srv_aruco.request.markerx = (double)tvecs[i][0];
                    srv_aruco.request.markery = (double)tvecs[i][1];
                    srv_aruco.request.markerz = (double)tvecs[i][2];

//                    cout << (double)tvecs[i][0] << (double)tvecs[i][1] << (double)tvecs[i][2] << endl;



                    if (aruco_client.call(srv_aruco))
                    {
                        ROS_INFO("working");
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service aruco");
                    }


                    marker_torso[0] = srv_aruco.response.markertorsox;
                    marker_torso[1] = srv_aruco.response.markertorsoy;
                    marker_torso[2] = srv_aruco.response.markertorsoz;
                    cout << marker_torso[0] <<endl;
                    cout << marker_torso[1] <<endl;
                    cout << marker_torso[2] <<endl;

                    return true;
                }
            }
        }
        return false;
    }


    bool checkArucoDistance(){
        //Detecting Markers
        Mat inputImage = frame;
        std::vector<int> markerIds;
        std::vector<std::vector<cv::Point2f> > markerCorners;
        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        cv::aruco::detectMarkers(inputImage, dictionary, markerCorners, markerIds);

        //Draw detected markers
        cv::Mat outputImage = inputImage.clone();
        cv::aruco::drawDetectedMarkers(outputImage, markerCorners, markerIds);
//        cv::imshow("arucoid", outputImage);
//        cv::waitKey(3);

        //SETTING CAMERA sPARAMETERS
        cv::Mat cameraMatrix, distCoeffs;
        float datad[5] = {-0.0481869853715082,  0.0201858398559121,
                           0.0030362056699177, -0.00172241952442813, 0};
        float datac[9] = {278.236008818534, 0,                156.194471689706,
                          0,                279.380102992049, 126.007123836447,
                          0,                0,                1};
        cv::Mat cameraP(3,3,CV_32FC1, datac);
        cv::Mat dist(5,1,CV_32FC1, datad);
        cameraMatrix = cameraP;
        distCoeffs = dist;

        //Estimate Pose Single Markers
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.085, cameraMatrix, distCoeffs, rvecs, tvecs);

        // Computation of aruco positions with respect to torso frame if we find aruco Marker 99
        if (!markerIds.empty()){
            ros::ServiceClient aruco_client = nh_.serviceClient<Naovid::Aruco>("aruco");
            Naovid::Aruco srv_aruco;

            for(int i=0; i<markerIds.size(); i++) {

                if(markerIds[i] == 99){

//                    cout << "marker id = 99" << endl;
                    srv_aruco.request.markerx = (double)tvecs[i][0];
                    srv_aruco.request.markery = (double)tvecs[i][1];
                    srv_aruco.request.markerz = (double)tvecs[i][2];



                    if (aruco_client.call(srv_aruco))
                    {
                        ROS_INFO("working");
                    }
                    else
                    {
                        ROS_ERROR("Failed to call service aruco");
                    }


                    marker_torso[0] = srv_aruco.response.markertorsox;

                    cout << "Aruco distance: " << marker_torso[0] <<endl;

                    if (marker_torso[0] < 1.0){
                        cout << "walk stopped in aruco" << endl;
                        stopWalk();
                        return true;
                    }
                }
            }
        }
        return false;

    }

    //walk functions

    //Stop Walking
    void stopWalk()
    {
        std_srvs::Empty srv;
        walk_stop_srv.call(srv);
    }

    // Walking intialisation
    void initWalk(){

        if (m_foot_contact){
                    std_srvs::Empty srv;
                    needs_start_walk_pose_srv.call(srv);
                    cout << "inside initiate walking pose condition" << endl;
                }
    }

    // Walks towards Aruco marker
    void walker()
    {
        if(m_foot_contact){
            //input x and y in meteres and theta in radians
            geometry_msgs::Pose2D msg_walk;
            msg_walk.x = marker_torso[0] - distance_x;
            msg_walk.y = marker_torso[1] + distance_y;
            msg_walk.theta = 0;
            walk_pub.publish(msg_walk);
            cout << "walking" << endl;
        }
        else{
            stopWalk();
            cout << "walk stopped" <<endl;
        }
    }

};

