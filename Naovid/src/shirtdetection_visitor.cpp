//Shirt detection
// Giovanni Cortigiani, Ziting Huang, Bernhard Glas

/*
 * This module contains the functions related to the visitor counter subsystem, including
 * visitor detection (understanding his direction), shirt color detection and visitor counter.
 *
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

static const std::string OPENCV_WINDOW_VC = "Visitor Count Top Camera";

//order comparison for the set of shirt coulors
struct Cmp{
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

class ShirtDetection
{

private:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    CascadeClassifier face_cascade;
    CascadeClassifier shirt_cascade;

    Mat frame;
    Mat saved_frame;
    Mat frame_gray;

    //shirt counter
    int count;

    //set of shirts seen
    set<Vec3b,Cmp> shirts;

    //vector of detected faces
    std::vector<Rect> faces;
    std::vector<Rect> faces_saved;

    //set of shirts saved in wait state
    set<Vec3b,Cmp> saved_shirts;

    std::vector<Rect> shirtsdetected = {};
    Rect focus_shirt;
    Rect focus_shirt_first;
    std::vector<int> focus_shirtcolour;
    std::vector<int> focus_size_check;
    bool shirt_already_seen; //true, if shirt was already seen
    bool entering; //true, if visitor is coming closer to the robot, false for disappearing
    bool gradient_det; //true, if focus shirt has moved enough between exit and robot

    // variables
    int focus_count = 0; //count, how many times the shirt under consideration is seen
    int focus_count_other = 0; //count, how many no shirts or shirts are seen which are not under consideration
    int focus_shirt_count_limit; //limit for the focus count
    int focus_count_other_limit; //limit for the focus count other
    int th_col; //color threshold for the shirt
    int th_width; //threshold to avoid detecting other upper bodies in the environment
    int th_distance; //minimum threshold to check, if entering or leaving (minimum differnce of rect.width)



public:


    ShirtDetection()
        : it_(nh_)
    {
        // Subscribe to input video feed from top camera
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
                                   &ShirtDetection::imageCb1, this);

//        cv::namedWindow(OPENCV_WINDOW_VC);

        String face_cascade_name = "/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_frontalface_alt.xml";
        String shirt_cascade_name = "/opt/ros/kinetic/share/OpenCV-3.3.1-dev/haarcascades/haarcascade_upperbody.xml";

        // Load the cascades
        if( !face_cascade.load( face_cascade_name ) )
        {
            cout << "--(!)Error loading face cascade\n";

        };

        if( !shirt_cascade.load( shirt_cascade_name ) )
        {
            cout << "--(!)Error loading face cascade\n";

        };


        //variables initialisation
        count = 0;
        focus_shirtcolour.push_back(0);
        focus_shirtcolour.push_back(0);
        focus_shirtcolour.push_back(0);
        shirt_already_seen = false;
        entering = false;

        //setting the parameters
        ros::param::get("/Naovid/shirtdetection/focus_count_other_limit", focus_count_other_limit);
        ros::param::get("/Naovid/shirtdetection/focus_shirt_count_limit", focus_shirt_count_limit);
        ros::param::get("/Naovid/shirtdetection/th_col_shirts", th_col);
        ros::param::get("/Naovid/shirtdetection/th_width_shirts", th_width);
        ros::param::get("/Naovid/shirtdetection/th_distance", th_distance);
    }


    ~ShirtDetection()
    {
//        cv::destroyWindow(OPENCV_WINDOW_VC);
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

        Mat frame_temp = cv_ptr->image;
        if (!frame_temp.empty()) frame = cv_ptr->image;


        // Update GUI Window
//        cv::imshow(OPENCV_WINDOW_VC, cv_ptr->image);
        cv::waitKey(3);
    }


    //Shirt Detection
    void detectAndDisplay(Mat frame)
    {
        cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
        equalizeHist( frame_gray, frame_gray );

        //-- Detect shirts
        shirt_cascade.detectMultiScale( frame_gray, shirtsdetected );
        for ( size_t i = 0; i < shirtsdetected.size(); i++ )
        {
            Point center(shirtsdetected[i].x + shirtsdetected[i].width/2, shirtsdetected[i].y + shirtsdetected[i].height );
            ellipse( frame, center, Size( shirtsdetected[i].width/2, shirtsdetected[i].height/2 ), 0, 0, 360, Scalar( 0, 0, 255 ), 4 );
            Mat shirtsdetectedROI = frame_gray( shirtsdetected[i] );
            cv::imshow("shirtsdetected", shirtsdetectedROI);

            //Get Colours of shirts
            Vec3b shirtcolours = frame.at<Vec3b>(center.y,center.x);
//            cout << "B: " << int(shirtcolours[0]) << endl;
//            cout << "G: " << int(shirtcolours[1]) << endl;
//            cout << "R: " << int(shirtcolours[2]) << endl;

            //Shirt update
            shirts.insert(shirtcolours);
            count ++;
        }
    }

    //Face Detection
    void detectAndDisplayFace(Mat frame)
    {
        cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
        equalizeHist( frame_gray, frame_gray );
        faces.clear();

        //-- Detect faces
        face_cascade.detectMultiScale( frame_gray, faces );
        for ( size_t i = 0; i < faces.size(); i++ )
        {
            Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
            ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4 );
            Mat faceROI = frame_gray( faces[i] );
            cout << "face_detected" << endl;
        }
    }

    // Print shirts seen
    void printShirts(){
        int i = 1;
        cout << "Printing shirts" << endl;
        set<Vec3b,Cmp>::const_iterator iter=shirts.begin();
        for(; iter!=shirts.end(); iter++){
            cout << i++;
            cout << " B: " << int((*iter)[0]) << " G: " << int((*iter)[1]) << " R: " << int((*iter)[2]) <<endl;
        }
    }

    // Check if a visitor is leaving or entering
    bool check_wait_state(){
        if (frame.empty()) return false;

        shirt_already_seen = false;
        gradient_det = true;

        //update the shirtsdetected Rect vector
        detectAndDisplay(frame);

        cout << "focus count: " << focus_count << endl;

        if (shirtsdetected.empty()){
            cout << "no shirts detetected" << endl;
            focus_count_other++;
            return false;
        }

        // only entering the first time it sees a shirt to focus on
        if (focus_count == 0){
            focus_shirt_first = shirtsdetected[0];
            focus_shirt = shirtsdetected[0];
        }

        // if other stuff was detected too often, reset the detect counters
        if (focus_count_other > focus_count_other_limit){
            focus_count_other = 0;
            focus_count = 0;
        }

        // if focus_shirt is in similar range of the detected shirt, update variables
        if (shirtsdetected[0].x+th_width > focus_shirt.x && shirtsdetected[0].x-th_width < focus_shirt.x) {

            // Calculate average focus shirt colour
            Point center(shirtsdetected[0].x + shirtsdetected[0].width/2, shirtsdetected[0].y + shirtsdetected[0].height );
            Vec3b shirtcolours = frame.at<Vec3b>(center.y,center.x);
            focus_shirtcolour[0] = ((focus_shirtcolour[0]*focus_count + (int)shirtcolours[0])/(focus_count+1));
            focus_shirtcolour[1] = ((focus_shirtcolour[1]*focus_count + (int)shirtcolours[1])/(focus_count+1));
            focus_shirtcolour[2] = ((focus_shirtcolour[2]*focus_count + (int)shirtcolours[2])/(focus_count+1));

            // UNSTABLE // Check, if focus_shirt is in-/ or decreasing // UNSTABLE
//            if (focus_shirt.width < shirtsdetected[0].width) focus_size_check.push_back(1);
//            else focus_size_check.push_back(-1);

            // Update variables
            focus_shirt = shirtsdetected[0];
            focus_count++;
            focus_count_other = 0;
        }

        else { // other stuff was detected
            focus_count_other++;
        }

        //if enough iterations went well (visitor recognized: entering or leaving)
        if (focus_count > focus_shirt_count_limit){

            // setting gradient_det to false, if distance wasn't enough
            if (abs(focus_shirt_first.width-focus_shirt.width) < th_distance){
                gradient_det = false;
            }
            // UNSTABLE version
            // if average of focus_size_check is bigger 0, focus shirt enters
            // if (std::accumulate(focus_size_check.begin(), focus_size_check.end(), 0LL) / focus_size_check.size() > 0){

            // OR just compare width with first detected focus shirt

            //enters, if the visitor is entering
            if (focus_shirt_first.width < focus_shirt.width){

                //check if shirt colour was already seen
                if (saved_shirts.empty()) {
                    shirt_already_seen = false;
                }
                else {
                    //check, if shirt colours is already in set saved_shirts
                    set<Vec3b,Cmp>::const_iterator iter=saved_shirts.begin();
                    for(; iter!=saved_shirts.end(); iter++){
                        if (int((*iter)[0])+th_col > focus_shirtcolour[0] && int((*iter)[0])-th_col < focus_shirtcolour[0] &&
                                int((*iter)[1])+th_col > focus_shirtcolour[1] && int((*iter)[1])-th_col < focus_shirtcolour[1] &&
                                int((*iter)[2])+th_col > focus_shirtcolour[2] && int((*iter)[2])-th_col < focus_shirtcolour[2]){

                            shirt_already_seen = true;
                        }
                    }
                }
                entering = true;
            }
            // else the visitor leaves
            else {
                //check if shirt colour was already seen
                set<Vec3b,Cmp>::const_iterator iter=saved_shirts.begin();
                for(; iter!=saved_shirts.end(); iter++){
                    if (int((*iter)[0])+th_col > focus_shirtcolour[0] && int((*iter)[0])-th_col < focus_shirtcolour[0] &&
                            int((*iter)[1])+th_col > focus_shirtcolour[1] && int((*iter)[1])-th_col < focus_shirtcolour[1] &&
                            int((*iter)[2])+th_col > focus_shirtcolour[2] && int((*iter)[2])-th_col < focus_shirtcolour[2]){

                        shirt_already_seen = true;
                    }
                }
                entering = false;
            }

            focus_count = 0;
            return true;
        }

        return false;
    }

    bool check_wait_state_face(){
        if (frame.empty()) return false;

        shirt_already_seen = false;
        gradient_det = true;

        //update the shirtsdetected and facedetected Rect vector
        detectAndDisplay(frame);
        detectAndDisplayFace(frame);

        if (!faces.empty()) {
            faces_saved.push_back(faces[0]);
            cout << "face added to faces_saved" << endl;
        }

        cout << "focus count: " << focus_count << endl;

        if (shirtsdetected.empty()){
            cout << "no shirts detetected" << endl;
            focus_count_other++;
            return false;
        }

        // only entering the first time it sees a shirt to focus on
        if (focus_count == 0){
            focus_shirt_first = shirtsdetected[0];
            focus_shirt = shirtsdetected[0];
        }

        // if other stuff was detected too often, reset the detect counters
        if (focus_count_other > focus_count_other_limit){
            focus_count_other = 0;
            focus_count = 0;
            faces_saved.clear();
        }

        // if focus_shirt is in similar range of the detected shirt, update variables
        if (shirtsdetected[0].x+th_width > focus_shirt.x && shirtsdetected[0].x-th_width < focus_shirt.x) {

            // Calculate average focus shirt colour
            Point center(shirtsdetected[0].x + shirtsdetected[0].width/2, shirtsdetected[0].y + shirtsdetected[0].height );
            Vec3b shirtcolours = frame.at<Vec3b>(center.y,center.x);
            focus_shirtcolour[0] = ((focus_shirtcolour[0]*focus_count + (int)shirtcolours[0])/(focus_count+1));
            focus_shirtcolour[1] = ((focus_shirtcolour[1]*focus_count + (int)shirtcolours[1])/(focus_count+1));
            focus_shirtcolour[2] = ((focus_shirtcolour[2]*focus_count + (int)shirtcolours[2])/(focus_count+1));

            // UNSTABLE // Check, if focus_shirt is in-/ or decreasing // UNSTABLE
//            if (focus_shirt.width < shirtsdetected[0].width) focus_size_check.push_back(1);
//            else focus_size_check.push_back(-1);

            // Update variables
            focus_shirt = shirtsdetected[0];
            focus_count++;
            focus_count_other = 0;
        }

        else { // other stuff was detected
            focus_count_other++;
        }

        //if enough iterations went well (visitor recognized: entering or leaving)
        if (focus_count > focus_shirt_count_limit){

            // setting gradient_det to false, if distance wasn't enough

                gradient_det = true;
                entering = false;

            // UNSTABLE version
            // if average of focus_size_check is bigger 0, focus shirt enters
            // if (std::accumulate(focus_size_check.begin(), focus_size_check.end(), 0LL) / focus_size_check.size() > 0){

            // OR just compare width with first detected focus shirt

            //enters, if the visitor is entering (faces detected)
            if (faces_saved.size()>1){

                //check if shirt colour was already seen
                if (saved_shirts.empty()) {
                    shirt_already_seen = false;
                }
                else {
                    //check, if shirt colours is already in set saved_shirts
                    set<Vec3b,Cmp>::const_iterator iter=saved_shirts.begin();
                    for(; iter!=saved_shirts.end(); iter++){
                        if (int((*iter)[0])+th_col > focus_shirtcolour[0] && int((*iter)[0])-th_col < focus_shirtcolour[0] &&
                                int((*iter)[1])+th_col > focus_shirtcolour[1] && int((*iter)[1])-th_col < focus_shirtcolour[1] &&
                                int((*iter)[2])+th_col > focus_shirtcolour[2] && int((*iter)[2])-th_col < focus_shirtcolour[2]){

                            shirt_already_seen = true;
                        }
                    }
                }
                entering = true;
            }
            // else the visitor leaves
            else {
                //check if shirt colour was already seen
                set<Vec3b,Cmp>::const_iterator iter=saved_shirts.begin();
                for(; iter!=saved_shirts.end(); iter++){
                    if (int((*iter)[0])+th_col > focus_shirtcolour[0] && int((*iter)[0])-th_col < focus_shirtcolour[0] &&
                            int((*iter)[1])+th_col > focus_shirtcolour[1] && int((*iter)[1])-th_col < focus_shirtcolour[1] &&
                            int((*iter)[2])+th_col > focus_shirtcolour[2] && int((*iter)[2])-th_col < focus_shirtcolour[2]){

                        shirt_already_seen = true;
                    }
                }
                entering = false;
            }

            focus_count = 0;
            faces_saved.clear();
            saved_frame = frame;
            return true;
        }

        return false;
    }

    // This function saves the shirt colour of a visitor entering inside the building (if not already seen)
    void save_shirt_colour(){
        // if shirt colour was not seen before, add the shirt colour to saved shirts
        if (!shirt_already_seen){
            Point center(focus_shirt.x + focus_shirt.width/2, focus_shirt.y + focus_shirt.height );
            Vec3b shirtcolours = frame.at<Vec3b>(center.y,center.x);
            shirtcolours[0] = (unsigned char)focus_shirtcolour[0];
            shirtcolours[1] = (unsigned char)focus_shirtcolour[1];
            shirtcolours[2] = (unsigned char)focus_shirtcolour[2];
            saved_shirts.insert(shirtcolours);
            cout << "Shirt saved" << endl;
            cout << "B: " << int(shirtcolours[0]) << endl;
            cout << "G: " << int(shirtcolours[1]) << endl;
            cout << "R: " << int(shirtcolours[2]) << endl;
        }
    }

    // Class Getters

    bool get_shirt_already_seen(){
        return shirt_already_seen;
    }

    bool get_entering(){
        return entering;
    }

    bool get_gradient_det(){
        return gradient_det;
    }

    void set_entering(bool entering_val){
        entering = entering_val;
    }

};

