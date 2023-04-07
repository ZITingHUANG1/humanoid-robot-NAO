//Shirt detection
// Giovanni Cortigiani, Ziting Huang, Bernhard Glas

// This file can be compiled as an executable to check the functionality of the shirtdetection


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/video/video.hpp>
#include "opencv2/objdetect.hpp"
#include <iostream>


using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW = "Shirtdetection Top Camera";

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
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    CascadeClassifier face_cascade;
    CascadeClassifier shirt_cascade;
    //shirt counter
    int count;
    //set of shirts seen
    set<Vec3b,Cmp> shirts;


public:


    ShirtDetection()
        : it_(nh_)
    {
        // Subscribe to input video feed from top camera
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
                                   &ShirtDetection::imageCb1, this);

        cv::namedWindow(OPENCV_WINDOW);

        //count initialisation
        count = 0;
    }



    ~ShirtDetection()
    {
        cv::destroyWindow(OPENCV_WINDOW);
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


        Mat frame = cv_ptr->image;

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

        //Face and Shirt Detection
        detectAndDisplay(frame);


        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);



    }


    //Face and Shirt Detection
    void detectAndDisplay(Mat frame)
    {
        Mat frame_gray;
        cvtColor( frame, frame_gray, COLOR_BGR2GRAY );
        equalizeHist( frame_gray, frame_gray );
        //-- Detect faces
        /*   std::vector<Rect> faces;
        face_cascade.detectMultiScale( frame_gray, faces );
        for ( size_t i = 0; i < faces.size(); i++ )
        {
            Point center( faces[i].x + faces[i].width/2, faces[i].y + faces[i].height/2 );
            ellipse( frame, center, Size( faces[i].width/2, faces[i].height/2 ), 0, 0, 360, Scalar( 255, 0, 255 ), 4 );
            Mat faceROI = frame_gray( faces[i] );
            //cv::imshow("face", faceROI);

        }*/
        //-- Detect shirts
        std::vector<Rect> shirtsdetected;
        shirt_cascade.detectMultiScale( frame_gray, shirtsdetected );
        for ( size_t i = 0; i < shirtsdetected.size(); i++ )
        {
            Point center(shirtsdetected[i].x + shirtsdetected[i].width/2, shirtsdetected[i].y + shirtsdetected[i].height );
            ellipse( frame, center, Size( shirtsdetected[i].width/2, shirtsdetected[i].height/2 ), 0, 0, 360, Scalar( 0, 0, 255 ), 4 );
            Mat shirtsdetectedROI = frame_gray( shirtsdetected[i] );
            cv::imshow("shirtsdetected", shirtsdetectedROI);

            //GetColours of shirts
            Vec3b shirtcolours = frame.at<Vec3b>(center.y,center.x);
            cout << "B: " << int(shirtcolours[0]) << endl;
            cout << "G: " << int(shirtcolours[1]) << endl;
            cout << "R: " << int(shirtcolours[2]) << endl;

            //Shirt update
            shirts.insert(shirtcolours);
            count ++;
            cout << "shirt number " << count << " inserted" <<endl;


        }
    }

    // Print shirts seen
    void PrintShirts(){
        int i = 1;
        cout << "Printing shirts" << endl;
        set<Vec3b,Cmp>::const_iterator iter=shirts.begin();
        for(; iter!=shirts.end(); iter++){
            cout << i++;
            cout << " B: " << int((*iter)[0]) << " G: " << int((*iter)[1]) << " R: " << int((*iter)[2]) <<endl;

        }

    }



};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "shirt_detection");
    ShirtDetection ic;
    ros::spin();
    //Prints when exiting
    ic.PrintShirts();
    return 0;
}


