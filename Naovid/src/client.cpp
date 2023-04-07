// Project Main
// Giovanni Cortigiani (ge45nab), Ziting Huang (ge95qus), Bernhard Glas (ge34ceh)

/*
 * This module is the main block of the project.
 * After the initialisation of all the submodules, the code enters into the main loop, containing
 * the state machine through which NAOvid-19 achieves his work of shop-assistant, exploiting the submodules.
 *
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <string.h>
#include  <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/videoio/videoio.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/imgcodecs.hpp>
#include "Naovid/AniSpeech.h"
#include "Naovid/Standup.h"
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
#include <Naovid/BlinkAction.h>

#include <zbar.h>
#include <SFML/Graphics.hpp>

//Subsystems
#include "covid_check.cpp"
#include "teleoperation.cpp"
#include "shirtdetection_visitor.cpp"
#include "facedetection.cpp"
#include "speech.cpp"
#include "arucomarker.cpp"

using namespace std;
using namespace cv;
using namespace zbar;

static const std::string OPENCV_WINDOW = "Bottom Camera QR Check";

class Client_Naovid
{

private:

    // member variables
    Mat m_frame; //Image captured by the camera
    int m_visitor_count = 0; //count the number of visitors inside the building
    int m_visitor_max; // maximum number of visitors
    bool m_check_attention; //If true NAovid checks attention, if false not
    bool person_status; //true if the person can enter, false otherwise
    bool firstw;

    // ros handler
    ros::NodeHandle nh_;

    // subscriber to joint states
    ros::Subscriber sensor_data_sub;

    // subscriber to top camera
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    // Speech system
    bool m_first_it; //true if it's the first iteration, false otherwise
    SpeechSystem ss;

    // Subscriber to speech recognition
    ros::Subscriber recog_sub;

    //member variable to tell the robot to keep or stop listening
    bool loop;

    // Subscriber to head tactile states
    ros::Subscriber tactile_sub;

    // Aruco system
    ArucoMarker am;
    bool arucosearch; //if true NAOvid searches for the aruco,

    // Subscriber for foot contact
    ros::Subscriber footContact_sub;
    bool walk;

public:

    Client_Naovid()
        : it_(nh_)
    {
        // subscribe to joint states
        sensor_data_sub=nh_.subscribe("/joint_states",1, &Client_Naovid::sensorCallback, this);

        //subscribe to bottom camera
        image_sub_ = it_.subscribe("/nao_robot/camera/bottom/camera/image_raw", 1, &Client_Naovid::imageCb1, this);

        //speech recognition
        recog_sub=nh_.subscribe("/word_recognized",1, &SpeechSystem::listenCallback, &ss);

        tactile_sub=nh_.subscribe("/tactile_touch",1, &SpeechSystem::tactileCallback, &ss);

        // foot contact subscriber
        footContact_sub = nh_.subscribe<std_msgs::Bool>("/foot_contact", 1, &ArucoMarker::footContactCallback, &am);

        // ros parameters get
        ros::param::get("/Naovid/complete/visitor_max", m_visitor_max);
        ros::param::get("/Naovid/complete/check_attention", m_check_attention);

        loop = true;
        person_status = false;
        arucosearch = true;
        walk = true;
        firstw= true;

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~Client_Naovid()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void sensorCallback(const sensor_msgs::JointState::ConstPtr& jointState)
    {
    }

    //handle bottom camera images
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
        m_frame = cv_ptr->image;
        imshow(OPENCV_WINDOW, m_frame);
        waitKey(3);
    }

    // main loop of the program
    void main_loop (ros::Rate rate_sleep){

        string state_var_c; //keeps the current state
        ros::param::get("/Naovid/complete/state_var", state_var_c);
        char state_var = state_var_c[0];

        // subsystem initialisation
        CovidCheck cc;
        Teleoperator to;
        ShirtDetection sd;
        FaceDetection fd;

        m_first_it = true;

        while (m_frame.empty()){
            ros::spinOnce();
        }

        while (nh_.ok()){

            // state machine
            switch (state_var){

            //wait state mode
            case 'w':
            {
                to.blink_stop();
                if(firstw){
                    firstw = false;
//                    ss.moveheadw();

                }
                cout << "Number of visitors inside the buliding: " << m_visitor_count << endl;

                if(sd.check_wait_state_face()){ //enters if a shirt is detected (use sd.check_wait_state(), if you want to do it without face detection)

                    to.blink_red();
                    to.blink_red();
                    to.blink_red();
                    sleep(6);
                    for (int i = 0; i<8; i++){

                        if (fd.checkAttention()) {
                            cout << "face detected inside waiting state" << endl;
                            sd.set_entering(true);
                        }
                        cout << "inside for loop" << endl;
                         ros::spinOnce();
                        sleep(0.2);
                    }
                    to.blink_stop();
                    to.blink_stop();
                    to.blink_stop();
                    to.blink_stop();

                    if (!sd.get_gradient_det()) {
                        cout << "could not detect if you are leaving or entering" << endl;
                        to.blink_red();
                        // ss.animated_speech("hello, I could not detect if you are leaving or entering. Can you redo your movement");
                    }

                    else if (sd.get_entering() && m_visitor_count < m_visitor_max){ //visitor is entering
                        if (sd.get_shirt_already_seen()) {
                            ss.animated_speech("hello,feel free to enter, shirt was already seen");
                            m_visitor_count++;
                            state_var = 'i';
                            firstw = true;
                            ss.animated_speech("Do you have any questions?");
                            person_status = true;

                        }
                        else {
                            ss.WavingHello(); // Welcomes the customer
                            ss.animated_speech("My name is Naovid. How are you doing? You can enter the building. Before that, please show me the QR code of your vaccination certificate. Put it to the camera at the mouth.");
                            state_var = 'c';
                            firstw = true;
                        }
                    }

                    else if (sd.get_entering() && m_visitor_count >= m_visitor_max){ // visitor is entering but the building is full
                        ss.animated_speech("I´m sorry we are full. Please step back the line and wait until another visitor leaves the store");
                        cout << "Please step back the line and wait until another visitor leaves the store" << endl;
                        firstw = true;

                    }

                    else if (!sd.get_entering()){ // visitor is exiting
                        if (sd.get_shirt_already_seen()) {
                            ss.WavingGoodbye(); // says goodbye to visitor
                            ss.FallbackPosition();
                            m_visitor_count--;
                            firstw = true;

                        }
                        else {
                            cout << "shirt was not seen before. Did you sneak into the shop?" << endl;
                        }
                    }
                }
                break;
            }

                //covid check mode
            case 'c':
            {
                cout << "covidcheck" << endl;
                to.blink_stop();
                if(cc.check_certificate(m_frame, nh_)){
                    if (cc.get_m_valid()){

                        m_visitor_count++;  //When you show the good Qr code the person is assumed that in the end he will enter for sure in the end
                        sd.save_shirt_colour();

                        string person_name = cc.get_m_name();
                        cout << "Name of Person: " << person_name << endl;
                        ss.animated_speech("Your covid certificate is valid, ");
                        ss.animated_speech(person_name);
                        cc.set_m_valid(false);
                        person_status = true;
                        state_var = 'i';

                    }

                    else {
                        cout << "CovidCertificate is not valid. Move Arm and Body to side. Please get a vaccination." << endl;
                        ss.StopwithVoice();
                        person_status = false;
                        state_var = 'i';
                    }
                    ss.animated_speech("Do you have any questions?");
                }
                break;
            }


                // interaction mode
            case 'i':
            {
                if (m_first_it){
                    cout << "vocabulary is set." << endl;
                    ss.vocabulary_init();
                    m_first_it = false;
                }

                int a = 0;

                loop = ss.recognisequestion(a); // listens to the question

                if(!loop){

                    if(a == 0){
                        ss.animated_speech("I am sorry, I didn't understand");
                        ss.animated_speech("Can you repeat the question? Or do you want to talk with an other person?");
                    }

                    else if(a == 1){

                        if(person_status){
                            ss.ComeIn(); //alows the visitor inside the building
                            person_status = false;
                        }
                        else{
                            ss.animated_speech("Ok, goodbye");
                        }
                        state_var = 'w';
                    }

                    else if(a == 2){

                        if (fd.checkAttention() || !m_check_attention) {
                            cout << "Face detected. Do interaction." << endl;
                            ss.animated_speech("the actual corona rules are the following:");
                        }
                        else {
                            ss.animated_speech("You are not listening, I will stop talking, Ask me a question when you are ready");
                            break;
                        }
                        sleep(0.3);
                        ros::spinOnce();
                        if (fd.checkAttention() || !m_check_attention) {
                            cout << "Face detected. Do interaction." << endl;
                            ss.animated_speech("you have to wear an FFP2 mask and the 2G rule applies for entering inside,");
                        }
                        else {
                            ss.animated_speech("You are not listening, I will stop talking, Ask me a question when you are ready");
                            break;
                        }
                        sleep(0.3);
                        ros::spinOnce();
                        if (fd.checkAttention() || !m_check_attention) {
                            cout << "Face detected. Do interaction." << endl;
                            ss.animated_speech("so you must be vaccinated or recovered.");                        }
                        else {
                            ss.animated_speech("You are not listening, I will stop talking, Ask me a question when you are ready");
                            break;
                        }
                        sleep(0.3);
                        ros::spinOnce();
                        ss.animated_speech(" Do you have another question?");
                    }

                    else if(a == 3){
                        ss.animated_speech("The bathroom is inside the building");
                        ss.animated_speech("You have to enter and then go left");
                        ss.animated_speech("So of course you have to be vaccinated to enter");
                        ss.animated_speech("Do you have another question?");
                    }

                    else if(a == 4){
                        string s, s2;
                        if(person_status){
                            s = to_string(m_visitor_count-1);
                        }
                        else{
                            s = to_string(m_visitor_count);
                        }
                        s2 = to_string(m_visitor_max);
                        ss.animated_speech("The number of people inside the building is ");
                        ss.animated_speech(s);
                        ss.animated_speech("The maximum number of people allowed inside the building is ");
                        ss.animated_speech(s2);
                        ss.animated_speech(" Do you have another question?");
                    }

                    else if(a == 5){
                        ss.animated_speech("As you want, goodbye");
                        state_var = 't';
                    }
                }
                break;
            }

                //teleoperation mode
            case 't':
            {
                cout << "teleoperation" << endl;
                //DO NOT uncomment during debugging
                to.stand_up();
                to.teleop_walk();
                to.blink_eyes();

                sf::Joystick::update();

                //stop teleoperation
                if (sf::Joystick::isButtonPressed(0,1)){
                    state_var = 'a';
                }
                else if (sf::Joystick::isButtonPressed(0,2)){
                    ss.WavingGoodbye();
                }
                else if (sf::Joystick::isButtonPressed(0,3)){
                    ss.StopTeleop();
                }
                break;
            }

                // aruco walking
            case 'a':
            {
                // looks for aruco marker id 99
                if(arucosearch){
                    cout << "99id aruco searching" << endl;
                    if(am.Arucodetection()){ // aruco found
                        arucosearch = false;
                        am.initWalk();
                        walk = true;

                    }
                }
                else{

                    if(walk){ //walks towards the aruco
                        cout << "CLIENT WALKING" << endl;
                        am.walker();
                        walk = false;
                    }

                    if(am.checkArucoDistance()){
                        cout << "arrived at target" << endl;
                        ss.animated_speech("I'm ready to begin my work");
                        state_var = 'w' ;
                    }

                }
                break;
            }

                // stand up mode
            case 'b' :
            {
                ros::ServiceClient standup_client = nh_.serviceClient<Naovid::Standup>("stand_up");
                Naovid::Standup srv_standup;
                srv_standup.request.fallen = true;

                if (standup_client.call(srv_standup))
                {
                    ROS_INFO("working");
                    cout << "client output: " << srv_standup.response << endl;
                }
                else
                {
                    ROS_ERROR("Failed to call service stand_up");

                }
                to.blink_stop();
                to.blink_stop();
                to.blink_stop();
                to.blink_stop();
                state_var = 'w';
                break;
            }

                //test mode
                //            case 's':
                //            {
                //                break;
                //            }


            }

            rate_sleep.sleep();
            ros::spinOnce();
        }
    }

};

//  main
int main(int argc, char** argv)
{

    ros::init(argc, argv, "naovid_project");
    Client_Naovid ic;

    int sleep_rate;
    ros::param::get("/Naovid/complete/sleep_rate", sleep_rate);
    ros::Rate rate_sleep(sleep_rate);

    ic.main_loop(rate_sleep);

    return 0;
}
