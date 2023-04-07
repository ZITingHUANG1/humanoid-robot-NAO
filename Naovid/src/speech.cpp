// Speech Module
// Giovanni Cortigiani, Ziting Huang, Bernhard Glas

/*
 *  This module contains all the functions related to the visitor interaction part of the NAO robot,
 *  i. e. speaking and listening with motions included.
 *
 */

#pragma once
#include <ros/ros.h>
#include <vector>
#include <numeric>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/JointState.h"
#include "message_filters/subscriber.h"
#include "Naovid/MoveJoints.h"
#include "Naovid/MoveJoints2.h"
#include <string.h>
#include  <tf/transform_broadcaster.h>
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
#include <Naovid/BlinkAction.h>
#include "Naovid/AniSpeech.h"
#include "Naovid/MoveHead.h"

using namespace std;

class SpeechSystem
{
public:

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;

    // Publisher for nao speech
    ros::Publisher speech_pub;

    // Publisher for nao vocabulary parameters
    ros::Publisher voc_params_pub;

    // Client for starting speech recognition
    ros::ServiceClient recog_start_srv;

    // Client for stoping speech recognition
    ros::ServiceClient recog_stop_srv;

    // Subscriber to speech recognition
    ros::Subscriber recog_sub;

    // Subscriber to head tactile states
    ros::Subscriber tactile_sub;

    // Publisher to nao walking
    ros::Publisher walk_pub;

    // Client for stoping walk
    ros::ServiceClient walk_stop_srv;

    // Client for initiate pose for walking
    ros::ServiceClient needs_start_walk_pose_srv;

    //vocabulary member variables
    vector<string> m_recognized_words;
    vector<string> vocabulary;

    //tactile member variables
    bool tactile_front;
    bool tactile_middle;
    bool tactile_rear;

public:

    SpeechSystem()
        : it_(nh_)
    {

        speech_pub = nh_.advertise<naoqi_bridge_msgs::SpeechWithFeedbackActionGoal>("/speech_action/goal", 1);

        voc_params_pub= nh_.advertise<naoqi_bridge_msgs::SetSpeechVocabularyActionGoal>("/speech_vocabulary_action/goal", 1);

        recog_start_srv=nh_.serviceClient<std_srvs::Empty>("/start_recognition");

        recog_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_recognition");

        walk_pub=nh_.advertise<geometry_msgs::Pose2D>("/cmd_pose", 1);

        walk_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_walk_srv");

        needs_start_walk_pose_srv=nh_.serviceClient<std_srvs::Empty>("/needs_start_walk_pose_srv");

        tactile_front = false;
        tactile_middle = false;
        tactile_rear = false;

    }

    ~SpeechSystem()
    {
    }

    //Callback Functions

    void listenCallback(const naoqi_bridge_msgs::WordRecognized::ConstPtr& msg){

        cout << "inside speechRecognitionCallback" << endl;
        cout << msg->words[0] << endl;
        m_recognized_words.push_back(msg->words[0]);

    }

    void tactileCallback(const naoqi_bridge_msgs::HeadTouch::ConstPtr& tactileState){

        if (tactileState->button == 1 && tactileState->state == 1) tactile_front = true;
        else if (tactileState->button == 2 && tactileState->state == 1) tactile_middle = true;
        else if (tactileState->button == 3 && tactileState->state == 1) tactile_rear = true;
        cout << "Tactile Callback " << (int)tactileState->button << (int)tactileState->state << tactile_front << tactile_middle << endl;

    }


    //Class-related functions

    // This function pruduces animated speech of NAO by calling the relative service
    bool animated_speech(string speech_string) {

        ros::ServiceClient anispeech_client = nh_.serviceClient<Naovid::AniSpeech>("animated_speech");
        Naovid::AniSpeech srv_anispeech;
        srv_anispeech.request.speech_string = speech_string;

        if (anispeech_client.call(srv_anispeech))
        {
            ROS_INFO("animated speech working");
        }
        else
        {
            ROS_ERROR("Failed to call service animated_speech");

        }

        //Head Movement to return to the initial position at the end of the speech
//        ros::ServiceClient head_client = nh_.serviceClient<Naovid::MoveHead>("move_head_service");
//        Naovid::MoveHead head_srv;
//        head_srv.request.angle_yaw = 0;
//        head_srv.request.angle_pitch = -11;
//        head_srv.request.time = 0.2;

//        if (head_client.call(head_srv))
//        {
//            ROS_INFO("head_movement working");
//        }
//        else
//        {
//            ROS_ERROR("Failed to call service move_head_service");
//        }
        moveheadw();
    }

    void moveheadw(){
        //Head Movement to return to the initial position at the end of the speech
        ros::ServiceClient head_client = nh_.serviceClient<Naovid::MoveHead>("move_head_service");
        Naovid::MoveHead head_srv;
        head_srv.request.angle_yaw = 0;
        head_srv.request.angle_pitch = -11;
        head_srv.request.time = 0.2;

        if (head_client.call(head_srv))
        {
            ROS_INFO("head_movement working");
        }
        else
        {
            ROS_ERROR("Failed to call service move_head_service");
        }
    }

    // This function produce the Stop movement by moving the right arm to tell a person not to enter
    void Stop(){

        ros::ServiceClient client = nh_.serviceClient<Naovid::MoveJoints2>("move_service");
        Naovid::MoveJoints2 srv1;
        string names = "RArm";
        srv1.request.jointname = names;
        std::vector<double> pos;
        pos.push_back(0.052957);
        pos.push_back( -0.30122);
        pos.push_back(0.066368);
        srv1.request.position = pos;
        std::vector<double> orient;
        orient.push_back( 1.2421);
        orient.push_back(0.13868);
        orient.push_back(-1.1604);
        srv1.request.orientation = orient;
        std::vector<double> t;
        t.push_back(1.5);
        srv1.request.time = t;

        if (client.call(srv1))
        {
            cout << "Stop worked: " << srv1.response << endl;
        }
        else
        {
            ROS_ERROR("Failed to call service srv1");
        }
    }

    // This function is the same of Stop() but including the voice
    void StopwithVoice(){

        ros::ServiceClient client = nh_.serviceClient<Naovid::MoveJoints2>("move_service");
        Naovid::MoveJoints2 srv1;
        string names = "RArm";
        srv1.request.jointname = names;
        std::vector<double> pos;
        pos.push_back(0.052957);
        pos.push_back( -0.30122);
        pos.push_back(0.066368);
        srv1.request.position = pos;
        std::vector<double> orient;
        orient.push_back( 1.2421);
        orient.push_back(0.13868);
        orient.push_back(-1.1604);
        srv1.request.orientation = orient;
        std::vector<double> t;
        t.push_back(1.5);
        srv1.request.time = t;

        string no = "You cannot enter, please get a vaccination";
        naoqi_bridge_msgs::SpeechWithFeedbackActionGoal msg_speech;
        msg_speech.goal_id.id = "1";
        msg_speech.goal.say = no;
        speech_pub.publish(msg_speech);


        if (client.call(srv1))
        {
            cout << "Stop worked: " << srv1.response << endl;
        }
        else
        {
            ROS_ERROR("Failed to call service srv1");
        }
    }

    // This function is a Stop() function useful for the teleoperation
    void StopTeleop(){

        ros::ServiceClient client = nh_.serviceClient<Naovid::MoveJoints2>("move_service");
        Naovid::MoveJoints2 srv1;
        string names = "RArm";
        srv1.request.jointname = names;
        std::vector<double> pos;
        pos.push_back(0.052957);
        pos.push_back( -0.30122);
        pos.push_back(0.066368);
        srv1.request.position = pos;
        std::vector<double> orient;
        orient.push_back( 1.2421);
        orient.push_back(0.13868);
        orient.push_back(-1.1604);
        srv1.request.orientation = orient;
        std::vector<double> t;
        t.push_back(1.5);
        srv1.request.time = t;

//        string no = "Please keep following me to the office of the teleassistant.";
//        naoqi_bridge_msgs::SpeechWithFeedbackActionGoal msg_speech;
//        msg_speech.goal_id.id = "1";
//        msg_speech.goal.say = no;
//        speech_pub.publish(msg_speech);


        if (client.call(srv1))
        {
            cout << "Stop worked: " << srv1.response << endl;
        }
        else
        {
            ROS_ERROR("Failed to call service srv1");
        }
    }

    // This function provides the gestures and speech to allow a visitor to come inside the building
    void ComeIn(){

        ros::ServiceClient client = nh_.serviceClient<Naovid::MoveJoints2>("move_service");
        Naovid::MoveJoints2 srv2;
        string names = "RArm";
        srv2.request.jointname = names;
        std::vector<double> pos;
        pos.push_back(-0.08860170841217041);
        pos.push_back( -0.1407432109117508);
        pos.push_back(-0.09632524847984314);
        srv2.request.position = pos;
        std::vector<double> orient;
        orient.push_back( 0.9218837022781372);
        orient.push_back(1.131942629814148);
        orient.push_back(-2.8761088848114014);
        srv2.request.orientation = orient;
        srv2.request.maxspeed = 1;
        std::vector<double> t;
        t.push_back(2.0);
        srv2.request.time = t;

        std_srvs::Empty srv;
        needs_start_walk_pose_srv.call(srv);

        geometry_msgs::Pose2D msg_walk;
        msg_walk.x = 0;
        msg_walk.y = -0.1;
        msg_walk.theta = 0;
        walk_pub.publish(msg_walk);

        string goodbye = "Ok, you can enter, goodbye";
        naoqi_bridge_msgs::SpeechWithFeedbackActionGoal msg_speech;
        msg_speech.goal_id.id = "1";
        msg_speech.goal.say = goodbye;
        speech_pub.publish(msg_speech);

        sleep(2);

        cout << "walking" << endl;
        if (client.call(srv2))
        {
            cout << "movement worked: " << srv2.response << endl;
        }
        else
        {
            ROS_ERROR("Failed to call service");
        }

        sleep(4);

        geometry_msgs::Pose2D msg_walk2;
        msg_walk2.x = 0;
        msg_walk2.y = 0.1;
        msg_walk2.theta = 0.087;
        walk_pub.publish(msg_walk2);

        //FallbackPosition();

        sleep(3);
        FallbackPosition();



        std_srvs::Empty srvw;
        walk_stop_srv.call(srvw);

        moveheadw();


    }

    // This function makes the Arm return to the fallback position
    void FallbackPosition(){

        ros::ServiceClient client = nh_.serviceClient<Naovid::MoveJoints2>("move_service");
        Naovid::MoveJoints2 srv3;
        string names = "RArm";
        srv3.request.jointname = names;
        std::vector<double> pos;
        pos.push_back(0.07548246532678604);
        pos.push_back( -0.1146557554602623);
        pos.push_back(-0.09718003869056702);
        srv3.request.position = pos;
        std::vector<double> orient;
        orient.push_back( 1.3473002910614014);
        orient.push_back(0.9731849431991577);
        orient.push_back(-0.024950077757239342);
        srv3.request.orientation = orient;
        srv3.request.maxspeed = 1;
        std::vector<double> t;
        t.push_back(2.0);
        srv3.request.time = t;

        if (client.call(srv3))
        {
            cout << "Fallback position worked: " << srv3.response << endl;
        }
        else
        {
            ROS_ERROR("Failed to call service srv3");
        }
    }

    // This function corresponds to one of the two positions needed to make the robot waving to a visitor to say "Hello"
    void anglearm(){

        ros::ServiceClient client = nh_.serviceClient<Naovid::MoveJoints2>("move_service");
        Naovid::MoveJoints2 srv4;
        string names = "RArm";
        srv4.request.jointname = names;
        std::vector<double> pos;
        pos.push_back(0.06098267436027527);
        pos.push_back( -0.22116714715957642);
        pos.push_back(0.1939358115196228);
        srv4.request.position = pos;
        std::vector<double> orient;
        orient.push_back( 0.6566203832626343);
        orient.push_back(-1.2457923889160156);
        orient.push_back(-0.5968418717384338);
        srv4.request.orientation = orient;
        srv4.request.maxspeed = 1;
        std::vector<double> t;
        t.push_back(0.5);
        srv4.request.time = t;

        if (client.call(srv4))
        {
            cout << "angle arm worked: " << srv4.response << endl;
        }
        else
        {
            ROS_ERROR("Failed to call service srv4");
        }
    }

    // This function corresponds to the other position needed to make the robot waving to a visitor to say "Hello"
    void straightarm(){ //second part of waving hello

        ros::ServiceClient client = nh_.serviceClient<Naovid::MoveJoints2>("move_service");
        Naovid::MoveJoints2 srv5;
        string names = "RArm";
        srv5.request.jointname = names;
        std::vector<double> pos;
        pos.push_back(0.07584589719772339);
        pos.push_back( -0.28096628189086914);
        pos.push_back(0.1493038833141327);
        srv5.request.position = pos;
        std::vector<double> orient;
        orient.push_back( 1.0772671699523926);
        orient.push_back(-0.6148801445960999);
        orient.push_back(-1.0763087272644043);
        srv5.request.orientation = orient;
        srv5.request.maxspeed = 1;
        std::vector<double> t;
        t.push_back(0.5);
        srv5.request.time = t;

        if (client.call(srv5))
        {
            cout << "straight arm worked: " << srv5.response << endl;
        }
        else
        {
            ROS_ERROR("Failed to call service srv5");
        }
    }

    // This function provides a Waving movement of the Arm for welcoming a visitor
    void WavingHello(){
        int i=0;
        string hello = "Goodmorning dear costumer";
        naoqi_bridge_msgs::SpeechWithFeedbackActionGoal msg_speech;
        msg_speech.goal_id.id = "1";
        msg_speech.goal.say = hello;
        straightarm();
        speech_pub.publish(msg_speech);
        anglearm();
        straightarm();
        anglearm();
    }

    // This function provides a Waving movement of the Arm for saying goodbye to a visitor
    void WavingGoodbye(){
        int i=0;
        string hello = "Goodbye dear costumer, hope you come back soon";
        naoqi_bridge_msgs::SpeechWithFeedbackActionGoal msg_speech;
        msg_speech.goal_id.id = "1";
        msg_speech.goal.say = hello;
        straightarm();
        speech_pub.publish(msg_speech);
        anglearm();
        straightarm();
        anglearm();
    }

    //NOT USED
    // This function makes the NAO listen to the answer of the visitor
    bool dyhquestion(bool &a){

        if (tactile_front){
            std_srvs::Empty srv;
            recog_start_srv.call(srv);
            tactile_front = false;
            cout << "text recognition started" << endl;
        }

        if (tactile_middle){
            std_srvs::Empty srv;
            recog_stop_srv.call(srv);
            cout << "text recognition stopped" << endl;
            if (!m_recognized_words.empty()) cout << "recognized first word: " <<  m_recognized_words[0] << endl;
            else cout << "no words were detected" << endl;
            a = 0;

            for (int i = 0; i < m_recognized_words.size(); i++) {
                if (m_recognized_words[i] == "no"){
                    a = 1;
                }
            }

            cout << "a = " << a << endl;
            m_recognized_words.clear();
            tactile_middle = false;
            return false;
        }
        return true;
    }

    // This function makes the NAO listen and understands the question or answer of a visitor
    bool recognisequestion(int &a){

        if (tactile_front){
            std_srvs::Empty srv;
            recog_start_srv.call(srv);
            tactile_front = false;
            cout << "text recognition started" << endl;
        }

        if (tactile_middle){
            std_srvs::Empty srv;
            recog_stop_srv.call(srv);
            cout << "text recognition stopped" << endl;

            // Sentence not understood
            if (!m_recognized_words.empty()) cout << "recognized first word: " <<  m_recognized_words[0] << endl;
            else cout << "no words were detected" << endl;
            a = 0;


            for (int i = 0; i < m_recognized_words.size(); i++) {
                //no more questions
                if (m_recognized_words[i] == "no"){
                    a = 1;
                }
                // Covid rules question
                if (m_recognized_words[i] == "covid" || m_recognized_words[i] == "rules" || m_recognized_words[i] == "mask"){
                    a = 2;
                }
                // Where can I find a toilet?
                if (m_recognized_words[i] == "toilet" || m_recognized_words[i] == "wc" || m_recognized_words[i] == "bathroom"){
                    a = 3;
                }
                // What is the number of people inside the building?
                if (m_recognized_words[i] == "people" || m_recognized_words[i] == "number"){
                    a = 4;
                }
                // Can I talk with a teleoperator?
                if (m_recognized_words[i] == "person" || m_recognized_words[i] == "assistant" || m_recognized_words[i] == "teleoperator" || m_recognized_words[i] == "real human"){
                    a = 5;
                }
            }

            m_recognized_words.clear();
            tactile_middle = false;
            return false;
        }
        return true;
    }

    // Vocabulary initialisation
    void vocabulary_init(){

        vector<string> vocabulary;
        vocabulary.push_back("yes");
        vocabulary.push_back("no");
        vocabulary.push_back("covid");
        vocabulary.push_back("rules");
        vocabulary.push_back("mask");
        vocabulary.push_back("toilet");
       // vocabulary.push_back("wc");
        vocabulary.push_back("bathroom");
        vocabulary.push_back("people");
        vocabulary.push_back("number");
        vocabulary.push_back("person");
        vocabulary.push_back("assistant");
        //vocabulary.push_back("teleoperator");
        //vocabulary.push_back("real human");

        naoqi_bridge_msgs::SetSpeechVocabularyActionGoal msg_voc;
        msg_voc.goal_id.id = "1";
        msg_voc.goal.words = vocabulary;
        voc_params_pub.publish(msg_voc);
    }


};

