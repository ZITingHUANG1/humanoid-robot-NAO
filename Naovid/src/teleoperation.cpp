//Teleoperation
// Giovanni Cortigiani, Ziting Huang, Bernhard Glas

//This subsystem handles the whole teleoperation functionalities.
//Planar movement is implemented by reading the values from the left stick with the SFML-library.
//Hereby, orientation of the stick without touching is meant to be zero,
//such that the robot is not moving. The payload is set to maximum of translational speed,
//if the maximum of deflection occurs. In between, linear interpolation is done.
//y using the right stick of the controller, rotation around z-axis can be enabled.
//By pressing the directional buttons with up/down/right/left, one could directly control the movement
//of the head. Moreover, the blinking of the eyes can be set to green or red in order to indicate
//whether the robot is already busy or open to interact with another customer.
//Furthermore, several gestures are pre-recorded and can be activated by simply pressing the according,
//button. As already proposed, all the functionalities are explained through the information given in the SFML-window.

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

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_srvs/Empty.h>
#include <naoqi_bridge_msgs/BlinkActionGoal.h>
#include <actionlib_msgs/GoalStatusArray.h>

#include "Naovid/Standup.h"
#include "Naovid/MoveHead.h"


#include <SFML/Graphics.hpp>
#include <iostream>
#include <set>
#include <thread>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>


using namespace cv;
using namespace std;

static const std::string OPENCV_WINDOW2 = "Top Camera Teleoperator";


class Teleoperator
{
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;

    //Teleoperation member variables
    double m_x_value = 0.0;
    double m_y_value = 0.0;
    double m_head_yaw_value = 0.0;
    double m_head_pitch_value = 0.0;
    double m_rz_value = 0.0;

    bool m_foot_contact = false;

    // SFML Window variables
    string m_string_p_path;
    sf::RenderWindow m_window;
    sf::Event m_e;
    thread m_window_thread;
    sf::Text m_text;
    sf::Font m_font;
    std::stringstream m_ss;

    // Client for stoping walk
    ros::ServiceClient walk_stop_srv;

    // Publisher to nao walking
    ros::Publisher walk_pub;

    // Subscriber for foot contact
    ros::Subscriber footContact_sub;

    // Publisher to access the action server functionalities “/blink/goal”
    ros::Publisher blink_eyes_pub;


public:

    Teleoperator()
        : it_(nh_), m_window(sf::VideoMode(1400,160),"Teleoperation Instructions"), m_window_thread(&Teleoperator::wait_window_event,this)
    {
        // Subscribe to input video feed from top camera
        image_sub_ = it_.subscribe("/nao_robot/camera/top/camera/image_raw", 1,
                                   &Teleoperator::imageCb1, this);

        // Initialise utilized services
        walk_stop_srv=nh_.serviceClient<std_srvs::Empty>("/stop_walk_srv");

        walk_pub=nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

        footContact_sub = nh_.subscribe<std_msgs::Bool>("/foot_contact", 1, &Teleoperator::footContactCallback, this);

        blink_eyes_pub=nh_.advertise<naoqi_bridge_msgs::BlinkActionGoal>("/blink/goal", 1);

        cv::namedWindow(OPENCV_WINDOW2);

        // Set SFML Window

        m_window.setJoystickThreshold(10);
        m_window.setPosition(sf::Vector2i(300, 1000));
        m_window.setActive(true);

        //Load Font and Settings for SFML Instruction Window
        boost::filesystem::path full_path(boost::filesystem::current_path());
        boost::filesystem::path parent_path = full_path.parent_path();
        m_string_p_path = parent_path.string();

        //        string ending = "Naovid";
        //        int slash_num = std::count(m_string_p_path.begin(),m_string_p_path.end(),'/');
        //        for (int i = 0; i<slash_num-1;i++){
        //            if(!boost::algorithm::ends_with(m_string_p_path, ending)) {
        //                boost::filesystem::path temp_path = parent_path.parent_path();
        //                parent_path = temp_path;
        //                m_string_p_path = parent_path.string();
        //            }
        //            else break;
        //        }

        string directory_font_file = m_string_p_path + "/ros/Naovid_ws/src/Naovid/launch/OpenSans-Regular.ttf";
        if (!m_font.loadFromFile(directory_font_file))
        {
            std::cout << "Font not loadable!" <<std::endl;
        }
        m_text.setFont(m_font);
        m_text.setCharacterSize(18);
    }



    ~Teleoperator()
    {
        cv::destroyWindow(OPENCV_WINDOW2);
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
        cv::imshow(OPENCV_WINDOW2, frame);
        cv::waitKey(3);
    }

    // handle foot contact callback
    void footContactCallback(const std_msgs::BoolConstPtr& contact)
    {
        /*
       * TODO control tutorial 3 Ex 3
       */

        m_foot_contact = contact->data;
        //      cout << "contact: " << contact << endl;
        //      cout << "m_foot_contact: " << m_foot_contact << endl;
    }

    // robot to pose init via NAIqi Python interface and self-defined ROS service to Python script
    void stand_up(){
        sf::Joystick::update();
        if (sf::Joystick::isButtonPressed(0,0)){
//            cout << "A is pressed" << endl;
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
        }
    }

    // Usage of NAO Walker App in order to make the robot walk during teleoperation
    void teleop_walk(){
        m_x_value = 0.0;
        m_y_value = 0.0;
        m_rz_value = 0.0;
        bool bool_move_head = false;

        sf::Joystick::update();
        //        cout << "left-joystick x value: " << sf::Joystick::getAxisPosition(0,sf::Joystick::X) << endl;
        //        cout << "left-joystick y value: " << sf::Joystick::getAxisPosition(0,sf::Joystick::Y) << endl;
        //        cout << "RT-joystick r value: " << sf::Joystick::getAxisPosition(0,sf::Joystick::R) << endl;
        //        cout << "LT-joystick z value: " << sf::Joystick::getAxisPosition(0,sf::Joystick::Z) << endl;

        //Walker
        if (sf::Joystick::getAxisPosition(0,sf::Joystick::X) < -9 || sf::Joystick::getAxisPosition(0,sf::Joystick::X) > 9){
            m_x_value = sf::Joystick::getAxisPosition(0,sf::Joystick::X)/100;
//            cout << "m_x_value: " << m_x_value << endl;
        }
        if (sf::Joystick::getAxisPosition(0,sf::Joystick::Y) < -7 || sf::Joystick::getAxisPosition(0,sf::Joystick::Y) > 7){
            m_y_value = sf::Joystick::getAxisPosition(0,sf::Joystick::Y)/100;
//            cout << "m_y_value: " << m_y_value << endl;
        }
        if (sf::Joystick::getAxisPosition(0,sf::Joystick::U) < -9 || sf::Joystick::getAxisPosition(0,sf::Joystick::U) > 9){
            m_rz_value = sf::Joystick::getAxisPosition(0,sf::Joystick::U)/100;
//            cout << "m_rz_value: " << m_rz_value << endl;
        }

        //HeadMovement
        double step_head_move = 2.0;
        if(m_head_yaw_value < 119.5 && m_head_yaw_value > -119.5){
            if (sf::Joystick::getAxisPosition(0,sf::Joystick::PovX) == 100) {
                bool_move_head = true;
                m_head_yaw_value -= step_head_move;
            }
            else if (sf::Joystick::getAxisPosition(0,sf::Joystick::PovX) == -100) {
                bool_move_head = true;
                m_head_yaw_value += step_head_move;
            }
        }

        if (m_head_pitch_value+step_head_move > -38.5 && m_head_pitch_value-step_head_move < 29.5) {
            if (sf::Joystick::getAxisPosition(0,sf::Joystick::PovY) == 100) {
                bool_move_head = true;
                m_head_pitch_value += step_head_move;
            }
            else if (sf::Joystick::getAxisPosition(0,sf::Joystick::PovY) == -100) {
                bool_move_head = true;
                m_head_pitch_value -= step_head_move;
            }
        }


        //        cout << "m_foot_contact: " << m_foot_contact << endl;

        // Set the values to the respective service messages and execute service
        if (m_foot_contact){
            //Walker
            geometry_msgs::Twist msg_walk;
            msg_walk.linear.y = -m_x_value;
            msg_walk.linear.x = -m_y_value;
            msg_walk.linear.z = 0.0;
            msg_walk.angular.x = 0.0;
            msg_walk.angular.y = 0.0;
            msg_walk.angular.z = -m_rz_value;

            walk_pub.publish(msg_walk);

            //Head Movement
            ros::ServiceClient head_client = nh_.serviceClient<Naovid::MoveHead>("move_head_service");
            Naovid::MoveHead head_srv;

            head_srv.request.angle_yaw = m_head_yaw_value;
            head_srv.request.angle_pitch = m_head_pitch_value;         
            head_srv.request.time = 0.1;

            if (bool_move_head) {
                if (head_client.call(head_srv))
                {
                    ROS_INFO("head_movement working");
                }
                else
                {
                    ROS_ERROR("Failed to call service move_head_service");

                }
            }
        }
    }

    // change colour of the eyes for telling the visitor if the robot is already busy or free to communicate within the teleoperation mode
    void blink_eyes(){
        sf::Joystick::update();
        if (sf::Joystick::isButtonPressed(0,5)){
//            cout << "inside blink right" << endl;

            naoqi_bridge_msgs::BlinkActionGoal msg;
            msg.goal_id.id = "3";

            std::vector<std_msgs::ColorRGBA> colors;
            std_msgs::ColorRGBA color;
            color.r = 0;
            color.g = 1;
            color.b = 0;
            color.a = 1;
            colors.push_back(color);
            msg.goal.colors = colors;

            msg.goal.blink_duration = ros::Duration(4.0);
            msg.goal.blink_rate_mean = 1.0;
            msg.goal.blink_rate_sd = 0.1;
            blink_eyes_pub.publish(msg);
        }
        if (sf::Joystick::isButtonPressed(0,4)){
//            cout << "inside blink left" << endl;

            naoqi_bridge_msgs::BlinkActionGoal msg2;
            msg2.goal_id.id = "1";

            std::vector<std_msgs::ColorRGBA> colors;
            std_msgs::ColorRGBA color;
            color.r = 1;
            color.g = 0;
            color.b = 0;
            color.a = 1;
            colors.push_back(color);
            msg2.goal.colors = colors;

            msg2.goal.blink_duration = ros::Duration(4.0);
            msg2.goal.blink_rate_mean = 1.0;
            msg2.goal.blink_rate_sd = 0.1;
            blink_eyes_pub.publish(msg2);
        }
        if (sf::Joystick::isButtonPressed(0,7)){
//            cout << "inside blink off" << endl;

            naoqi_bridge_msgs::BlinkActionGoal msg2;
            msg2.goal_id.id = "2";

            std::vector<std_msgs::ColorRGBA> colors;
            std_msgs::ColorRGBA color;
            color.r = 0;
            color.g = 0;
            color.b = 0;
            color.a = 1;
            colors.push_back(color);
            msg2.goal.colors = colors;

            msg2.goal.blink_duration = ros::Duration(4.0);
            msg2.goal.blink_rate_mean = 1.0;
            msg2.goal.blink_rate_sd = 0.1;
            blink_eyes_pub.publish(msg2);
        }
    }

    //function to let the eyes blink red
    void blink_red(){
        naoqi_bridge_msgs::BlinkActionGoal msg2;
        msg2.goal_id.id = "1";

        std::vector<std_msgs::ColorRGBA> colors;
        std_msgs::ColorRGBA color;
        color.r = 1;
        color.g = 0;
        color.b = 0;
        color.a = 1;
        colors.push_back(color);
        msg2.goal.colors = colors;

        msg2.goal.blink_duration = ros::Duration(4.0);
        msg2.goal.blink_rate_mean = 1.0;
        msg2.goal.blink_rate_sd = 0.1;
        blink_eyes_pub.publish(msg2);
    }

    //function to stop blinking the eyes
    void blink_stop(){
//        sleep(2);
        naoqi_bridge_msgs::BlinkActionGoal msg2;
        msg2.goal_id.id = "2";

        std::vector<std_msgs::ColorRGBA> colors;
        std_msgs::ColorRGBA color;
        color.r = 0;
        color.g = 0;
        color.b = 0;
        color.a = 1;
        colors.push_back(color);
        msg2.goal.colors = colors;

        msg2.goal.blink_duration = ros::Duration(4.0);
        msg2.goal.blink_rate_mean = 1.0;
        msg2.goal.blink_rate_sd = 0.1;
        blink_eyes_pub.publish(msg2);
    }

    // Set SFML Instruction Window with updated content
    void set_window() {
//        cout << "m_window is open" << endl;
        m_ss << "Planar movement with left joystick ; rotation z-axis with right joystick ; head-movement with up/down and right/left \n" <<
                "Stand up with A ; Stop teleoperation with B ; Wave to say Goodbye with X ; Stop person when she or he is not following you with Y \n" <<
                "Press RB to set eyes blinking to green, LB for red and List button for turniing the eyes off \n" <<
                "Enjoy operating the robot";

        std::string s = m_ss.str();
        m_ss.str("");
        m_text.setString(s);
        m_window.clear();
        m_window.draw(m_text);
        m_window.display();
    }

    // SFML specific commands in order to not break the window thread
    void wait_window_event(){
        m_window.setActive(true);
        while(m_window.isOpen()){
            m_window.waitEvent(m_e);
            if(m_e.type==sf::Event::JoystickButtonPressed){
            }
            set_window();
        }
    }

};

