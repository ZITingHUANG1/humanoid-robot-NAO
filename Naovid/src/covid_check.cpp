// Covid Certificate Check
// Giovanni Cortigiani (ge45nab), Ziting Huang (ge95qus), Bernhard Glas (ge34ceh)

/*
 * This module is responsible for the Covid Certificate Check.
 * After encoding the shown QR-code, the read \texttt{string} is sent as a ROS service message  to the
 * corresponding ROS service script. The respective ROS service response contains every important
 * information, including the validity, number of vaccinations and name of the visitor.
 */

#pragma once
#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdlib.h>
#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include "Naovid/CertificateCheck.h"
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
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>

using namespace std;
using namespace cv;
using namespace zbar;

class CovidCheck{

private:
    // CovidCheck member variables
    bool m_valid;
    string m_name;
    string m_qr_string;

    typedef struct
    {
      string type;
      string data;
      vector <Point> location;
    } decodedObject;

public:
    CovidCheck()
    {
        m_valid = false;
    }

    ~CovidCheck(){}

    // uses decode function and transforms the shown QR code of frame into a string which is sent to the covid check service, returns the status of the shown certificate
    bool check_certificate(Mat frame, ros::NodeHandle nh_){
        if (m_qr_string.empty()){
            m_valid = false;
            m_name.clear();
            // Read image
            Mat im = frame;

            // Variable for decoded objects
            vector<decodedObject> decodedObjects;

            // Find and decode barcodes and QR codes
            decode(im, decodedObjects);

            if (!m_qr_string.empty()){
                cout << "m_qr_string: " << m_qr_string << endl;

                ros::ServiceClient client = nh_.serviceClient<Naovid::CertificateCheck>("certificate_check");
                Naovid::CertificateCheck srv;
                srv.request.qrstring = m_qr_string;

                if (client.call(srv))
                {
                    ROS_INFO("working");
                    cout << "client output: " << srv.response << endl;
                    if(srv.response.valid == true){
                        m_valid = true;
                        m_name = srv.response.name;
                    }
                    else {
                        m_valid = false;
                        return true;
                    }

                    m_qr_string.clear();

                }
                else
                {
                    ROS_ERROR("Failed to call service certificate_check");
                    m_valid = false;
                }
            }

            //                // Display location
            //                display(im, decodedObjects);

            //                m_decoded = true;

            //                cout << "string decoded: " << decodedObjects.data << endl;

        }
        m_qr_string.clear();
        return m_valid;
    }

    // adjusted from zbar library, decodes QR codes into strings
    void decode(Mat &im, vector<decodedObject>&decodedObjects){
        // Create zbar scanner
        ImageScanner scanner;

        // Configure scanner
        scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

        // Convert image to grayscale
        Mat imGray;
        cvtColor(im, imGray,COLOR_BGR2GRAY);

        // Wrap image data in a zbar image
        Image image(im.cols, im.rows, "Y800", (uchar *)imGray.data, im.cols * im.rows);

        // Scan the image for barcodes and QRCodes
        int n = scanner.scan(image);

        // Print results
        for(Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
        {
            decodedObject obj;

            obj.type = symbol->get_type_name();
            obj.data = symbol->get_data();

            // Print type and data
            cout << "Type : " << obj.type << endl;
            cout << "Data : " << obj.data << endl << endl;

            m_qr_string = obj.data;

            // Obtain location
            for(int i = 0; i< symbol->get_location_size(); i++)
            {
                obj.location.push_back(Point(symbol->get_location_x(i),symbol->get_location_y(i)));
            }

            decodedObjects.push_back(obj);
        }
    }

    // adjusted from zbar library, marks QR codes in the respective frame
    void display(Mat &im, vector<decodedObject>&decodedObjects){
        // Loop over all decoded objects
        for(int i = 0; i < decodedObjects.size(); i++)
        {
            vector<Point> points = decodedObjects[i].location;
            vector<Point> hull;

            // If the points do not form a quad, find convex hull
            if(points.size() > 4)
                convexHull(points, hull);
            else
                hull = points;

            // Number of points in the convex hull
            int n = hull.size();

            for(int j = 0; j < n; j++)
            {
                line(im, hull[j], hull[ (j+1) % n], Scalar(255,0,0), 3);
            }

        }

        // Display results
        imshow("Results", im);
        waitKey(0);
    }

    //get-function for private variable m_valid
    bool get_m_valid(){
        return m_valid;
    }

    //get-function for private variable m_name
    string get_m_name(){
        return m_name;
    }

    //set-function for the private variable m_valid
    void set_m_valid(bool value){
        m_valid = value;
    }

};
