/*
 * File         : bottomBeaconDetector.cpp
 * Author       : Benjamin Faul
 *
 * Created      : 16/09/2016
 * Description  : AR Drone Project beacon detector. This code receives images
 *                from the bottom camera of the AR Drone and performs circle detection.
 *                The circles detected are drawn on the image and then the image is 
 *                published.
 *
 * Version      : 1.0 - bfaul 16/9/16
 *                Initial version.
 *
 *                1.1 - bfaul 18/9/16
 *                Added OpenCV code for circle detection.
 *
 */

#include <ros/ros.h>
#include <bottomBeaconDetector.hpp>

namespace drone {

BottomBeaconDetector::BottomBeaconDetector(){}

void BottomBeaconDetector::configure() {}

void BottomBeaconDetector::startup() {
    ros::NodeHandle nh;
    bottomCameraSub = nh.subscribe<sensor_msgs::Image>(
        "ardrone/bottom/image_raw",
        1,
        &BottomBeaconDetector::callbackControl,
         this
    );
    bottomCameraPub = nh.advertise<sensor_msgs::Image>("drone/bottom_detector", 1);
}

void BottomBeaconDetector::shutdown(){}

void BottomBeaconDetector::callbackControl(const sensor_msgs::ImageConstPtr& frame) {
    //ROS_INFO("BottomBeaconDetector::callbackControl - Image received");
    cv_bridge::CvImagePtr img_ptr;
    try{
        img_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    cv::Mat grey_img;
    cv::cvtColor(img_ptr->image, grey_img, CV_BGR2GRAY);

    // This code displays the greyscale image - used for testing.
    //cv::namedWindow("Image Greyscale", CV_WINDOW_AUTOSIZE);
    //cv::imshow("Image Greyscale", grey_img);
    //cv::waitKey(3);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(grey_img, circles, CV_HOUGH_GRADIENT, 2, 5, 200, 100, 0, 1000);
    ROS_INFO("finished detection. Found %d circles", int(circles.size())); 
    //draw circles detected
    for(int i = 0; i < circles.size(); i++){
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        cv::circle(img_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
        cv::circle(img_ptr->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
    }
    sensor_msgs::Image img;
    img = *(img_ptr->toImageMsg());
    //ROS_INFO("got image");
    bottomCameraPub.publish(img);

}

}
