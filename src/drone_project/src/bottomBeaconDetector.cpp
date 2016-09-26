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
    /*
    //split image to colour channels
    std::vector<cv::Mat> split_img;
    cv::split(img_ptr->image, split_img);
    cv::Mat empty_img = cv::Mat::zeros(img_ptr->image.rows, img_ptr->image.cols, CV_8UC1);
    cv::Mat blue(img_ptr->image.rows, img_ptr->image.cols, CV_8UC3);
    cv::Mat green(img_ptr->image.rows, img_ptr->image.cols, CV_8UC3);
    cv::Mat red(img_ptr->image.rows, img_ptr->image.cols, CV_8UC3);
    //blue
    cv::Mat in1[] = {split_img[0], empty_img, empty_img};
    int from_to1[] = {0,0,1,1,2,2};
    cv::mixChannels(in1, 3, &blue, 1, from_to1, 3);
    //green
    cv::Mat in2[] = {empty_img, split_img[1], empty_img};
    int from_to2[] = {0,0,1,1,2,2};
    cv::mixChannels(in2, 3, &green, 1, from_to2, 3);
    //red
    cv::Mat in3[] = {empty_img, empty_img, split_img[2]};
    int from_to3[] = {0,0,1,1,2,2};
    cv::mixChannels(in3, 3, &red, 1, from_to3, 3);   

    cv::namedWindow("blue", CV_WINDOW_AUTOSIZE);
    cv::imshow("blue", blue); 
    cv::namedWindow("green", CV_WINDOW_AUTOSIZE);
    cv::imshow("green", green);
    cv::namedWindow("red", CV_WINDOW_AUTOSIZE);
    cv::imshow("red", red);
    */
    cv::Mat grey_img;
    cv::cvtColor(img_ptr->image, grey_img, CV_BGR2GRAY);
    /*
    cv::Mat bw_img;
    bw_img = grey_img > 180;
    cv::namedWindow("Image Black-White", CV_WINDOW_AUTOSIZE);
    cv::imshow("Image Black-White", bw_img);
    cv::waitKey(3);
    */
    cv::GaussianBlur(grey_img, grey_img, cv::Size(9,9), 2, 2);
    // This code displays the greyscale image - used for testing.
    //cv::namedWindow("Image Greyscale", CV_WINDOW_AUTOSIZE);
    //cv::imshow("Image Greyscale", grey_img);
    //cv::waitKey(3);

    std::vector<cv::Vec3f> circles;
    // The last parameter is the max radius to detect. The whole black beacon is 70-80 pixels
    // reducing this to 50 should detect only small circles, but it doesnt. Maybe other 
    // parameters can be tweaked to detect the small circles only.
    cv::HoughCircles(grey_img, circles, CV_HOUGH_GRADIENT, 2, 20, 200, 100, 0, 100);
    ROS_INFO("finished detection. Found %d circles", int(circles.size())); 
    //draw circles detected
    for(int i = 0; i < circles.size(); i++){
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        //ROS_INFO("circle has radius = %d\n", radius);
        cv::circle(img_ptr->image, center, 3, cv::Scalar(0,255,0), -1, 8, 0);
        cv::circle(img_ptr->image, center, radius, cv::Scalar(0,0,255), 3, 8, 0);
    }
    sensor_msgs::Image img;
    img = *(img_ptr->toImageMsg());
    //ROS_INFO("got image");
    bottomCameraPub.publish(img);

}

}
