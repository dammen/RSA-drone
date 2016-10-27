/*
 * bottomBeaconDetector.cpp
 *
 *  Created on: 16/09/2016
 *      Author: Ben Faul (z3422539)
 */

#include <ros/ros.h>

#include <bottomBeaconDetector.hpp>
#include <vector>
#include <drone/beaconGeometry.h>

#define MASK_THRESHOLD 2.0


namespace drone {
using namespace cv;

BottomBeaconDetector::BottomBeaconDetector() : it_(nh) {
    /*
    cv::namedWindow("Normal Image", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Black Filter", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Blue Filter", cv::WINDOW_AUTOSIZE);
    */
}

void BottomBeaconDetector::configure() {}

void BottomBeaconDetector::startup() {
    
    bottomCameraSub = nh.subscribe<sensor_msgs::Image>(
        "/ardrone/bottom/image_raw",
        1,
        &BottomBeaconDetector::callbackControl,
         this
    );
    
    // Make custom message type later
    beaconPub = nh.advertise<drone::beaconGeometry>("/ardrone/beaconGeometry", 1);
    imagePublisher = it_.advertise("/ardrone/ModifiedImage", 1);
    
}

void BottomBeaconDetector::shutdown() {
     cv::destroyWindow(OPENCV_WINDOW);
}

void BottomBeaconDetector::callbackControl(const sensor_msgs::ImageConstPtr& frame) {
   // ROS_INFO("BottomBeaconDetector::callbackControl - Image received");
    cv_bridge::CvImagePtr cv_ptr;
    
    try {
        cv_ptr = cv_bridge::toCvCopy(frame, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    
    analyseImage(cv_ptr);
}
/*
void BottomBeaconDetector::trackBarblack(int, void*) {
    
    int h1 = 100;
    int s1 = 100;
    int v1 = 100;
    int h = 100;
    int s = 100;
    int v = 100;
    h1 = getTrackbarPos("h1", "Black Filter");
    s1 = getTrackbarPos("s1", "Black Filter");
    v1 = getTrackbarPos("v1", "Black Filter");
    h = getTrackbarPos("h", "Black Filter");
    s = getTrackbarPos("s", "Black Filter");
    v = getTrackbarPos("v", "Black Filter");
    cv::inRange(hsv_image, cv::Scalar(h, s, v), cv::Scalar(h1, s1, v1), blackFilterRange);
    imshow("Black Filter", blackFilterRange);
}

void BottomBeaconDetector::trackBarblue(int, void*) {
    int h1 = 100;
    int s1 = 100;
    int v1 = 100;
    int h = 100;
    int s = 100;
    int v = 100;
    
    h = getTrackbarPos("h", "Blue Filter");
    s = getTrackbarPos("s", "Blue Filter");
    v = getTrackbarPos("v", "Blue Filter");
    h1 = getTrackbarPos("h1", "Blue Filter");
    s1 = getTrackbarPos("s1", "Blue Filter");
    v1 = getTrackbarPos("v1", "Blue Filter");
    
    cv::inRange(hsv_image, cv::Scalar(h, s, v), cv::Scalar(h1, s1, v1), blueFilterRange);
    cv::imshow("Blue Filter", blueFilterRange);
}
*/
void BottomBeaconDetector::analyseImage(cv_bridge::CvImagePtr cv_ptr) {
    cv::Mat bgr_image = cv_ptr->image;
    cv::Mat hsv_image;
    cv::Mat mask;
    //cv::Mat helperMask;
    cv::Mat blueFilterRange;
    cv::Mat blackFilterRange;
    drone::beaconGeometry msg;
    
   // ROS_INFO("X: %d, Y: %d", bgr_image.cols, bgr_image.rows);

    cv::medianBlur(bgr_image, bgr_image, 3);

    // Convert input image to HSV   
    cv::cvtColor(bgr_image, hsv_image, cv::COLOR_BGR2HSV);

    cv::inRange(hsv_image, cv::Scalar(59, 68, 80), cv::Scalar(180, 255, 150), blackFilterRange);    // Initially used as a beacon test 
    cv::inRange(hsv_image, cv::Scalar(100, 130, 57), cv::Scalar(115, 255, 255), blueFilterRange);
	
   // Find appropriate threshold in testing
   if (cv::countNonZero(blackFilterRange) < 500) {
      msg.canSeeBeacon = false;
      msg.positionX = -1;
      msg.positionY = -1;
      msg.angle = -1;
      beaconPub.publish(msg);
      imagePublisher.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image).toImageMsg());
        return;
    }
    
    //ROS_INFO("Beacon Detected");
    msg.canSeeBeacon = true;
    
    std::vector<Vec3f> circles;
    vector<Vec4i> linesP;

    GaussianBlur(blueFilterRange, blueFilterRange, Size(9,9), 2, 2);
    HoughCircles(blueFilterRange, circles, CV_HOUGH_GRADIENT, 2, blueFilterRange.rows/4, 100, 50);
    // this code assumes the first circle found is the beacon.
    // is this safe? Hough Circles above assumes circles are rows/4 pixels apart
    if (!circles.empty()) {
      // ROS_INFO("Beacon Location: X:%d Y:%d, Radius: %d", (int)circles[0][0], (int)circles[0][1], (int)circles[0][2]);
        
        msg.positionX = (int)circles[0][0];
        msg.positionY = (int)circles[0][1];
        


        //Drawing Circles
        Point center(cvRound(circles[0][0]), cvRound(circles[0][1]));
        int radius = cvRound(circles[0][2]);
        circle(bgr_image, center, radius, Scalar(0,255,0), -3, 8, 0);

        
        // set mask for lines
        mask = Mat::zeros(hsv_image.size(), hsv_image.type());

        Point topLeft (circles[0][0] - circles[0][2] * MASK_THRESHOLD, circles[0][1] - circles[0][2]*MASK_THRESHOLD);
        Point bottomRight (circles [0][0] + circles[0][2] * MASK_THRESHOLD, circles[0][1] + circles[0][2] * MASK_THRESHOLD);
        
        if (topLeft.x < 0) topLeft.x = 0;
        if (topLeft.y < 0) topLeft.y = 0;
        if (bottomRight.x > hsv_image.cols) bottomRight.x = hsv_image.cols - 1;
        if (bottomRight.y > hsv_image.rows) bottomRight.y = hsv_image.rows - 1;
        
        cv::rectangle(mask, topLeft, bottomRight, Scalar(255, 255, 255), -1, 8, 0);
        
        hsv_image.copyTo(blackFilterRange, mask);
    } else {
            //no beacon found by Hough Circles
	    msg.canSeeBeacon = false;
	    msg.positionX = -1;
	    msg.positionY = -1;
	    msg.angle = -1;
	    beaconPub.publish(msg);
        imagePublisher.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image).toImageMsg());
        return;
    }
    
	//cv::inRange(helperMask, cv::Scalar(101, 30, 100), cv::Scalar(180, 45, 160), blackFilterRange);
	cv::inRange(blackFilterRange, cv::Scalar(0, 49, 20), cv::Scalar(180, 255, 150), blackFilterRange);
    Canny(blackFilterRange, blackFilterRange, 50, 200);
    HoughLinesP(blackFilterRange, linesP, 1, CV_PI/180, 30, 30, 5);
   
   
    
   if (!linesP.empty()) {

        int circleRadius = (int)circles[0][2];
        int radiusSqrd = circleRadius*circleRadius;
        int circleX = (int)circles[0][0];
        int circleY = (int)circles[0][1];
        Vec4i currentLine = (-1, -1, -1, -1);
        Vec4i beaconEdge = (-1, -1, -1, -1);

        for( size_t i = 0; i < linesP.size(); i++){

            currentLine = linesP[i];
            
            // distance2 calculates the distance^2 between two points - dont want to to sqrt when image processing!
            int diff1 = distance2(currentLine[0], circleX, currentLine[1], circleY)-radiusSqrd;
            if(diff1 < 0) diff1 *= -1;

            int diff2 = distance2(currentLine[2], circleX, currentLine[3], circleY)-radiusSqrd;
            if(diff2 < 0) diff2 *= -1;

            if( diff1 < 5 && diff2 < 5){
                // both ends points of the line are within 5 pixels of being on the beacon edge.
                beaconEdge = currentLine;
                break;
            }
        }

        // now currentLine = the flat edge of the beacon if found
        if (beaconEdge[0] == -1 && beaconEdge[1] == -1 && beaconEdge[2] == -1 && beaconEdge[3] == -1){
            ROS_INFO("*** No beacon edge found ***\n"); // but atleast we can still get the beacon center
            msg.angle = -1;
	        beaconPub.publish(msg);
            imagePublisher.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image).toImageMsg());
            return;
        }
        
        // get midpoint of beacon edge
        int midpointX = (currentLine[0] + currentLine[2])/2;
        int midpointY = (currentLine[1] + currentLine[3])/2;

        int diffY = midpointY - circleY;
        int diffX = midpointX - circleX;

        float angle = atan2(diffY, diffX) * 180.0/CV_PI; 
        //angle += 180;

        ROS_INFO("Beacon Angle: %f, Total Lines Detected: %d", angle, (int)linesP.size());
        msg.angle = angle; 
        line( bgr_image, Point(currentLine[0], currentLine[1]), Point(currentLine[2], currentLine[3]), Scalar(0,0,255), 3, CV_AA);
    }

/*
        Vec4i max_l;
        bool vecSet = false;
        double max_dist = 50;
        // Get first line over 50 pixels, avoiding large lines found in environment
        for( size_t i = 0; i < linesP.size(); i++ )
        {
            Vec4i l = linesP[i];
            double theta1,theta2, hyp, result;
            // x & y differences not theta
            theta1 = (l[3]-l[1]);
            theta2 = (l[2]-l[0]);
            hyp = hypot(theta1,theta2);

            if (max_dist <  hyp) {
                max_l = l;
                vecSet = true;
                max_dist = hyp;
                break;
            }           
        }
        
        if (!vecSet) {
            max_dist = 40;
            // Get first line over 40 pixels, avoiding large lines found in environment
            for( size_t i = 0; i < linesP.size(); i++ )
            {
                Vec4i l = linesP[i];
                double theta1,theta2, hyp, result;

                theta1 = (l[3]-l[1]);
                theta2 = (l[2]-l[0]);
                hyp = hypot(theta1,theta2);

                if (max_dist <  hyp) {
                    max_l = l;
                    vecSet = true;
                    max_dist = hyp;
                    break;
                }           
            }
        }
        if (!vecSet) max_l = linesP[0];
*/        
        /*
        //Draw all matching lines
        for( size_t i = 0; i < linesP.size(); i++ )
        {
          Vec4i l = linesP[i];
          line( bgr_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 3, CV_AA);
        }
        */

/*
        line( bgr_image, Point(max_l[0], max_l[1]), Point(max_l[2], max_l[3]), Scalar(255,0,0), 3, CV_AA);
        
        int diffX, diffY;
        Point p1 (max_l[0], max_l[1]);
        Point p2 (max_l[2], max_l[3]);
        
        // ?? What is this for??
        p1.x -= circles[0][0];
        p2.x -= circles[0][0];
        p1.y -= circles[0][1];
        p1.y *= -1;
        p2.y -= circles[0][1];
        p2.y *= -1;
        ROS_INFO("%d, %d", p1.x, p1.y);
        ROS_INFO("%d, %d", p2.x, p2.y);
*/        
        /*int set = 0;
        if (max_l[3] > max_l[1]) {
            set= 1;
            diffY = max_l[3] - max_l[1];
            diffX = max_l[2] - max_l[0];
        }*/
        /*
        else {
            set = 2;
            diffY = max_l[1] - max_l[3];
            diffX = max_l[0] - max_l[2];
        }*/
        // REALLY INCORRECT BUT ITLL WORK
        
//        diffY = p2.y - p1.y;
 //       diffX = p2.x - p1.x;

        //atan2 returns a value between -pi/2 and pi/2
        // convert to degrees then add 180 to get range from 0-360
//        float angle = atan2(diffY, diffX) * 180.0/CV_PI; 
//        angle += 180;
        
        /*
        if ( angle > 0 && p1.x < 0 && p1.y > 0) {
            ROS_INFO("QUAD 2");
            angle = 180 - angle;
        }
        else if (angle > 0 && p1.x > 0 && p1.y < 0) {
            ROS_INFO("QUAD 4");
            angle = 360 - angle;
        }
        else if ( angle < 0 && p1.x < 0 && p1.y < 0) {
            ROS_INFO("QUAD 3");
            angle = 180 - angle;
        }
        else if ( angle < 0 && p1.x > 0 && p1.y > 0) {
            ROS_INFO("QUAD 1");
            angle *= -1;
            angle += 360;
        }*/

        //ROS_INFO("P1 (%d,%d) P2(%d,%d), SET: %d", max_l[0], max_l[1], max_l[2], max_l[3], set);
        
	// THIS IS NOT RIGHT. Need to find whether circle center is above ot below line.
//        angle -= 90; // off set
  
  		//angle +=180;      
//        ROS_INFO("Beacon Angle: %f, Total Lines Detected: %d", angle, (int)linesP.size());
//        msg.angle = angle; 
        
 //   }


    // Display filtered Images
    /*
    cv::imshow("Blue Filter", blueFilterRange);
    cv::imshow("Black Filter", blackFilterRange);
    cv::imshow("Normal Image", bgr_image);
    
    createTrackbar("h", "Black Filter", 0, 179, trackBarblack);
    createTrackbar("s", "Black Filter", 0, 255,trackBarblack);
    createTrackbar("v", "Black Filter", 0, 255,trackBarblack);
    createTrackbar("h1", "Black Filter", 0, 179, trackBarblack);
    createTrackbar("s1", "Black Filter", 0, 255,trackBarblack);
    createTrackbar("v1", "Black Filter", 0, 255,trackBarblack);
    
    createTrackbar("h", "Blue Filter", 0, 179,trackBarblue);
    createTrackbar("s", "Blue Filter", 0, 255, trackBarblue);
    createTrackbar("v", "Blue Filter", 0, 255, trackBarblue);
    createTrackbar("h1", "Blue Filter", 0, 179,trackBarblue);
    createTrackbar("s1", "Blue Filter", 0, 255, trackBarblue);
    createTrackbar("v1", "Blue Filter", 0, 255, trackBarblue);  
    */

    beaconPub.publish(msg);
    imagePublisher.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", bgr_image).toImageMsg());
}

int BottomBeaconDetector::distance2(int x1, int x2, int y1, int y2){
    return ((x1-x2)*(x1-x2)) + ((y1-y2)*(y1-y2));
}

} // namespace drone








