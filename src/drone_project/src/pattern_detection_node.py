#!/usr/bin/env python
import sherlock_holmes as holmes
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
import drone_movements
import pattern as p
from drone_controller import DroneController
from cv_bridge import CvBridge, CvBridgeError
from drone.msg import beaconGeometry

# The pattern rotation:
# e.g. if the rotation is 40 degrees then that means the drone
# should move 40 degrees counter clockwise to correctly orient the pattern.
# if the rotation is -20 degrees then that means that the drone
# should move -20 degrees counter clockwise (aka 20 degrees clockwise)

NODE_NAME = 'pattern_detector'

class PatternDetector(object):
    def __init__(self):
        self.image_subscriber = rospy.Subscriber('/ardrone/bottom/image_raw', Image, self.image_callback)
        self.pattern_publisher = rospy.Publisher("/ardrone/beaconGeometry", beaconGeometry)
        self.pattern_id_publisher = rospy.Publisher('pattern_id', Int8, queue_size=10)
        self.bridge = CvBridge()
        self.controller = DroneController()
        self.sherlock = holmes.SherlockHolmes()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.sherlock.image = cv_image
        pattern = self.sherlock.detect_pattern()

        if pattern:
            self.pattern_publisher.publish(canSeeBeacon=True, positionX=pattern.x, positionY=pattern.y, angle=pattern.rotation)
            self.pattern_id_publisher.publish(data=pattern.ID)
        else:
            self.pattern_publisher.publish(canSeeBeacon=False, positionX=0, positionY=0, angle=0)

def main():
    rospy.init_node(NODE_NAME, anonymous=True)
    pattern_detector = PatternDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
