#!/usr/bin/env python
import sherlock_holmes as holmes
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
import pattern as p
from cv_bridge import CvBridge, CvBridgeError
from drone.msg import beaconGeometry

NODE_NAME = 'pattern_detector'

class PatternDetector(object):
    def __init__(self):
        self.image_subscriber = rospy.Subscriber('/ardrone/bottom/image_raw', Image, self.image_callback)
        self.patternPublisher = rospy.Publisher("/ardrone/beaconGeometry", beaconGeometry)
        self.pattern_id_publisher = rospy.Publisher('pattern_id', Int8, queue_size=10)
        self.pattern_pos_publisher = rospy.Publisher('pattern_pos', Point, queue_size=10)
        self.pattern_rot_publisher = rospy.Publisher('pattern_rot', Int8, queue_size=10)
        self.bridge = CvBridge()
        self.sherlock = holmes.SherlockHolmes()

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        self.sherlock.image = cv_image
        pattern = self.sherlock.detect_pattern()
        self.pattenPublisher.publish(canSeeBeacon=True, positionX=pattern.x, positionY=pattern.y, angle=pattern.rotation)
        self.pattern_id_publisher.publish(data=pattern.ID)
        self.pattern_pos_publisher.publish(x=pattern.x, y=pattern.y)
        self.pattern_rot_publisher.publish(data=int(pattern.rotation))

def main():
    rospy.init_node(NODE_NAME, anonymous=True)
    pattern_detector = PatternDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
