#!/usr/bin/env python
import sherlock_holmes as holmes
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
import pattern as p
from drone.msg import beaconGeometry

NODE_NAME = 'pattern_detector'

class PatternDetector(object):
    def __init__(self):
    	self.patternPublisher = rospy.Publisher("/ardrone/beaconGeometry", beaconGeometry)
        self.pattern_id_publisher = rospy.Publisher('pattern_id', Int8, queue_size=10)
        self.pattern_pos_publisher = rospy.Publisher('pattern_pos', Point, queue_size=10)
        self.pattern_rot_publisher = rospy.Publisher('pattern_rot', Int8, queue_size=10)
        self.sherlock = holmes.SherlockHolmes()

    def image_callback(self, data):
        self.sherlock.image = data.data
        pattern = self.sherlock.detect_pattern()
        self.pattenPublisher.publish(canSeeBeacon=true, positionX=pattern.x, positionY=pattern.y, angle=pattern.rotation)
        self.pattern_id_publisher.publish(data=pattern.ID)
        self.pattern_pos_publisher.publish(x=pattern.x, y=pattern.y)
        self.pattern_rot_publisher.publish(data=int(pattern.rotation))

def main():
    rospy.init_node(NODE_NAME, anonymous=True)
    thing = PatternDetector()
    rospy.Subscriber('/ardrone/bottom/image_raw', Image, thing.image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
