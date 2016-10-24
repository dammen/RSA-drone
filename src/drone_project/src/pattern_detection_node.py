#!/usr/bin/env python
import sherlock_holmes as holmes
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from geometry_msgs.msg import Point, Vector3
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
        self.movement_publisher = rospy.Publisher('/ardrone/movements', Vector3)
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
            movements = drone_movements.DroneMovement(pattern, self.sherlock.image.shape[1], self.sherlock.image.shape[0])
            vec = movements.make_decision()
            self.movement_publisher.publish(x=vec[0], y=vec[1], z=vec[2])
        else:
            self.pattern_publisher.publish(canSeeBeacon=False, positionX=-1, positionY=-1, angle=0)

def main():
    rospy.init_node(NODE_NAME, anonymous=True)
    pattern_detector = PatternDetector()
    rospy.spin()

if __name__ == '__main__':
    main()
