#!/usr/bin/python

import rospy, roslib

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import NavData
from drone.msg import beaconGeometry

INTERVAL = 30
IMAGE_HEIGHT = 360
IMAGE_WIDTH = 640

class DroneController:
	# AR drone state constants
	UNKNOWN = 0
	INITED = 1
	LANDED = 2
	FLYING1 = 3
	HOVERING = 4
	TEST = 5
	TAKING_OFF = 6
	FLYING2 = 7
	LANDING = 8
	LOOPING = 9

	def __init__(self):
		# unknown state to begin with
		self.state = 0

		# whether we are controlling or not
		self.autopilot = False

		# subscribe to the drone's current state
		self.NavDataSubscriber = rospy.Subscriber('/ardrone/navdata', NavData, self.updateState)
		self.BeaconSubscriber = rospy.Subscriber("/ardrone/beaconGeometry", beaconGeometry, self.geometryUpdate)

		# publishers for landing, taking off, resetting and sending Twist commands to move the drone
		self.LandingPublisher = rospy.Publisher('/ardrone/land', Empty)
		self.TakeoffPublisher = rospy.Publisher('/ardrone/takeoff', Empty)
		self.ResetPublisher = rospy.Publisher('/ardrone/reset', Empty)
		self.CommandPublisher = rospy.Publisher('/cmd_vel', Twist)

		# empty command to begin with
		self.command = Twist()
		# send command every {INTERVAL} milliseconds
		self.commandTimer = rospy.Timer(rospy.Duration(INTERVAL / 1000.0), self.sendCommand)
		# self.decisionTimer = rospy.Timer(rospy.Duration(INTERVAL / 1000.0), self.makeDecision)

		# safely land if we are shutting down
		rospy.on_shutdown(self.land)

	# don't have to set every single param, set to 0 for default
	def setCommand(self, pitch = 0, roll = 0, zVel = 0, yawVel = 0, angularX = 0, angularY = 0):
		self.command.linear.x = pitch
		self.command.linear.y = roll
		self.command.linear.z = zVel
		self.command.angular.z = yawVel
		self.command.angular.x = angularX
		self.command.angular.y = angularY

	def sendCommand(self):
		if self.state == self.FLYING1 or self.state == self.FLYING2 or self.state == self.HOVERING:
			self.CommandPublisher.publish(self.command)

	def updateState(self, navdata):
		self.state = navdata.state

	def land(self):
		self.LandPublisher.publish(Empty())

	# emergency stop
	def reset(self):
		self.ResetPublisher.publish(Empty())

	def takeoff(self):
		if self.state == self.LANDED:
			self.TakeoffPublisher.publish(Empty())

	# go into auto hover mode
	def autoHover(self):
		self.setCommand()

	def geometryUpdate(self, geometryData):
		self.beaconPointX = geometryData.positionX
		self.beaconPointY = geometryData.positionY
		self.beaconAngle = self.normalise(geometryData.angle)

	def normalise(self, angle):
		return (angle + 360) % 360


