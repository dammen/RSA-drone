#!/usr/bin/python

import rospy, roslib

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import NavData

INTERVAL = 100

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

		# subscribe to the drone's current state
		self.NavDataSubscriber = rospy.Subscriber('/ardrone/navdata', NavData, self.updateState)

		# publishers for landing, taking off, resetting and sending Twist commands to move the drone
		self.LandingPublisher = rospy.Publisher('/ardrone/land', Empty)
		self.TakeoffPublisher = rospy.Publisher('/ardrone/takeoff', Empty)
		self.ResetPublisher = rospy.Publisher('/ardrone/reset', Empty)
		self.CommandPublisher = rospy.Publisher('/cmd_vel', Twist)

		# empty command to begin with
		self.command = Twist()
		# send command every {INTERVAL} milliseconds
		self.commandTimer = rospy.Timer(rospy.Duration(INTERVAL / 1000.0), self.sendCommand)

		# safely land if we are shutting down
		rospy.on_shutdown(self.land)

	def setCommand(self, pitch = 0, roll = 0, zVel = 0, yawVel = 0):
		self.command.linear.x = pitch
		self.command.linear.y = roll
		self.command.linear.z = zVel
		self.command.angular.z = yawVel

	def sendCommand(self):
		if self.state == FLYING1 or self.state == FLYING2 or self.state == HOVERING:
			self.CommandPublisher.publish(self.command)

	def updateState(self, navdata):
		self.state = navdata.state

	def land(self):
		self.LandPublisher.publish(Empty())

	def reset(self):
		self.ResetPublisher.publish(Empty())

	def takeoff(self):
		if self.state == LANDED:
			self.TakeoffPublisher.publish(Empty())