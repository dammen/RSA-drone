#!/usr/bin/env python

import rospy, roslib

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
from drone.msg import beaconGeometry


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
		self.beaconPointX = 0
		self.beaconPointY = 0
		self.beaconAngle = 0

		# subscribe to the drone's current state
		self.NavDataSubscriber = rospy.Subscriber('/ardrone/navdata', Navdata, self.updateState)
		
		self.beaconSubscriber = rospy.Subscriber("/ardrone/beaconGeometry", beaconGeometry, self.geometryUpdate)
		

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

	def setCommand(self, roll = 0, pitch = 0, yawVel = 0, zVel = 0):
		self.command.linear.x = pitch
		self.command.linear.y = roll
		self.command.linear.z = zVel
		self.command.angular.z = yawVel

	def sendCommand(self, event):
		if self.state == 3 or self.state == 7 or self.state == 4:
			self.CommandPublisher.publish(self.command)

	def updateState(self, navdata):
		self.state = navdata.state

	def land(self):
		self.LandingPublisher.publish(Empty())

	def reset(self):
		self.ResetPublisher.publish(Empty())

	def takeoff(self):
		if self.state == 2:
			self.TakeoffPublisher.publish(Empty())

	def geometryUpdate(self, geometryData):
		self.beaconPointX = geometryData.positionX
		self.beaconPointY = geometryData.positionY
		self.beaconAngle = geometryData.angle
		

	def goAutonome(self):
		print("AUTONOME")
		print(self.beaconPointX)
		print(self.beaconPointY)

		if self.beaconPointX < 0 or self.beaconPointY < 0:
			print("Lost track of beacon... Landing now")
			self.LandingPublisher.publish(Empty())

		else:
			x = self.beaconPointX
			y = self.beaconPointY
			if x < 300:
				self.command.linear.x = 3
			elif x > 340 :
				self.command.linear.x = -3
			else: 
				self.command.linear.x = 0 

			if y < 160:
				self.command.linear.y = 3
			if y > 200 :
				self.command.linear.y = -3
			else:
				self.command.linear.y = 0
			
			if self.beaconAngle > 10 and self.beaconAngle < 180:
				self.command.angular.z = 3
			elif self.beaconAngle > 180 and self.beaconAngle < 				350:
				self.command.angular.z = -3
			else: 
				self.command.angular.z = 0
		print(self.command.linear.x)
		print(self.command.linear.y)
		print(self.command.angular.z)
		if self.state == 3 or self.state == 7 or self.state == 4:
			self.CommandPublisher.publish(self.command)



