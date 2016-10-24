#!/usr/bin/env python

import rospy, roslib

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
from drone.msg import beaconGeometry


INTERVAL = 33

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
		self.beaconLostFrames = 0
		self.beaconPointX = 0
		self.beaconPointY = 0
		self.beaconAngle = 0
		self.memory = [[0,0,0]]
		self.count = 0
		self.autoMode = False


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
		
		self.setCommandTimer = rospy.Timer(rospy.Duration(INTERVAL / 1000.0), self.goAutonome)

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
		if (self.beaconAngle < 0) :
			self.beaconAngle = 180 - self.beaconAngle

	def setAuto(self):
		self.autoMode = True

	def quitAuto(self):
		self.autoMode = False


	def goAutonome(self, event):
		if self.autoMode == False:
			return
		
		print("--------------AUTONOME-----------------")
		print(self.beaconPointX)
		print(self.beaconPointY)
		

		if self.beaconPointX < 0 or self.beaconPointY < 0:
			self.count += 1
	
		else:
			if self.beaconPointX == self.memory[-1][0] and self.beaconPointY == self.memory[-1][1]:
				self.command.linear.x = 0
				self.command.linear.y = 0
				self.command.angular.z = 0	
				self.CommandPublisher.publish(self.command)
				return
				


			self.memory.append([self.beaconPointX, self.beaconPointY, self.beaconAngle])	
			self.count = 0
			if len(self.memory) > 30 :
				self.memory.pop(0)

			
		

		if self.beaconPointX < 0 or self.beaconPointY < 0 :
			if self.count >= 4:
				self.autoMode = False
				print("Lost track of beacon... Landing now")
				self.LandingPublisher.publish(Empty())
				return
			elif self.count < 2:
				self.command.linear.x = 0
				self.command.linear.y = 0
				self.command.angular.z = 0	
				self.CommandPublisher.publish(self.command)
				return
				
			else:
				print("getting average for the ", self.count, " time")
				x = 0
				y = 0
				angle = 0
				for i in range(len(self.memory)):
					x += self.memory[i][0]
					y += self.memory[i][1]
					angle += self.memory[i][2]
				averageX = x/len(self.memory)
				averageY = y/len(self.memory)
				averageAngle = angle/len(self.memory)
				self.beaconPointX = averageX
				self.beaconPointY = averageY
				self.beaconAngle = averageAngle
				print("Average calculated ")
				print("AverageX = ", averageX) 
				print("AverageY = ", averageY) 
				print("AverageAngle = ", averageAngle) 
				

		
		self.beaconLostFrames = 0
		x = self.beaconPointX
		y = self.beaconPointY
		
		x2 = (320.0 - x)
		y2 = (180.0 - y)

		if x < 120:
			self.command.linear.y = -1
		elif x > 520 :
			self.command.linear.y = 1
		elif (x2 >= 20 and x2 <= 200) or (x2 >= -200 and x2 <= -20) :
			print("x2", x2)
			self.command.linear.y = -x2/200.0
		else: 
			self.command.linear.y = 0 

		if y < 80:
			self.command.linear.x = -1
		elif y > 280 :
			self.command.linear.x = 1
		elif (y2 >= 20 and y2 <= 100) or (y2 >= -100 and y2 <= -20) :
			#print("y2", y2)
			self.command.linear.x = -y2/100.0
		else:
			self.command.linear.x = 0
		
		if self.beaconAngle > 5 and self.beaconAngle < 180:
			self.command.angular.z = 1
		elif self.beaconAngle > 180 and self.beaconAngle < 355:
			self.command.angular.z = -1
		elif self.beaconAngle < 0:
			self.command.angular.z = -1
		else: 
			self.command.angular.z = 0

		self.command.angular.z = 0

		self.command.linear.x = -self.command.linear.x
		self.command.linear.y = -self.command.linear.y


		print("commands being sent are: ")
		print(self.command.linear.x)
		print(self.command.linear.y)
		print(self.command.angular.z)
		if self.state == 3 or self.state == 7 or self.state == 4:
			self.CommandPublisher.publish(self.command)


	#		self.beaconLostFrames+= 1
	#		print("lost beacon for ", str(self.beaconLostFrames))
	#		if self.beaconLostFrames >= 10:
	#			print("Lost track of beacon... Landing now")


