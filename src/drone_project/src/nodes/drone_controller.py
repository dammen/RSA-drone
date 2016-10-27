#!/usr/bin/env python

import rospy, roslib, math

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from ardrone_autonomy.msg import Navdata
from drone.msg import beaconGeometry


INTERVAL = 100 #ms


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
		self.memory = [[320,180,0]]
		self.count = 0
		self.autoMode = False
		self.stop = False
		self.width = 640.0
		self.height = 360.0
		self.center = (self.width/2, self.height/2)
		self.MAX_SPEED = 1
		self.countHover = False


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

		#Enable this if you want to Send Commands every [INTERVAL] milliseconds
		#instead of each time you recieve something on the beaconGeometry Topic

		#self.setCommandTimer = rospy.Timer(rospy.Duration(INTERVAL / 1000.0), self.goAutonome)

		#Sends commands When you are in Manual mode. Turns on and off in setAuto and QuitAuto
		self.commandTimer = rospy.Timer(rospy.Duration(INTERVAL / 1000.0), self.sendCommand)

		
		

		# safely land if we are shutting down
		rospy.on_shutdown(self.land)



	def setCommand(self, roll = 0, pitch = 0, yawVel = 0, zVel = 0):
		self.command.linear.x = pitch
		self.command.linear.y = roll
		self.command.linear.z = zVel
		self.command.angular.z = yawVel

	def sendCommand(self, event):
		if self.autoMode == True:
			return
		if self.state == 3 or self.state == 7 or self.state == 4:
			self.CommandPublisher.publish(self.command)

	def updateState(self, navdata):
		self.state = navdata.state

	def land(self):
		self.LandingPublisher.publish(Empty())

	def reset(self):
		self.ResetPublisher.publish(Empty())

	def takeoff(self):
		self.autoMode = False
		self.count = 0
		if self.state == 2:
			self.TakeoffPublisher.publish(Empty())


			#Called when the controller recieves data from beaconGeometry topic
			#saves the values localy and runs goAutonome, which calculates and
			#and publishes new values to cmd_vel topic.
	def geometryUpdate(self, geometryData):
		self.beaconPointX = geometryData.positionX
		self.beaconPointY = geometryData.positionY
		self.beaconAngle = geometryData.angle 
		if (self.beaconAngle < 0) :
			self.beaconAngle = 180 - self.beaconAngle
		#print("BEACON ANGLE: ", self.beaconAngle)
		if self.autoMode==True and not self.countHover:
			self.goAutonome()
		elif (self.autoMode==True and self.countHover):
			self.hover()
		self.countHover = not self.countHover
			#Enables Automode

	def setAuto(self):
		self.autoMode = True
		self.commandTimer.shutdown()

		#Disables Automode
	def quitAuto(self):
		self.autoMode = False
		self.commandTimer = rospy.Timer(rospy.Duration(INTERVAL / 1000.0), self.sendCommand)

		# Makes the drone hover instead of making a movement. Not permanent.
	def hover(self):
		print("hovering")
		self.command.linear.x = 0
		self.command.linear.y = 0
		self.command.angular.z = 0	
		self.CommandPublisher.publish(self.command)

		#Called from goAutonome
	def saveNewValuesToList(self, x, y):
		#if it doesn`t see the circle add to count else add recieved point to memory-list. 
		if x < 0 or y < 0:
			self.count += 1
		else:  #checks to see if it has gotten a new value on the topic
			if x == self.memory[-1][0] and y == self.memory[-1][1]:
				self.hover()
				self.stop = True
				return
				
			self.memory.append([self.beaconPointX, self.beaconPointY, self.beaconAngle])	
			self.count = 0
			if len(self.memory) > 20 :
				self.memory.pop(0)

			# Called from goAutonome
			# finds the average value of where the beacon is located
			# so that the drone can fly that way if it loose sight of the beacon.
	def calculateAverageValues(self):
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
		self.beaconPointX = 3*averageX
		self.beaconPointY = 3*averageY
		self.beaconAngle = averageAngle
		print("Average calculated ")
		print("AverageX = ", averageX) 
		print("AverageY = ", averageY) 
		print("AverageAngle = ", averageAngle) 


		# Called from goAutonome
		#Sets speed in x-direction, uses beaconPoint-Y.
		#This is because the Image x-axis equals the drones Y axis, and opposite.
	def setSpeedX(self, y):
		y2 = (180.0 - y)

		if y < 20:
			self.command.linear.x = 1
		elif y > 340 :
			self.command.linear.x = -1
		elif (y2 >= 20 and y2 <= 160) or (y2 >= -160 and y2 <= -20) :
			#print("y2", y2)
			self.command.linear.x = y2/160.0
		else:
			self.command.linear.x = 0

	# Called from goAutonome
	#Sets speed in y-direction, uses beaconPoint-X
	#This is because the Image x-axis equals the drones Y axis, and opposite.
	def setSpeedY(self, x): 
		x2 = (320.0 - x)
		if x < 30:
			self.command.linear.y = 1
		elif x > 610 :
			self.command.linear.y = -1
		elif (x2 >= 20 and x2 <= 290) or (x2 >= -290 and x2 <= -20) :
			print("x2", x2)
			self.command.linear.y = x2/290.0
		else: 
			self.command.linear.y = 0

		# Called from goAutonome
		#Sets the angular z speed
	def setSpeedAngular(self): 

		if self.beaconAngle > 5 and self.beaconAngle < 180:
			self.command.angular.z = self.beaconAngle/180
		elif self.beaconAngle > 180 and self.beaconAngle < 355:
			self.command.angular.z = -(self.beaconAngle -180)/180
		else: 
			self.command.angular.z = 0
		self.command.angular.z = 0

#----------MAIN AUTOMODE METHOD----------#
	def goAutonome(self):
		#Check if it is in Autonome mode
		if self.autoMode == False:
			return

		#Prints values recieved from bottomBeaconDetector
		print("--------------AUTONOME-----------------")
		print("recieved beacon-point X = ", self.beaconPointX)
		print("recieved beacon-point y = ", self.beaconPointY)
		print("recieved beacon-Angle z = ", self.beaconAngle)
		
		#checks recieved values and add them to memory
		#print("saving values to list")
		self.saveNewValuesToList(self.beaconPointX, self.beaconPointY)

		#stops rest of calculations if it didn`t get a new value
		if self.stop == True: 
			self.stop= False
			return

		# Lands Drone if it hasn`t seen the circle in self.count times,
		# Else it will calculate average values to find which way to fly to find the circle again. 	
		if self.beaconPointX < 0 or self.beaconPointY < 0 :
			if self.count >= 60:
				self.autoMode = False
				print("Lost track of beacon... Landing now")
				self.LandingPublisher.publish(Empty())
				return
			elif self.count < 40:
				self.hover()
				return
			else:
				self.calculateAverageValues()
				
				
				
		#print("setting speed")
		x = self.beaconPointX
		y = self.beaconPointY

		#Switching x with y and opposite because the Image x-axis equals the drones Y axis, and opposite.
		
		self.setSpeedX(y)
		self.setSpeedY(x)
		self.setSpeedAngular()	

		print("commands being sent are: ")
		print("Linear-x = ", self.command.linear.x)
		print("Linear-y = ", self.command.linear.y)
		print("Angular-z = ", self.command.angular.z)

		if self.state == 3 or self.state == 7 or self.state == 4:
			self.CommandPublisher.publish(self.command)





