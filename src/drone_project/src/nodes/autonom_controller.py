#!/usr/bin/env python

import roslib
import rospy

from drone_controller import DroneController
from drone_video_display import DroneVideoDisplay

from PySide import QtCore, QtGui

class KeyMapping(object):
	PitchForward     = QtCore.Qt.Key.Key_E
	PitchBackward    = QtCore.Qt.Key.Key_D
	RollLeft         = QtCore.Qt.Key.Key_S
	RollRight        = QtCore.Qt.Key.Key_F
	YawLeft          = QtCore.Qt.Key.Key_W
	YawRight         = QtCore.Qt.Key.Key_R
	IncreaseAltitude = QtCore.Qt.Key.Key_Q
	DecreaseAltitude = QtCore.Qt.Key.Key_A
	Takeoff          = QtCore.Qt.Key.Key_Y
	Land             = QtCore.Qt.Key.Key_H
	Emergency        = QtCore.Qt.Key.Key_Space
	GoAuto		 = QtCore.Qt.Key.Key_Z
	GoManuell	 = QtCore.Qt.Key.Key_X

class KeyboardController(DroneVideoDisplay):
	def __init__(self):
		super(KeyboardController,self).__init__()
		self.autonome = False
		self.pitch = 0
		self.roll = 0
		self.yaw_velocity = 0 
		self.z_velocity = 0

	def keyPressEvent(self, event):
		key = event.key()
		if controller is not None and not event.isAutoRepeat():
			if key == KeyMapping.Emergency:
				controller.reset()
			elif key == KeyMapping.Takeoff:
				controller.takeoff()
			elif key == KeyMapping.Land:
				controller.land()
			elif key == KeyMapping.GoAuto:
				controller.goAutonome()
			else:
				if key == KeyMapping.YawLeft:
					self.yaw_velocity += 2
				elif key == KeyMapping.YawRight:
					self.yaw_velocity += -2

				elif key == KeyMapping.PitchForward:
					self.pitch += 2
				elif key == KeyMapping.PitchBackward:
					self.pitch += -2

				elif key == KeyMapping.RollLeft:
					self.roll += 2
				elif key == KeyMapping.RollRight:
					self.roll += -2

				elif key == KeyMapping.IncreaseAltitude:
					self.z_velocity += 2
				elif key == KeyMapping.DecreaseAltitude:
					self.z_velocity += -2

			print("key press")
			print("roll:" + str(self.roll))
			print("pitch:" + str(self.pitch))
			print("yaw_velocity: " + str(self.yaw_velocity))
			print("z_velocity: " + str(self.z_velocity))
			controller.setCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)


	def keyReleaseEvent(self,event):
		key = event.key()
		if controller is not None and not event.isAutoRepeat():
			
			if key == KeyMapping.YawLeft:
				self.yaw_velocity -= 2
			elif key == KeyMapping.YawRight:
				self.yaw_velocity -= -2

			elif key == KeyMapping.PitchForward:
				self.pitch -= 2
			elif key == KeyMapping.PitchBackward:
				self.pitch -= -2

			elif key == KeyMapping.RollLeft:
				self.roll -= 2
			elif key == KeyMapping.RollRight:
				self.roll -= -2

			elif key == KeyMapping.IncreaseAltitude:
				self.z_velocity -= 2
			elif key == KeyMapping.DecreaseAltitude:
				self.z_velocity -= -2

			# finally we set the command to be sent. The controller handles sending this at regular intervals
			print("key release")
			print("roll:" + str(self.roll))
			print("pitch:" + str(self.pitch))
			print("yaw_velocity: " + str(self.yaw_velocity))
			print("z_velocity: " + str(self.z_velocity))
			controller.setCommand(self.roll, self.pitch, self.yaw_velocity, self.z_velocity)



# Setup the application
if __name__=='__main__':
	import sys
	# Firstly we setup a ros node, so that we can communicate with the other packages
	rospy.init_node('ardrone_autonom_controller')

	# Now we construct our Qt Application and associated controllers and windows
	app = QtGui.QApplication(sys.argv)
	controller = DroneController()
	display = KeyboardController()

	display.show()

	# executes the QT application
	status = app.exec_()

	# and only progresses to here once the application has been shutdown
	rospy.signal_shutdown('Great Flying!')
	sys.exit(status)
