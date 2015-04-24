from math import sin, cos, pow, pi, atan, sqrt
from random import seed, gauss
import numpy as np

class imu():
	""" An imu object that defines the true movement and simulated sensor data for a 9dof imu """
	# Real positions
	self.theta_x = 0.0
	self.theta_y = 0.0
	self.theta_z = 0.0
	
	# True sensor bias
	self.bias_x = 0.0
	self.bias_y = 0.0
	self.bias_z = 0.0

	# Sensor values
	self.gyro_x = 0.0
	self.gyro_y = 0.0
	self.gyro_z = 0.0

	self.acc_x = 0.0
	self.acc_y = 0.0
	self.acc_z = 0.0

	self.mag_x = 0.0
	self.mag_y = 0.0
	self.mag_z = 0.0

	# Sensor Properties
	self.gyroStdDev_x = 0.4
	self.gyroStdDev_y = 0.4
	self.gyroStdDev_z = 0.4
	self.gyroVar_x = self.gyroStdDev_x*self.gyroStdDev_x
	self.gyroVar_y = self.gyroStdDev_y*self.gyroStdDev_y
	self.gyroVar_z = self.gyroStdDev_z*self.gyroStdDev_z

	self.accStdDev_x = 0.4
	self.accStdDev_y = 0.4
	self.accStdDev_z = 0.4
	self.accVar_x = self.accStdDev_x*self.accStdDev_x
	self.accVar_y = self.accStdDev_y*self.accStdDev_y
	self.accVar_z = self.accStdDev_z*self.accStdDev_z

	self.magStdDev_x = 0.4
	self.magStdDev_y = 0.4
	self.magStdDev_z = 0.4
	self.magVar_x = self.magStdDev_x*self.magStdDev_x
	self.magVar_y = self.magStdDev_y*self.magStdDev_y
	self.magVar_z = self.magStdDev_z*self.magStdDev_z
