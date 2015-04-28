from imu import imu
import numpy as np
IMU = imu()
n=0
while n < 10:

	IMU.move_body(10.0, 10.0, 10.0)
	IMU.get_predict_sensor()
	IMU.update_prediction_matrices()
	#print IMU.process_model

	IMU.P = np.dot(IMU.A, np.dot(IMU.P, IMU.A.T)) + np.dot(IMU.G, np.dot(IMU.Q, IMU.G.T))

	print IMU.actual_pos


	IMU.get_update_sensor('acc')
	IMU.update_correction_matrices()
	Y = IMU.Z - np.dot(IMU.H, IMU.states)
	#print Y



	kalman_gain = np.dot(np.dot(IMU.P, IMU.H.T), np.linalg.inv(np.dot(IMU.H, np.dot(IMU.P, IMU.H.T)) + IMU.R))
	#print kalman_gain

	IMU.states = IMU.states + np.dot(kalman_gain, Y)
	print IMU.states

	IMU.P = np.dot((np.identity(6) - np.dot(kalman_gain, IMU.H)), IMU.P)
	#print IMU.P
	n +=1