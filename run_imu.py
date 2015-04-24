from imu import imu
import numpy as np
IMU = imu()

IMU.move_body(10.0, 10.0, 10.0)
IMU.get_predict_sensor()
IMU.update_prediction_matrices()
#print IMU.process_model

IMU.P = np.dot(IMU.A, np.dot(IMU.P, IMU.A.T)) + np.dot(IMU.G, np.dot(IMU.Q, IMU.G.T))
#print IMU.P

print IMU.actual_pos
print IMU.process_model


IMU.get_update_sensor('acc')
IMU.update_correction_matrices()
Y = IMU.Z - IMU.H*IMU.states
print Y

kalman_gain = np.dot(np.dot(IMU.P, IMU.H.T), np.linalg.inv(np.dot(IMU.H, np.dot(IMU.P, IMU.H.T)) + np.dot(IMU.V, np.dot(IMU.R, IMU.V.T))))
print kalman_gain