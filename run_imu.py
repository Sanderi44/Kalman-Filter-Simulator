from imu import imu

IMU = imu()

IMU.move_body(10.0, 10.0, 10.0)
IMU.get_predict_sensor()
IMU.get_update_sensor('acc')
print IMU.actual_pos
print IMU.w
print IMU.gyro