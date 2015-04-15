from robotcar import robotcar
from kalman_filter import kalman_filter
from random import seed, uniform
import numpy as np
import matplotlib.pyplot as plt


# Create Random Physical Landmarks
num_landmarks = 20
landmark_range = 300
landmarksx = np.empty(num_landmarks)
landmarksy = np.empty(num_landmarks)
seed()
for i in range(num_landmarks):
	landmarksx[i] = uniform(-landmark_range,landmark_range)
	landmarksy[i] = uniform(-landmark_range,landmark_range)

# Initialize Car and Kalman Filter
robot = robotcar(2, 0.5, num_landmarks, ts=0.1)
kf = kalman_filter(robot)

# Time Period
time = 120 # sec
n = int(time/robot.ts)

# Initialize empty arrays for plotting
t = np.empty(n)
e = np.empty(n)
x = np.empty(n)
y = np.empty(n)
x_pred = np.empty(n)
y_pred = np.empty(n)
x_update = np.empty(n)
y_update = np.empty(n)

print "START \n\n\n"

for i in range(n):
	# Create random movement of wheels
	l_wheel = uniform(5, 15)
	r_wheel = uniform(5, 15)
	robot.move_wheels(l_wheel, r_wheel)

	# Prediction Step
	kf.predict()

	# Add data to plot array
	x_pred[i] = robot.position[0,0]
	y_pred[i] = robot.position[1,0]

	# Update Steps - Perform an update to the current prediction for all landmarks
	for j in range(num_landmarks):
		landmarkx = landmarksx[j]
		landmarky = landmarksy[j]
		kf.update(landmarkx, landmarky, j)

	# Uncomment to print all position data
	# print "Run " + str(i) + ", updated: \n" + str(robot.position[0:3, 0])
	# print "Run " + str(i) + ", actual: \n" + str(robot.positionVector)

	# Add data to plot arrays
	x_update[i] = robot.position[0,0]
	y_update[i] = robot.position[1,0]
	x[i] = robot.positionVector[0]
	y[i] = robot.positionVector[1]
	t[i] = robot.time



# Plot map and error
e = np.sqrt(np.square(x_update - x) + np.square(y_update-y))
print "Average RMS Error of Position: " + str(np.mean(e))


plt.figure(1)
actual, = plt.plot(x, y)
predicted, = plt.plot(x_pred, y_pred)
updated, = plt.plot(x_update, y_update)
lms, = plt.plot(landmarksx, landmarksy, 'o', label="Landmarks")
plt.figlegend( (actual, predicted, updated, lms), ('Actual Position', 'Predicted Position','Updated Position', 'Landmarks'), 'lower right')
plt.title('Map of Actual, Predicted and Updated Position')
plt.xlabel('X Position')
plt.ylabel('Y Position')

plt.figure(2)
plt.plot(t,e)
plt.title('Root Mean Square Error Between Actual and Updated Position')
plt.xlabel('Time (Sec)')
plt.ylabel('Error')
plt.show()



