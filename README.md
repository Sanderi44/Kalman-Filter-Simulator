# Kalman-Filter-Simulator
This is a simulator for a differential drive 2-wheel robot.  The robot uses simulated encoder and range finder data to attempt to determine the actual location of the robot.  It uses a Kalman filter to optimally asses where the robot is.  

## Setup
Make sure that you have python-virtualenv and python pip installed.
 
Clone the repository and cd into it.
```
	git clone https://github.com/Sanderi44/Kalman-Filter-Simulator.git
	cd Kalman-Filter-Simulator
```
Start the virtual environment 
``` 
	source venv/bin/activate
```
Install dependencies with requirements file:
```
	pip install -r requirements.txt
```

## Run
To run the program, simply type into the console, while in the virtual environment:
```
	python run.py
```
The settings that can be adjusted from the run file are:
	
	1.  The number of landmarks (set to 20)
	2.  The maximum distance that the landmarks can be (set to 300)
	3.  The length of the car (set to 2)
	4.  The radius of the wheels (set to 0.5)
	5.  The time period for simulation (set 120 sec)
	6.  The time step (set to 0.1 sec)
