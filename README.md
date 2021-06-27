# ISS_SILSIM

ISS Software-In-The-Loop Simulator

Non-linear 6DOF rocket trajectory simulation utility

## Dependencies
- Linux Operating System
	- Tested on Ubuntu 20.04 (Recommended)
- GNU Make
- Python3
	- matplotlib package

## Instructions

Modify parameters in `src/main.cpp` as desired, then run the following:

```
make
./main
python3 sim_data/plotter.py
```
or
```
./run_and_plot.sh
```
## Feature ToDo List

- [ ] Incorporate sensors into Rocket object
	- [ ] Create a list of sensors
	- [ ] Enumerate sensors and update their data
	- [ ] Add noise and bias injection
- [ ] Create an "Atmosphere" class that governs air pressure/density properties
	- [ ] Parameters could vary by altitude
	- [ ] How to propogate atmospheric variables to other classes?
	- [ ] Use existing atmospheric databases and interpolate
- [ ] Look into a better method of trajectory visualtization
	- [ ] Possibly PyBullet?
- [ ] Create a "FlightSoftware" class that encapsulates flight software being tested.
	- [ ] Could have containers that sort-of emulate threads
	- [ ] Some sort of way of injecting faults or halting threads would be cool
- [ ] Add graphical arrows in simulation playback to illustrate in-flight forces
- [ ] Make a logging library that is modular and accepts data from configurable sources
	- [ ] Decide on mechanism and architecture of logging system
		- Single file or multiple files?
		- Human readable format? - need a parser if not human readable
	- [ ] Implement a LogAggregator class that collects logs from all sources
	- [ ] Implement a LogSource class that supplies LogAggregator with data 
	- [ ] Perhaps some phython to parse the logged data?

## Improvements/Fixes ToDo List

- [ ] Added functions to Barometer sensor class to either read altitude or air pressure of various units
