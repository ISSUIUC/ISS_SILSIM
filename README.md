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
	- Create a list of sensors
	- Enumerate sensors and update their data
- [ ] Create an "Atmosphere" class that governs air pressure/density properties
	- Parameters could vary by altitude
	- How to propogate atmospheric variables to other classes?
	- Use existing atmospheric databases and interpolate
- [ ] Look into a better method of trajectory visualtization
	- Possibly PyBullet?
- [ ] Create a "FlightSoftware" class that encapsulates flight software being tested.
	- Could have containers that sort-of emulate threads
	- Some sort of way of injecting faults or halting threads would be cool
- [ ] Add graphical arrows in simulation playback to illustrate in-flight forces
