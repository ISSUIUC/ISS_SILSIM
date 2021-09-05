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

#### MacOS Installation Process

There are few extra steps to get your simulator runnning on your Mac.

Install the `g++` compiler through brew:

```
brew install gcc
```

Open `~/.bash_profile` in `nano`:

```
nano ~/.bash_profile
```

Add the following lines to the end of the file:

```
# Setting PATH for local executables
export PATH=/usr/local/bin:$PATH
```

Add symbolic link from `g++-11` to `g++`:

```
ln -s /usr/local/bin/g++-11 /usr/local/bin/g++
```


## Feature ToDo List

- [x] Incorporate sensors into Rocket object
	- [x] Create a list of sensors
	- [x] Enumerate sensors and update their data
	- [x] Add noise and bias injection
- [ ] Create a "FlightSoftware" class that encapsulates flight software being tested.
	- [ ] Could have containers that sort-of emulate threads
	- [ ] Some sort of way of injecting faults or halting threads would be cool
- [ ] Utilize an XML parsing library to take in OpenRocket design files (.ork)
	- [ ] Choose one of many libraries; RapidXML, pugiXML, etc..
	- [ ] Implement a constructor for Rocket class that'll take a .ork file
- [ ] Create an "Atmosphere" class that governs air pressure/density properties
	- [ ] Parameters could vary by altitude
	- [ ] How to propogate atmospheric variables to other classes?
	- [ ] Use existing atmospheric databases and interpolate
	- [ ] How to model wind???
- [ ] Look into a better method of trajectory visualtization
	- [ ] Possibly PyBullet?
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
- [ ] Added documentation for MacOS users
