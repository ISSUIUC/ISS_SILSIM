# ISS_SILSIM

ISS Software-In-The-Loop Simulator

Non-linear 6DOF rocket trajectory simulation utility

## Dependencies
- Linux Operating System
	- Tested on Ubuntu 20.04 (Recommended)
- GNU Make
- Python3
	- matplotlib package

## Installation and Setup Instructions

Modify parameters in `src/main.cpp` as desired, then run the following:

```
cmake .
make
./ISS_SILSIM
python3 sim_data/plotter.py
```
or
```
./run_and_plot.sh
```

#### MacOS Installation

There are few extra steps to get your simulator runnning on your Mac.

Install the `g++` compiler through brew:

```
brew install gcc
```

Open `~/.bash_profile` in `nano`:

```
nano ~/.bash_profile
```

Add the following lines to the beginning of the file:

```
# Setting PATH for local executables
export PATH=/usr/local/bin:$PATH
```

Add symbolic link from `g++-11` to `g++`:

```
ln -s /usr/local/bin/g++-11 /usr/local/bin/g++
```
#### Windows Installation

On Windows it is recommended to use visual studio, but gcc and clang both work

Visual Studio instructions:

Get visual studio here: https://visualstudio.microsoft.com/

When installing visual studio install the c++ desktop development package

Once visual studio finishes installing clone the repo

`git clone https://github.com/ISSUIUC/ISS_SILSIM`

In visual studio choose "Open a local folder" and open the cloned repo

After visual studio finishes loading the project go to "Select Startup Item" in the top center of the screen and select "ISS_SILSIM.exe" from the dropdown

Build and run the project by clicking the run button or by pressing "F5"

#### Python Dependencies

Run the following command to include the dependencies for the Python simulator in your environment:

```
pip install -r requirements.txt
```


## Development Workflow

### Branch Naming Convention
Please use the following naming conventions when creating branches while developing:

- `user/<github-username>/<branch-name>` for individual tasks or contributions, or as a sandbox for yourself
- `feature/<branch-name>` for **new** functionality that didn't exist before
- `bug/<branch-name>` for bug fixes
- `general/<branch-name>` for overall repository organization or development pipeline tweaks
- `misc/<branch-name>` or `junk/<branch-name>` for just messing around :)

Please include the Trello ticket ID when relevant! i.e. for a ticket [AV-69] your branch might look like

`user/AyberkY/AV-69-implement-runga-kutta-integrator`
or
`feature/AV-69-create-data-logger-class`

### Code Style Guide
The repository now has a GitHub Actions instance that will automatically check for code style violations!

The Actions instance **will not** inhibit a pull-request from merging. It is merely there to _encourage_ style consistency throughout our code base.

There is also an auto formatting script that will _format your code for you_! (its beautiful, you should use it) This means that you don't have to worry about coding to meet the style yourself, as you can simply run the formatting script before you commit/push your changes.

You can run the script on Linux, Mac, or WSL like so:
```
clang-format -i **/*.cpp **/*.h
```

Things to keep in mind about code formatting:
- The code style being used is defined in `.clang-format`. It currently follows Google's C++ style guide exactly.
- Changing/tweaking the style guide is always option! If you have ideas, reach out!



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
- [x] Added documentation for MacOS users
