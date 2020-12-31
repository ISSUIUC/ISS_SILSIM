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
