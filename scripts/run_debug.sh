rm logs/silsim_datalog.log
./scripts/build-and-run-linux.sh
python data_processing/kalman_debug.py
