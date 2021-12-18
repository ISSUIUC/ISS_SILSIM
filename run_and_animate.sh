rm sim_data/data.csv
make clean
make -j8
./ISS_SILSIM
python3 sim_data/animation.py
