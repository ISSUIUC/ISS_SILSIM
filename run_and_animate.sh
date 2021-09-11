rm sim_data/data.csv
make clean
make -j8
./main
python3 sim_data/animation.py
