rm sim_data/data.csv
make clean
make
./main
python sim_data/plotter.py
