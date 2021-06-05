rm sim_data/data.csv
make clean
make -j8
./main
python sim_data/plotter.py
