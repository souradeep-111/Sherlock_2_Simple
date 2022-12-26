#!/bin/bash

make clean 
make

python3 sherlock_python_interface/generate_network.py --seed=1
./run_file
echo -e "\n"

python3 sherlock_python_interface/generate_network.py --seed=2
./run_file
echo -e "\n"

python3 sherlock_python_interface/generate_network.py --seed=3
./run_file
echo -e "\n"

python3 sherlock_python_interface/generate_network.py --seed=4
./run_file
echo -e "\n"

python3 sherlock_python_interface/generate_network.py --seed=5
./run_file
echo -e "\n"

python3 sherlock_python_interface/generate_network.py --seed=6
./run_file
echo -e "\n"

python3 sherlock_python_interface/generate_network.py --seed=7
./run_file
echo -e "\n"

python3 sherlock_python_interface/generate_network.py --seed=8
./run_file
echo -e "\n"

python3 sherlock_python_interface/generate_network.py --seed=9
./run_file
echo -e "\n"

python3 sherlock_python_interface/generate_network.py --seed=10
./run_file
echo -e "\n"
