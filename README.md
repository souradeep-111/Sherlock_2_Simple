# sherlock

Sherlock is an output range analysis tool for deep neural networks.
The only library that is needed is the MILP solver Gurobi. It's free
for academic purposes and can be downloaded from here :
http://www.gurobi.com/resources/getting-started/mip-basics


## Instructions to Compile

Please modify the file Makefile.locale to help us find Gurobi.

For a Mac with the latest gurobi10.0 installed your likely settings
will be:

> HOST_ARCH=macos_universal2

> GUROBI\_PATH=/Library/gurobi1000

For a linux box, your settings will be: (Not tested yet)

> ARCH=linux64 # if you are using a linux box
> GUROBI\_PATH=/opt/gurobi1000

You should feel free to modify these two variables. The Makefile will look for Gurobi headers under

> $(GUROBI\\_PATH)/$(HOST_ARCH)/include

and libraries under

> $(GUROBI\\_PATH)/$(HOST_ARCH)/include


Once these are set, you should type

> make

to compile. It should work out of the box.

## Instructions to run

Please use the command line run_file to run all the benchmarks.

> ./run_file


Enjoy the output as it scrolls on your screen :-)

## Instructions to Test Against Randomly generated Pytorch networks

Sherlock can read networks in the ONNX format. Right now the test script
generates fixed architecture networks with random weights.

> bash test_script.sh

### Catch Alert !
Right now its not as automated as I would want it to be. You need to manually select
the index of the output node where you are interested. This can be obtained from the
variable index as $modelOutput$ when you execute the binary $runfile$. This matches the number assigned to output\_index in Line 41 of main.cpp
