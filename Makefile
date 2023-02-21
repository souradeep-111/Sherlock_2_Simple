CXX = g++


include Makefile.locale
GUROBI_INCLUDEDIR=$(strip $(GUROBI_PATH))/$(strip $(HOST_ARCH))/include/
GUROBI_LIBDIR=$(strip $(GUROBI_PATH))/$(strip $(HOST_ARCH))/lib/

#-D_GLIBCXX_USE_CXX11_ABI=0
LIBS = -lgurobi_c++ -lgurobi100 -m64 -lprotoc -lprotobuf -lm -w -pthread -lpthread -std=c++17

# CXXFLAGS = -MMD -I . -I ./src/ -I /usr/local/include/ -I $(GUROBI_INCLUDEDIR) -g -O3 -std=c++17
CXXFLAGS = -I . -I ./src/ -I /usr/local/include/ -I $(GUROBI_INCLUDEDIR) -g -O3 -std=c++17

LINK_FLAGS = -g -L ./ -L /usr/local/lib/ -L $(GUROBI_LIBDIR)

OBJS = ./src/onnx.pb.o ./src/sherlock_message.pb.o ./src/sherlock.o ./src/network_computation.o \
./src/configuration.o ./src/nodes.o ./src/computation_graph.o ./src/region_constraints.o \
./src/generate_constraints.o ./src/network_signatures.o ./src/gurobi_interface.o  \
./src/parsing_onnx.o  ./src/image_handler.o ./src/sherlock_poly.o \
# ./src/selective_binarization.o

DEPENDS = ${OBJECTS:.o=.d}


all: libs run_file
interface_python: libs python_interface

# protoc -I=./src/ --cpp_out=./src/ ./src/onnx.proto

libs: $(OBJS)
	ar rcs ./src/libsherlock.a $(OBJS)
	ranlib ./src/libsherlock.a
	cp ./src/*.h ./include

run_file: main.o $(OBJS)
	$(CXX) -O3 -w $(LINK_FLAGS) -o $@ $^ $(LIBS)

python_interface: ./src/python_interface.o $(OBJS)
	$(CXX) -O3 -w $(LINK_FLAGS) -o $@ $^ $(LIBS)


%.o: %.cc
	$(CXX) -O3 -c $(CXXFLAGS) -o $@ $< $(LIBS)
%.o: %.cpp
	$(CXX) -O3 -c $(CXXFLAGS) -o $@ $< $(LIBS)
%.o: %.c
	$(CXX) -O3 -c $(CXXFLAGS) -o $@ $< $(LIBS)
%.pb.o: %.proto
	protoc -I=./src/ --cpp_out=./src/ ./src/onnx.proto
	protoc -I=./src/ --cpp_out=./src/ --python_out=./src/ ./src/sherlock_message.proto
	$(CXX) -O3 -c $(CXXFLAGS) -o ./src/onnx.pb.o ./src/onnx.pb.cc $(LIBS)
	$(CXX) -O3 -c $(CXXFLAGS) -o ./src/sherlock_message.pb.o ./src/sherlock_message.pb.cc $(LIBS)

clean:
	rm -f ./src/*.o  *.o ./run_file ./lib/* ./include/*.h ./src/*.pb.*

-include ${DEPENDS}
