	/*

Contributors to the tool :
Souradeep Dutta

email : duttasouradeep39@gmail.com

LICENSE : Please see the license file, in the main directory

*/

#include "sherlock.h"
#include "sherlock_poly.h"

using namespace std;
using namespace std::chrono;

int main(int argc, char ** argv)
{
	sherlock_parameters.thread_count = 1;
	sherlock_parameters.do_incremental_constant_search = true;
	sherlock_parameters.verbosity = false;
	sherlock_parameters.grad_search_point_verbosity = false;
	sherlock_parameters.time_verbosity = false;
	sherlock_parameters.skip_invariant_guarantees_in_binarization = true;
	sherlock_parameters.skip_invariant_addition = true;
	sherlock_parameters.MILP_M = 1e4;
	sherlock_parameters.verbose_onnx = false;
	sherlock_parameters.use_gurobi_internal_constraints = true;
	sherlock_parameters.find_extra_directions = true;

	string random_network = "./sherlock_python_interface/random_network.onnx";

	computation_graph CG;
	onnx_parser my_parser(random_network);
	map<string, ParameterValues <uint32_t> > tensor_mapping;
	my_parser.build_graph(CG, tensor_mapping);

	map<uint32_t , double > inputs, outputs;

  uint32_t output_index = 203;
  set < uint32_t > input_indices, output_indices;
  input_indices.insert(1);
  input_indices.insert(2);
  output_indices.insert(output_index);

	// Testing for a sample input
	{
		inputs[1] = 1.0;
		inputs[2] = 2.0;
	  CG.evaluate_graph(inputs, outputs);
	  cout << "Test Output - " << outputs[output_index] << endl;
	}

  map< uint32_t, pair< double, double > > input_interval;
  input_interval[1] = make_pair(0,1);
  input_interval[2] = make_pair(0,1);
  region_constraints input_polyhedron, output_polyhedron;
  input_polyhedron.create_region_from_interval(input_interval);

  sherlock sherlock_instance(CG);

  pair< double, double > output_range;
  sherlock_instance.compute_output_range(output_index, input_polyhedron, output_range);
  cout << "Computed output range by Sherlock = [" <<
	output_range.first << " , " << output_range.second << " ] " << endl;

  sherlock_instance.compute_output_range_by_sampling(input_polyhedron, output_index, output_range, 1000);
	cout << "Computed output range from random sampling = [" <<
	output_range.first << " , " << output_range.second << " ] " << endl;

  return 0;
}
