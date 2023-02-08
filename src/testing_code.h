#ifndef _testing_code_h
#define _testing_code_h

#include <iostream>
#include <fstream>
#include <string>
#include "sherlock_message.pb.h"

using namespace std;

void test_1(int arg_count, char ** arg_val)
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

	string network_name;

	if (arg_count == 1)
	{
		network_name = "./sherlock_python_interface/random_network.onnx";
	}
	else if (arg_count == 2)
	{
		network_name = arg_val[1];
	}
	else
	{
		cout << "More arguments than expected ! Exiting " << endl;
		assert(false);
	}

	computation_graph CG;
	onnx_parser my_parser(network_name);
	map<string, ParameterValues <uint32_t> > tensor_mapping;
	_id_list_ input_indices, output_indices;
	my_parser.build_graph_and_return_indices(CG, tensor_mapping, input_indices, output_indices);


	assert (output_indices.size() == 1);
	uint32_t output_index = output_indices[0];

	// Testing for a sample input
	{
		map<uint32_t , double > inputs, outputs;
		inputs[1] = 1.0;
		inputs[2] = 2.0;
	  CG.evaluate_graph(inputs, outputs);
	  cout << "Test Output - " << outputs[output_index] << endl;
	}

  map< uint32_t, pair< double, double > > input_interval;

	for(uint32_t idx : input_indices)
	{
		input_interval[idx] = make_pair(0,1);
	}

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
}

void test_2(int arg_count, char ** arg_val)
{
  // Steps in this test file :
  // 1. Read the protobuf file which describes the problem
  // 2. Compute the optima using Sherlock
  // 3. Write a protobuf file which has the output range and the witness maybe in the sane file

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

  string network_name;

  assert (arg_count == 3);
  string sherlock_message_file = arg_val[1];

  GOOGLE_PROTOBUF_VERIFY_VERSION;
  __sherlock__ :: sherlock_message problem_description;

  fstream input_file(sherlock_message_file, ios::in | ios::binary);

  if (! problem_description.ParseFromIstream(& input_file))
  {
     cerr << "Failed to parse Sherlock Message " << endl;
     assert(false);
  }

  const __sherlock__::network & network_description = problem_description.network_description();
  const string & onnx_network_filename = network_description.onnx_filename();

  computation_graph CG;
  onnx_parser parser(onnx_network_filename);
  map < string, ParameterValues < uint32_t > > tensor_mapping;
  _id_list_ input_indices, output_indices;
  parser.build_graph_and_return_indices(CG, tensor_mapping, input_indices, output_indices);
  sherlock sherlock_instance(CG);

  // Reading the input intervals
  map< uint32_t, pair< double, double > > input_set;
  map< int, uint32_t > mapping_id_to_input_index;

  __sherlock__::interval input_interval = problem_description.input_interval();

  assert(input_interval.limits_size() == input_indices.size());
  for(int index = 0; index < input_interval.limits_size(); index++)
  {
    __sherlock__::mapping current_map = input_interval.limits(index);
    uint32_t ordered_index = current_map.node_index();
    input_set[input_indices[ordered_index]] = make_pair(current_map.lower_limit(), current_map.upper_limit());
    mapping_id_to_input_index[input_indices[ordered_index]] = ordered_index;

  }

  region_constraints input_polyhedron;
  input_polyhedron.create_region_from_interval(input_set);


  // Reading the output index for the correct node index
  __sherlock__ :: objective objective_function = problem_description.optimization_problem();

  // Restricting this to single interested node for now. This is not important just a random
  // design choice to test things quickly first
  assert (objective_function.linear_terms_size() == 1);
  __sherlock__ :: linear_combo term = objective_function.linear_terms(0);
  uint32_t ordered_index = term.node_index();

  uint32_t output_index;
  for(uint32_t idx = 0; idx < output_indices.size(); idx ++)
  {
    if(idx == ordered_index)
    {
      output_index = output_indices[idx];
      break;
    }
  }

  _point_ witness_point;
  // Hard coding the optimization direction here
  bool direction = objective_function.direction();
  double optima_computed;

  assert( objective_function.status_flag() == __sherlock__::objective::NOT_STARTED);


  sherlock_instance.optimize_using_gradient(output_index, input_polyhedron, direction,
                                                 optima_computed, witness_point);

  // Writing the computed optima to sherlock message

  problem_description.set_optima_val(optima_computed);

  // Modifying the optimal value witness
  __sherlock__ :: point * witness_val = problem_description.mutable_witness();
  for (auto const& x : witness_point)
  {
    __sherlock__ :: value_binding * value = witness_val->add_values();
    int32_t ordered_index = mapping_id_to_input_index[x.first];

    value->set_node_index(ordered_index);
    value->set_value(x.second);
  }


  // Change the output interval and see if you can write it back
  fstream output_file( sherlock_message_file , ios::out | ios::trunc | ios::binary);
    if (!problem_description.SerializeToOstream(& output_file)) {
      cerr << "Failed to write address book." << endl;
      assert(false);
    }


  google::protobuf::ShutdownProtobufLibrary();


}

void test_3(int arg_count, char ** arg_val)
{
  // Steps in this test file :
  // 1. Read the protobuf file which describes the problem
  // 2. Compute the optima using Sherlock's gradient search part
  // 3. Write a protobuf file which has the output range and the witness maybe in the sane file

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

  string network_name;

  assert (arg_count == 3);
  string sherlock_message_file = arg_val[1];

  GOOGLE_PROTOBUF_VERIFY_VERSION;
  __sherlock__ :: sherlock_message problem_description;

  fstream input_file(sherlock_message_file, ios::in | ios::binary);

  if (! problem_description.ParseFromIstream(& input_file))
  {
     cerr << "Failed to parse Sherlock Message " << endl;
     assert(false);
  }

  const __sherlock__::network & network_description = problem_description.network_description();
  const string & onnx_network_filename = network_description.onnx_filename();

  computation_graph CG;
  onnx_parser parser(onnx_network_filename);
  map < string, ParameterValues < uint32_t > > tensor_mapping;
  _id_list_ input_indices, output_indices;
  parser.build_graph_and_return_indices(CG, tensor_mapping, input_indices, output_indices);
  sherlock sherlock_instance(CG);

  // Reading the input intervals
  map< uint32_t, pair< double, double > > input_set;
  map< int, uint32_t > mapping_id_to_input_index;

  __sherlock__::interval input_interval = problem_description.input_interval();

  _point_ witness_point;
  assert(input_interval.limits_size() == input_indices.size());
  for(int index = 0; index < input_interval.limits_size(); index++)
  {
    __sherlock__::mapping current_map = input_interval.limits(index);
    uint32_t ordered_index = current_map.node_index();
    input_set[input_indices[ordered_index]] = make_pair(current_map.lower_limit(), current_map.upper_limit());
    mapping_id_to_input_index[input_indices[ordered_index]] = ordered_index;

    witness_point[input_indices[ordered_index]] =
    0.5 * (current_map.lower_limit() + current_map.upper_limit()) ;
  }

  region_constraints input_polyhedron;
  input_polyhedron.create_region_from_interval(input_set);


  // Reading the output index for the correct node index
  __sherlock__ :: objective objective_function = problem_description.optimization_problem();

  // Restricting this to single interested node for now. This is not important just a random
  // design choice to test things quickly first
  assert (objective_function.linear_terms_size() == 1);
  __sherlock__ :: linear_combo term = objective_function.linear_terms(0);
  uint32_t ordered_index = term.node_index();

  uint32_t output_index;
  for(uint32_t idx = 0; idx < output_indices.size(); idx ++)
  {
    if(idx == ordered_index)
    {
      output_index = output_indices[idx];
      break;
    }
  }

  // Hard coding the optimization direction here
  bool direction = objective_function.direction();
  double optima_computed;

  assert( objective_function.status_flag() == __sherlock__::objective::NOT_STARTED);


  // sherlock_instance.optimize_using_gradient(output_index, input_polyhedron, direction,
  //                                                optima_computed, witness_point);
  sherlock_instance.perform_gradient_search_with_random_restarts(output_index,
                        direction, input_polyhedron, witness_point, optima_computed );
  // Writing the computed optima to sherlock message

  problem_description.set_optima_val(optima_computed);

  // Modifying the optimal value witness
  __sherlock__ :: point * witness_val = problem_description.mutable_witness();
  for (auto const& x : witness_point)
  {
    __sherlock__ :: value_binding * value = witness_val->add_values();
    int32_t ordered_index = mapping_id_to_input_index[x.first];

    value->set_node_index(ordered_index);
    value->set_value(x.second);
  }


  // Change the output interval and see if you can write it back
  fstream output_file( sherlock_message_file , ios::out | ios::trunc | ios::binary);
    if (!problem_description.SerializeToOstream(& output_file)) {
      cerr << "Failed to write address book." << endl;
      assert(false);
    }


  google::protobuf::ShutdownProtobufLibrary();
}

void test_4()
{
  // This code test simple network creation :
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

  double weight = 1.0;
  node node_1(2, "relu");
  node_1.set_bias(0.0);
  node node_input(1, "constant");

  computation_graph CG;

  CG.add_new_node(1, node_input);
  CG.add_new_node(2, node_1);
  CG.connect_node1_to_node2_with_weight(1,2, weight);

  CG.mark_node_as_input(1);
  CG.mark_node_as_output(2);

  {
    map<uint32_t , double > inputs, outputs;
		inputs[1] = -1.0;
	  CG.evaluate_graph(inputs, outputs);
	  cout << "Test Output - " << outputs[2] << endl;
  }

  map< uint32_t, pair< double, double > > input_interval;
	input_interval[1] = make_pair(3,5);

	region_constraints input_polyhedron, output_polyhedron;
  input_polyhedron.create_region_from_interval(input_interval);

  sherlock sherlock_instance(CG);
  uint32_t output_index = 2;

  pair< double, double > output_range;
  sherlock_instance.compute_output_range(output_index, input_polyhedron, output_range);
  cout << "Computed output range by Sherlock = [" <<
	output_range.first << " , " << output_range.second << " ] " << endl;

  sherlock_instance.compute_output_range_by_sampling(input_polyhedron, output_index, output_range, 1000);
	cout << "Computed output range from random sampling = [" <<
	output_range.first << " , " << output_range.second << " ] " << endl;
}

#endif
