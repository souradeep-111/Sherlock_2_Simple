#include "sherlock_poly.h"
#include "drone_deps.h"

using namespace std;
using namespace std::chrono;

int main(int argc, char  ** argv)
{
  // if(debug_encoding)
  //   trace.clear();

  sherlock_parameters.thread_count = 1;
  sherlock_parameters.do_incremental_constant_search = true;
  sherlock_parameters.verbosity = false;
  sherlock_parameters.grad_search_point_verbosity = false;
  sherlock_parameters.time_verbosity = false;
  sherlock_parameters.skip_invariant_guarantees_in_binarization = true;
  sherlock_parameters.skip_invariant_addition = false;
  sherlock_parameters.MILP_M = 1e4;
  sherlock_parameters.verbose_onnx = false;
  sherlock_parameters.use_gurobi_internal_constraints = true;
  sherlock_parameters.timeout_seconds = 1e8;


  string filepath = "./systems_with_networks/flocking_controller/drone_controller.onnx";
  // controller_range_propagation(filepath);

  // sherlock_instance.compute_output_range_by_sampling(region, output_index, output_range, 1000);
  // cout << "Computed output range by sampling = [" <<
	// output_range.first << " , " << output_range.second << " ] " << endl;

  // Plant Dynamics Here
  // x(k+1) = x(k) + dt * v(k)
  // v(k+1) = v(k) + dt * a(k)

  // Number of plants in the current neural network : 2
  // Order of inputs seems to be : [relative pos, velocity]


  // Forming the initial states :
  Plant_index_to_Interval initial_position_limits;
  Plant_index_to_Value counter_example;

  Name_to_Interval Plant_1_init;
  // Plant_1_init["x"] = make_pair(1.4321, 1.4324);
  // Plant_1_init["y"] = make_pair(-1.8, -1.7096);
  // Plant_1_init["vx"] = make_pair(-0.7, -0.69);
  // Plant_1_init["vy"] = make_pair(0.79, 0.8);
  //
  // Name_to_Interval Plant_2_init;
  // Plant_2_init["x"] = make_pair(-0.3, -0.29);
  // Plant_2_init["y"] = make_pair(2.0, 2.08);
  // Plant_2_init["vx"] = make_pair(0.92, 0.93);
  // Plant_2_init["vy"] = make_pair(0.1, 0.2);

  Plant_1_init["x"] = make_pair(-3, -2.9);
  Plant_1_init["y"] = make_pair(-3, -2.9);
  Plant_1_init["vx"] = make_pair(-1, -0.9);
  Plant_1_init["vy"] = make_pair(1, 1.1);

  Name_to_Interval Plant_2_init;
  Plant_2_init["x"] = make_pair(3, 3.1);
  Plant_2_init["y"] = make_pair(3, 3.1);
  Plant_2_init["vx"] = make_pair(-1, -0.9);
  Plant_2_init["vy"] = make_pair(1, 1.1);

  initial_position_limits[0] = Plant_1_init;
  initial_position_limits[1] = Plant_2_init;
  double distance  = 2 * 1.414; // 2 * 1.414

  bool status = check_safety(initial_position_limits, distance,
                             filepath, 1, 2, distance, counter_example);


  if(status)
    cout << "Property holds" << endl;
  else
  {
    cout << "Property fails ! " << endl;
    cout << "Here is the counter example " << endl;
    print(counter_example);
  }
}
