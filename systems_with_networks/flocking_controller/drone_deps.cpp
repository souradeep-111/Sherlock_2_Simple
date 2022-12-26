#include "drone_deps.h"

bool debug_encoding = true;
map< string, GRBVar > trace;

void controller_range_propagation(string onnx_file)
{

  computation_graph CG;
  onnx_parser my_parser(onnx_file);
  map<string, ParameterValues < uint32_t > > tensor_mapping;
  my_parser.build_graph(CG, tensor_mapping);

  // Trying a sample range propagation through the controller
  map< uint32_t, pair< double, double > > interval;

  double delta = 0.1;
  interval[1] = make_pair(0, delta);
  interval[2] = make_pair(0, delta);
  interval[3] = make_pair(0, delta);
  interval[4] = make_pair(0, delta);
  interval[5] = make_pair(0, delta);
  interval[6] = make_pair(0, delta);
  interval[7] = make_pair(0, delta);
  interval[8] = make_pair(0, delta);

  region_constraints region;
  region.create_region_from_interval(interval);

  uint32_t output_index = 1009;
  pair<double, double > output_range;
  sherlock sherlock_instance(CG);
  sherlock_instance.compute_output_range(output_index, region, output_range);
  // Expected Value : [-0.812381 , -0.783142 ]
  cout << "Computed output range by Sherlock for output 1 = [" <<
  output_range.first << " , " << output_range.second << " ] " << endl;

  output_index = 1010;
  sherlock_instance.compute_output_range(output_index, region, output_range);
  // Expected Value : [0.50932 , 0.511107 ]
  cout << "Computed output range by Sherlock for output 2 = [" <<
  output_range.first << " , " << output_range.second << " ] " << endl;


}

bool check_safety(Plant_index_to_Interval initial_position_limits,
                  double min_initial_distance, string controller_filename,
                  int simulation_steps, int steps_per_control, double min_final_distance,
                  Plant_index_to_Value & witness)
{

  GRBEnv * env_ptr;
  GRBModel * model_ptr;
  env_ptr = new GRBEnv();
  erase_line();
  env_ptr->set(GRB_IntParam_OutputFlag, 0);
  model_ptr = new GRBModel(*env_ptr);
  model_ptr->set(GRB_DoubleParam_IntFeasTol, sherlock_parameters.int_tolerance);
  model_ptr->set(GRB_DoubleParam_TimeLimit, sherlock_parameters.timeout_seconds);


  assert(initial_position_limits.size() >= 2);

  // Declare Plant states for Initial Variables
  Plant_index_to_States initial_all_plant_states, final_all_plant_states, temp_closest_states;
  Name_to_Var plant_states;
  uint32_t no_of_plants = initial_position_limits.size();

  // Add initial interval constraints

  for(auto each_plant : initial_position_limits)
  {
    plant_states.clear();

    GRBVar new_x = model_ptr->addVar(each_plant.second["x"].first, each_plant.second["x"].second, 0.0,
    GRB_CONTINUOUS, "plant_x_init_" + to_string(each_plant.first) );
    plant_states["x"] =  new_x;

    GRBVar new_y = model_ptr->addVar(each_plant.second["y"].first, each_plant.second["y"].second, 0.0,
    GRB_CONTINUOUS, "plant_y_init_" + to_string(each_plant.first) );
    plant_states["y"] =  new_y;

    GRBVar new_vx = model_ptr->addVar(each_plant.second["vx"].first, each_plant.second["vx"].second, 0.0,
    GRB_CONTINUOUS, "plant_vx_init_" + to_string(each_plant.first));
    plant_states["vx"] =  new_vx;

    GRBVar new_vy = model_ptr->addVar(each_plant.second["vy"].first, each_plant.second["vy"].second, 0.0,
    GRB_CONTINUOUS, "plant_vy_init_" + to_string(each_plant.first) );
    plant_states["vy"] =  new_vy;

    initial_all_plant_states[each_plant.first] = plant_states;
  }

  GRBVar * closest_plant_distances = new GRBVar [no_of_plants];
  GRBVar init_min_dist;

  int index = 0;
  // Add initial distance constraints
  for(auto & each_plant : initial_all_plant_states)
  {
    closest_plant_distances[index] = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "_min_distance_for_" + to_string(each_plant.first));

    assert(initial_all_plant_states.size() > 1);
    closest_k_rel_pos(initial_all_plant_states, each_plant.first, 2, temp_closest_states, model_ptr);
    assert(temp_closest_states.size() == 2); // 2 since, the closest one is itself, 2nd is the closest other plant
    int order = 0;
    for(auto & plant_states : temp_closest_states)
    {
      if(order > 0) // the second one
      {
        compute_norm(plant_states.second, closest_plant_distances[index], model_ptr);
        break;
      }
      order++;
    }


    index++;
  }

  init_min_dist = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "_min_dist_across_all_plants_init_");
  model_ptr->addGenConstrMin(init_min_dist, closest_plant_distances, no_of_plants, GRB_INFINITY, "_comp_min_dist_init_");
  model_ptr->addConstr(init_min_dist, GRB_GREATER_EQUAL, min_initial_distance, "_closest_dist_lower_limit_init_");



    simulate_plant_and_controller(initial_all_plant_states, final_all_plant_states, controller_filename,
      simulation_steps, steps_per_control, model_ptr);



  // Add final distance limits constraints
  GRBVar * final_plant_distances = new GRBVar [no_of_plants];
  GRBVar final_min_dist;

  index = 0;
  // Add ifinal distance constraints
  for(auto & each_plant : final_all_plant_states)
  {
    final_plant_distances[index] = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "_min_distance_for_" + to_string(each_plant.first));

    closest_k_rel_pos(final_all_plant_states, each_plant.first, 2, temp_closest_states, model_ptr);
    assert(temp_closest_states.size() == 2); // 2 since, the closest one is itself, 2nd is the closest other plant

    int order = 0;
    for(auto & plant_states : temp_closest_states)
    {
      if(order > 0) // the second one
      {
        compute_norm(plant_states.second, final_plant_distances[index], model_ptr);
        break;
      }
      order++;
    }


    index++;
  }

  final_min_dist = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "_min_dist_across_all_plants_final_");
  model_ptr->addGenConstrMin(final_min_dist, final_plant_distances, no_of_plants, GRB_INFINITY, "_comp_min_dist_final_");
  model_ptr->addConstr(final_min_dist, GRB_LESS_EQUAL, min_final_distance, "_closest_dist_lower_limit_final_");


  // Optimize and get the counter example if any

  string s = "./Gurobi_file_created/Linear_program.lp";
  model_ptr->write(s);

  GRBLinExpr objective_expr;
  objective_expr = 0.0;
  model_ptr->setObjective(objective_expr, GRB_MINIMIZE);
  model_ptr->update();
  model_ptr->optimize();

  if(model_ptr->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
  {
      witness.clear();
      for(auto & each_plant : initial_all_plant_states)
      {
        Name_to_Value plant_value;
        plant_value.clear();

        for(auto & each_state : each_plant.second)
          plant_value[each_state.first] = each_state.second.get(GRB_DoubleAttr_X);

        witness[each_plant.first] = plant_value;
      }

      return false;
  }
  else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
  {
      witness.clear();
      return true;
  }
  else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INF_OR_UNBD)
  {
    model_ptr->set(GRB_IntParam_DualReductions, 0);
    model_ptr->update();
    model_ptr->optimize();
    if(model_ptr->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
    {
        witness.clear();
        for(auto & each_plant : initial_all_plant_states)
        {
          Name_to_Value plant_value;
          plant_value.clear();

          for(auto & each_state : each_plant.second)
            plant_value[each_state.first] = each_state.second.get(GRB_DoubleAttr_X);

          witness[each_plant.first] = plant_value;
        }

        return false;
    }
    else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
    {
        witness.clear();
        return true;
    }
  }
  else
  {
      cout << "Some unkown Gurobi flag during system simulation !" << endl;
      cout << "Flag returned - " << model_ptr->get(GRB_IntAttr_Status) << endl;
      cout << "At the time of writing the code the meaning of the Flags from Gurobi can be " <<
      "found here : https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html" << endl << endl;
      assert(false);
      return false;
  }

  if(model_ptr)
    delete model_ptr;
  if(env_ptr)
    delete env_ptr;
}

bool check_system_encoding(Plant_index_to_Interval initial_position_limits,
                            string controller_filename,
                            Plant_index_to_Value & witness,
                            int simulation_steps, int steps_per_control)
{
  GRBEnv * env_ptr;
  GRBModel * model_ptr;
  env_ptr = new GRBEnv();
  erase_line();
  env_ptr->set(GRB_IntParam_OutputFlag, 0);
  model_ptr = new GRBModel(*env_ptr);
  model_ptr->set(GRB_DoubleParam_IntFeasTol, sherlock_parameters.int_tolerance);
  model_ptr->set(GRB_DoubleParam_TimeLimit, sherlock_parameters.timeout_seconds);


  assert(initial_position_limits.size() >= 2);

  // Declare Plant states for Initial Variables
  Plant_index_to_States initial_all_plant_states, final_all_plant_states, temp_closest_states;
  Name_to_Var plant_states;
  uint32_t no_of_plants = initial_position_limits.size();

  // Add initial interval constraints

  for(auto each_plant : initial_position_limits)
  {
    plant_states.clear();

    GRBVar new_x = model_ptr->addVar(each_plant.second["x"].first, each_plant.second["x"].second, 0.0,
    GRB_CONTINUOUS, "plant_x_init_" + to_string(each_plant.first) );
    plant_states["x"] =  new_x;

    GRBVar new_y = model_ptr->addVar(each_plant.second["y"].first, each_plant.second["y"].second, 0.0,
    GRB_CONTINUOUS, "plant_y_init_" + to_string(each_plant.first) );
    plant_states["y"] =  new_y;

    GRBVar new_vx = model_ptr->addVar(each_plant.second["vx"].first, each_plant.second["vx"].second, 0.0,
    GRB_CONTINUOUS, "plant_vx_init_" + to_string(each_plant.first));
    plant_states["vx"] =  new_vx;

    GRBVar new_vy = model_ptr->addVar(each_plant.second["vy"].first, each_plant.second["vy"].second, 0.0,
    GRB_CONTINUOUS, "plant_vy_init_" + to_string(each_plant.first) );
    plant_states["vy"] =  new_vy;

    initial_all_plant_states[each_plant.first] = plant_states;
  }

  simulate_plant_and_controller(initial_all_plant_states, final_all_plant_states, controller_filename,
                                simulation_steps, steps_per_control, model_ptr);


  // Optimize and get the counter example if any

  GRBLinExpr objective_expr;
  objective_expr = 0;
  double data = 1.0;
  objective_expr.addTerms(& data, & final_all_plant_states[0]["x"], 1);

  if(debug_encoding)
  cout << "Pre adding objective " << endl;

  model_ptr->setObjective(objective_expr, GRB_MAXIMIZE);

  if(debug_encoding)
    cout << "Post adding objective " << endl;

  model_ptr->update();
  string s = "./Gurobi_file_created/Linear_program.lp";
  model_ptr->write(s);

  model_ptr->optimize();

  if(debug_encoding)
   cout << "Done optimizing " << endl;


  if(model_ptr->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
  {
    if(debug_encoding)
      cout << "Found an optimal value" << endl;
      witness.clear();
      for(auto & each_plant : final_all_plant_states)
      {
        Name_to_Value plant_value;
        plant_value.clear();

        for(auto & each_state : each_plant.second)
          plant_value[each_state.first] = each_state.second.get(GRB_DoubleAttr_X);

        witness[each_plant.first] = plant_value;
      }

      cout << " -------------------------------------------------------- " << endl;
      cout << "Printing the intermediate variable values here" << endl;
      for(auto & gurobi_vars : trace)
      {
          cout << gurobi_vars.first << " ---- " << gurobi_vars.second.get(GRB_DoubleAttr_X) << endl;
      }
      cout << " ------------------------------------------------------- " << endl;

      return false;
  }
  else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
  {
      witness.clear();
      return true;
  }
  else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INF_OR_UNBD)
  {
    model_ptr->set(GRB_IntParam_DualReductions, 0);
    model_ptr->update();
    model_ptr->optimize();
    if(model_ptr->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
    {
        witness.clear();
        for(auto & each_plant : final_all_plant_states)
        {
          Name_to_Value plant_value;
          plant_value.clear();

          for(auto & each_state : each_plant.second)
            plant_value[each_state.first] = each_state.second.get(GRB_DoubleAttr_X);

          witness[each_plant.first] = plant_value;
        }

        return false;
    }
    else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
    {
        witness.clear();
        return true;
    }
  }
  else
  {
      cout << "Some unkown Gurobi flag during system simulation !" << endl;
      cout << "Flag returned - " << model_ptr->get(GRB_IntAttr_Status) << endl;
      cout << "At the time of writing the code the meaning of the Flags from Gurobi can be " <<
      "found here : https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html" << endl << endl;
      assert(false);
      return false;
  }

  if(model_ptr)
    delete model_ptr;
  if(env_ptr)
    delete env_ptr;

}

bool check_network_encoding(Plant_index_to_Interval all_plant_values,
                            string controller_filename)
{
  GRBEnv * env_ptr;
  GRBModel * model_ptr;
  env_ptr = new GRBEnv();
  erase_line();
  env_ptr->set(GRB_IntParam_OutputFlag, 0);
  model_ptr = new GRBModel(*env_ptr);
  model_ptr->set(GRB_DoubleParam_IntFeasTol, sherlock_parameters.int_tolerance);
  model_ptr->set(GRB_DoubleParam_TimeLimit, sherlock_parameters.timeout_seconds);


  assert(all_plant_values.size() >= 2);

  // Declare Plant states for Initial Variables
  Plant_index_to_States all_plant_states;
  Name_to_Var plant_states;

  uint32_t no_of_plants = all_plant_values.size();

  // Add initial interval constraints

  all_plant_states.clear();

  for(auto each_plant : all_plant_values)
  {
    plant_states.clear();

    GRBVar new_x = model_ptr->addVar(each_plant.second["x"].first, each_plant.second["x"].second, 0.0,
    GRB_CONTINUOUS, "plant_x_init_" + to_string(each_plant.first) );
    plant_states["x"] =  new_x;

    GRBVar new_y = model_ptr->addVar(each_plant.second["y"].first, each_plant.second["y"].second, 0.0,
    GRB_CONTINUOUS, "plant_y_init_" + to_string(each_plant.first) );
    plant_states["y"] =  new_y;

    GRBVar new_vx = model_ptr->addVar(each_plant.second["vx"].first, each_plant.second["vx"].second, 0.0,
    GRB_CONTINUOUS, "plant_vx_init_" + to_string(each_plant.first));
    plant_states["vx"] =  new_vx;

    GRBVar new_vy = model_ptr->addVar(each_plant.second["vy"].first, each_plant.second["vy"].second, 0.0,
    GRB_CONTINUOUS, "plant_vy_init_" + to_string(each_plant.first) );
    plant_states["vy"] =  new_vy;

    all_plant_states[each_plant.first] = plant_states;
  }

  pair< GRBVar, GRBVar > network_output;
  implement_network(all_plant_states, network_output, controller_filename, model_ptr);

  GRBLinExpr objective_expr;
  objective_expr = 0;
  double data = 1.0;
  objective_expr.addTerms(& data, & network_output.first, 1);
  objective_expr.addTerms(& data, & network_output.second, 1);

  if(debug_encoding)
  cout << "Pre adding objective " << endl;

  model_ptr->setObjective(objective_expr, GRB_MAXIMIZE);

  if(debug_encoding)
    cout << "Post adding objective " << endl;

  model_ptr->update();
  string s = "./Gurobi_file_created/Linear_program.lp";
  model_ptr->write(s);

  model_ptr->optimize();

  if(debug_encoding)
   cout << "Done optimizing " << endl;


  if(model_ptr->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
  {
    cout << "Network output computed - " << endl;
    cout << "First output - " << network_output.first.get(GRB_DoubleAttr_X) << " Second Output - "
    << network_output.second.get(GRB_DoubleAttr_X) << endl;

      return false;
  }
  else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
  {
      return true;
  }
  else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INF_OR_UNBD)
  {
    model_ptr->set(GRB_IntParam_DualReductions, 0);
    model_ptr->update();
    model_ptr->optimize();
    if(model_ptr->get(GRB_IntAttr_Status) == GRB_OPTIMAL)
    {
      cout << "Network output computed - " << endl;
      cout << "First output - " << network_output.first.get(GRB_DoubleAttr_X) << " Second Output - "
      << network_output.second.get(GRB_DoubleAttr_X) << endl;
      return false;
    }
    else if(model_ptr->get(GRB_IntAttr_Status) == GRB_INFEASIBLE)
    {
        return true;
    }
  }
  else
  {
      cout << "Some unkown Gurobi flag during system simulation !" << endl;
      cout << "Flag returned - " << model_ptr->get(GRB_IntAttr_Status) << endl;
      cout << "At the time of writing the code the meaning of the Flags from Gurobi can be " <<
      "found here : https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html" << endl << endl;
      assert(false);
      return false;
  }

  if(model_ptr)
    delete model_ptr;
  if(env_ptr)
    delete env_ptr;


}

void simulate_plant_and_controller(
                        Plant_index_to_States & initial_plant_states,
                        Plant_index_to_States & final_plant_states,
                        string controller_filename, int simulation_steps, int steps_per_control,
                        GRBModel * model_ptr)
{

  /*
    State coordinates have an [ p-1 -- { (x,y), (v_x,v_y) }, p-2 --- { (x,y), (v_x,v_y) } ]
    Output is a 'k' step simulation of the plant
  */

  // Set the states limits
  if(debug_encoding)
    cout << "Enters simulate plant and controller " << endl;

  uint32_t no_of_relatives = 2;
  double time_step = 0.1;
  // Declare gurobi variables for states
  map< uint32_t, Plant_index_to_States > time_stamped_variables; // Time --> variables

  Name_to_Var plant_states, control_actions, next_state;
  plant_states.clear();
  Plant_index_to_States current_states_of_plants, next_states_of_plants, controls;

  time_stamped_variables[0] = initial_plant_states;

  for(auto i = 0; i < simulation_steps; i++)
  {
    if(debug_encoding)
      cout << "At simulation step - " << i << endl;

    current_states_of_plants.clear();
    next_states_of_plants.clear();

    // Get the variables for time 'i'
    current_states_of_plants = time_stamped_variables[i];

    control_action( current_states_of_plants, controls, controller_filename,
      i, no_of_relatives, model_ptr);

    if(debug_encoding)
      cout << "Control actions added " << endl;
    // Simulate each plant

    for(auto & each_plant : current_states_of_plants)
    {
      plant_states = each_plant.second;
      control_actions = controls[each_plant.first];
      plant_states["ax"] = control_actions["ax"];
      plant_states["ay"] = control_actions["ay"];

      compute_dynamics(plant_states, steps_per_control, next_state, time_step, model_ptr);
      next_states_of_plants[each_plant.first] = next_state;
    }

    // Assign that to time 'i+1'
    time_stamped_variables[i+1] = next_states_of_plants;
  }

  final_plant_states = time_stamped_variables[simulation_steps];

}


void compute_dynamics(Name_to_Var & plant_current_state_, /*Plant State and Control Vars */
                      int steps, Name_to_Var & plant_next_state,
                      double time_step, GRBModel * model_ptr)
{
  /*
    Input : Plant state current, control action current, steps - k
    Output : Next Plant State, 'k' steps in the future
  */

  Name_to_Var plant_current_state = plant_current_state_ ;

  for(auto current_step = 0; current_step < steps; current_step ++ )
  {
    // pos new = pos old + time_step * vel
    GRBVar new_x = model_ptr->addVar(-GRB_INFINITY,GRB_INFINITY, 0.0, GRB_CONTINUOUS, "next_x_state_" + to_string(current_step+1));
    GRBLinExpr expr = 0;
    double data;
    data = 1.0;
    expr.addTerms(& data, & plant_current_state["x"], 1);
    data = time_step;
    expr.addTerms(& data, & plant_current_state["vx"], 1);
    model_ptr->addConstr(new_x, GRB_EQUAL, expr, "_assigning_new_x_");
    plant_next_state["x"] = new_x;


    GRBVar new_y = model_ptr->addVar(-GRB_INFINITY,GRB_INFINITY, 0.0, GRB_CONTINUOUS, "next_y_state_"+ to_string(current_step+1));
    expr = 0;
    data = 1.0;
    expr.addTerms(& data, & plant_current_state["y"], 1);
    data = time_step;
    expr.addTerms(& data, & plant_current_state["vy"], 1);
    model_ptr->addConstr(new_y, GRB_EQUAL, expr, "_assigning_new_y_");
    plant_next_state["y"] = new_y;

    GRBVar new_vx = model_ptr->addVar(-GRB_INFINITY,GRB_INFINITY, 0.0, GRB_CONTINUOUS, "next_vx_state_"+ to_string(current_step+1));
    expr = 0;
    data = 1.0;
    expr.addTerms(& data, & plant_current_state["vx"], 1);
    data = time_step;
    expr.addTerms(& data, & plant_current_state["ax"], 1);
    model_ptr->addConstr(new_vx, GRB_EQUAL, expr, "_assigning_new_vx_");
    plant_next_state["vx"] = new_vx;


    GRBVar new_vy = model_ptr->addVar(-GRB_INFINITY,GRB_INFINITY, 0.0, GRB_CONTINUOUS, "next_vy_state_"+ to_string(current_step+1));
    expr = 0;
    data = 1.0;
    expr.addTerms(& data, & plant_current_state["vy"], 1);
    data = time_step;
    expr.addTerms(& data, & plant_current_state["ay"], 1);
    model_ptr->addConstr(new_vy, GRB_EQUAL, expr, "_assigning_new_vy_");
    plant_next_state["vy"] = new_vy;

    plant_current_state = plant_next_state;
    // Using the constant control input for the whole sim duration
    plant_current_state["ax"] = plant_current_state_["ax"];
    plant_current_state["ay"] = plant_current_state_["ay"];

  }

}

void control_action( Plant_index_to_States & current_plant_states,
                     Plant_index_to_States & control_actions, string network_name,
                     uint32_t time_stamp, uint32_t no_of_relatives, GRBModel * model_ptr)
{

  assert(! current_plant_states.empty());
  /*
    Here is how a control action is computed :
    1. Take the position of all the 'n' plants involved
    2. Find the relative positions of the closest 'k' plants with respect to each of the plants
    3. Calculate the relative positions of the closest 'k' plants
    4. Make a column vector output of [rel_x_1, rel_x_2, rel_x_3, ... , rel_x_k,  rel_y_1, rel_y_2, rel_y_3, .. rel_y_k,
       abs_vel_x_1, abs_vel_x_2, .... , abs_vel_x_k, abs_vel_y_1, abs_vel_y_2, abs_vel_y_3, ...., abs_vel_y_k]
       Note : rel_x_1 = 0, rel_y_1 = 0
    5. Compute the control output, [a_x, a_y] and then saturate the control action
  */

  Name_to_Var current_accn;
  Plant_index_to_States closest_k_plant_rel_states;
  control_actions.clear();

  for(auto each_plant : current_plant_states)
  {
    uint32_t current_plant_index = each_plant.first;

    if(debug_encoding)
    {
      trace["curr index - " + to_string(current_plant_index) + " plant 0 , x - "] = current_plant_states[0]["x"];
      trace["curr index - " + to_string(current_plant_index) + " plant 0 , y - "] = current_plant_states[0]["y"];
      trace["curr index - " + to_string(current_plant_index) + " plant 0 , vx - "] = current_plant_states[0]["vx"];
      trace["curr index - " + to_string(current_plant_index) + " plant 0 , vy - "] = current_plant_states[0]["vy"];

      trace["curr index - " + to_string(current_plant_index) + " plant 1 , x - "] = current_plant_states[1]["x"];
      trace["curr index - " + to_string(current_plant_index) + " plant 1 , y - "] = current_plant_states[1]["y"];
      trace["curr index - " + to_string(current_plant_index) + " plant 1 , vx - "] = current_plant_states[1]["vx"];
      trace["curr index - " + to_string(current_plant_index) + " plant 1 , vy - "] = current_plant_states[1]["vy"];
    }

    closest_k_rel_pos(current_plant_states, current_plant_index, no_of_relatives,
                      closest_k_plant_rel_states, model_ptr);

    assert(closest_k_plant_rel_states.size() == no_of_relatives);

    if(debug_encoding)
    {
      trace["curr index - " + to_string(current_plant_index) + " closest 1st relative x coord - "] = closest_k_plant_rel_states[0]["x"];
      trace["curr index - " + to_string(current_plant_index) + " closest 1st relative y coord - "] = closest_k_plant_rel_states[0]["y"];
      trace["curr index - " + to_string(current_plant_index) + " closest 1st relative vx coord - "] = closest_k_plant_rel_states[0]["vx"];
      trace["curr index - " + to_string(current_plant_index) + " closest 1st relative vy coord - "] = closest_k_plant_rel_states[0]["vy"];

      trace["curr index - " + to_string(current_plant_index) + " closest 2nd relative x coord - "] = closest_k_plant_rel_states[1]["x"];
      trace["curr index - " + to_string(current_plant_index) + " closest 2nd relative y coord - "] = closest_k_plant_rel_states[1]["y"];
      trace["curr index - " + to_string(current_plant_index) + " closest 2nd relative vx coord - "] = closest_k_plant_rel_states[1]["vx"];
      trace["curr index - " + to_string(current_plant_index) + " closest 2nd relative vy coord - "] = closest_k_plant_rel_states[1]["vy"];
    }

    // GRBVar ax = model_ptr->addVar(-GRB_INFINITY,GRB_INFINITY, 0.0, GRB_CONTINUOUS,
    //                       "control_" + to_string(current_plant_index) + "_x_" + to_string(time_stamp));
    // GRBVar ay = model_ptr->addVar(-GRB_INFINITY,GRB_INFINITY, 0.0, GRB_CONTINUOUS,
    //                       "control_" + to_string(current_plant_index) + "_y_" + to_string(time_stamp));

    pair< GRBVar, GRBVar > network_output;
    implement_network(closest_k_plant_rel_states , network_output, network_name, model_ptr);

    current_accn.clear();
    current_accn["ax"] = network_output.first;
    current_accn["ay"] = network_output.second;

    control_actions[current_plant_index] = current_accn;

    if(debug_encoding)
    {
      trace["For plant " + to_string(current_plant_index) + " control ax - "] = current_accn["ax"];
      trace["For plant " + to_string(current_plant_index) + " control ay - "] = current_accn["ay"];
    }
  }


}

void implement_network( Plant_index_to_States & closest_k_plant_rel_states ,
                        pair<GRBVar, GRBVar> & network_output,
                        string network_name, GRBModel * model_ptr)
{
  //==========================================================
  // if(debug_encoding)
  // {
  //   network_output.first = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "_control_1");
  //   network_output.second = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "_control_2");
  //   model_ptr->addConstr(network_output.first, GRB_EQUAL, 1.0, "setting value to controls");
  //   model_ptr->addConstr(network_output.second, GRB_EQUAL, 1.0, "setting value to controls");
  //   return;
  // }
  //==========================================================

  if(debug_encoding)
    cout << "Enters implement network " << endl;


  assert(closest_k_plant_rel_states.size() >= 1);

  // Flatten the states into a vector for the neural network
  /*
    Make a column vector output of [rel_x_1, rel_x_2, rel_x_3, ... , rel_x_k,  rel_y_1, rel_y_2, rel_y_3, .. rel_y_k,
     abs_vel_x_1, abs_vel_x_2, .... , abs_vel_x_k, abs_vel_y_1, abs_vel_y_2, abs_vel_y_3, ...., abs_vel_y_k]
     Note : rel_x_1 = 0, rel_y_1 = 0
  */


  vector< GRBVar > flattened;

  // Collect all the plant state variable strings in a vector
  vector< string > state_var_names;

  // Setting thr order in which controller expects the inputs
  state_var_names.clear();
  state_var_names.push_back("x");
  state_var_names.push_back("y");
  state_var_names.push_back("vx");
  state_var_names.push_back("vy");


  flattened.clear();
  for(auto & each_state : state_var_names)
  {
    for(auto & each_plant : closest_k_plant_rel_states)
    {
      flattened.push_back(each_plant.second[each_state]);

    }
  }

  computation_graph CG;
  onnx_parser parser(network_name);
  map<string, ParameterValues < uint32_t > > tensor_mapping;
  parser.build_graph(CG, tensor_mapping);

  CG.mark_node_as_output(1009);
  CG.mark_node_as_output(1010);

  if(debug_encoding)
    cout << "Starting out adding network constraints " << endl;

  map< uint32_t, GRBVar > graph_inputs, graph_outputs;
  constraints_stack graph_constraints;
  graph_constraints.fill_model_with_graph_constraints(graph_inputs, graph_outputs, CG, model_ptr);


  assert(!graph_inputs.empty());
  assert(!graph_outputs.empty());

  assert(flattened.size() == graph_inputs.size());
  // Add a constraint which equates the graph inputs to the states from the plants

  int index = 0;
  for(auto & each_pair : graph_inputs)
  {
    model_ptr->addConstr(flattened[index], GRB_EQUAL, each_pair.second, "_control_input_equals_rel_pos _" );

    index ++;
  }



  assert(graph_outputs.size() == 2);
  network_output.first = graph_outputs[1009];
  network_output.second = graph_outputs[1010];


}

void closest_k_rel_pos( Plant_index_to_States & current_plant_states, uint32_t current_plant_index,
                        uint32_t no_of_relatives, Plant_index_to_States & closest_k_plant_rel_states,
                        GRBModel * model_ptr)
{
  assert(current_plant_states.size() > 1);
  assert(no_of_relatives >= 0);

  // Calculate the relative positions
  Plant_index_to_States rel_plant_states;
  rel_plant_states.clear();
  for(auto & each_plant_state : current_plant_states)
  {

    Name_to_Var relative_pos;
    relative_pos.clear();
    for(auto & state_var : each_plant_state.second)
    {

      if((state_var.first == "x") || (state_var.first == "y"))
      {
        GRBVar diff_var = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
                                            "diff_var_" + state_var.first);
        GRBLinExpr expr = 0;
        double data = -1.0;
        expr.addTerms( & data, & current_plant_states[current_plant_index][state_var.first] , 1.0);
        data = 1.0;
        expr.addTerms( & data, & state_var.second , 1.0);
        model_ptr->addConstr(diff_var, GRB_EQUAL, expr,
          "_rel_comp_" + state_var.first + "_plant_index_" + to_string(each_plant_state.first));
        relative_pos[state_var.first] = diff_var;

        if(debug_encoding)
        {
          trace["With respect to "+ to_string(current_plant_index)+ " for plant " + to_string(each_plant_state.first)
           + " " + state_var.first + " diff - " ] = diff_var;
        }
      }
      else if((state_var.first == "vx") || (state_var.first == "vy"))
      {
        if(debug_encoding)
        {
          trace["With respect to "+ to_string(current_plant_index) + " for plant " + to_string(each_plant_state.first)
          + " " + state_var.first + " diff - " ] = state_var.second;
        }
        relative_pos[state_var.first] = state_var.second;
      }


    }
    rel_plant_states[each_plant_state.first] = relative_pos;
  }

  // Generate the permuation matrix, and get the output variables
  Plant_index_to_States sorted_rel_plant_states;
  generate_order(rel_plant_states, sorted_rel_plant_states, model_ptr);

  // Impose the constraint on the first "no of relatives" variables
  closest_k_plant_rel_states.clear();

  int index = 0;
  for(auto it = sorted_rel_plant_states.begin(); it != sorted_rel_plant_states.end(); it++)
  {
    closest_k_plant_rel_states[it->first] = it->second;

    if(debug_encoding)
    {
      trace["After ordering for plant id " + to_string(current_plant_index) + " for closest index - " + to_string(index) + " x rel - "] = (it->second)["x"];
      trace["After ordering for plant id " + to_string(current_plant_index) + " for closest index - " + to_string(index) + " y rel - "] = (it->second)["y"];
      trace["After ordering for plant id " + to_string(current_plant_index) + " for closest index - " + to_string(index) + " vx rel - "] = (it->second)["vx"];
      trace["After ordering for plant id " + to_string(current_plant_index) + " for closest index - " + to_string(index) + " vy rel - "] = (it->second)["vy"];
    }

    index++;
    if(index == no_of_relatives)
      break;
  }


  assert(closest_k_plant_rel_states.size() == no_of_relatives);
}

void generate_order(Plant_index_to_States & rel_plant_states,
                          Plant_index_to_States & sorted_rel_plant_states,
                          GRBModel * model_ptr)
{

  // Generate the permutation constraint
  generate_permuation_constraint(rel_plant_states, sorted_rel_plant_states, model_ptr);

  map< uint32_t, GRBVar > norm_computed;
  // Compute norm of the relative plant states
  for(auto & each_plant : sorted_rel_plant_states)
  {
    GRBVar normed_var = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
      "norm_var" + to_string(each_plant.first));
    compute_norm(each_plant.second, normed_var, model_ptr);
    norm_computed[each_plant.first] = normed_var;

  }

  // Enforce that there is an ascending order of things
  for(auto & current_norm : norm_computed)
  {
    for(auto & other_norm : norm_computed)
    {
      if(other_norm.first > current_norm.first) // All the others that are after the current one
        model_ptr->addConstr(other_norm.second, GRB_GREATER_EQUAL, current_norm.second);
    }
  }

}


void generate_permuation_constraint(Plant_index_to_States & rel_plant_states,
                                    Plant_index_to_States & sorted_rel_plant_states,
                                    GRBModel * model_ptr)
{
  vector< vector< GRBVar > >  matrix_vars;
  int no_of_plants = rel_plant_states.size();
  assert(no_of_plants > 0);

  matrix_vars.clear();
  vector< GRBVar > row_vector;

  for(auto row_index = 0; row_index < no_of_plants; row_index++)
  {
    row_vector.clear();
    for(auto col_index = 0; col_index < no_of_plants; col_index++)
    {
      GRBVar new_var = model_ptr->addVar(0.0, 1.0, 0.0, GRB_BINARY,
        "_var_for_perm_" + to_string(row_index) + "_" + to_string(col_index));
      row_vector.push_back(new_var);
    }
    matrix_vars.push_back(row_vector);
  }

  // Asserting all row_sums = 0
  for(auto row_index = 0; row_index < no_of_plants; row_index++)
  {
    GRBLinExpr expr = 0.0;
    for(auto col_index = 0; col_index < no_of_plants; col_index++)
    {
      double data = 1.0;
      expr.addTerms( & data, & matrix_vars[row_index][col_index], 1);
    }
    model_ptr->addConstr(expr, GRB_EQUAL, 1.0, "_row_constr_");
  }

  // Asserting all col_sums = 0
  for(auto col_index = 0; col_index < no_of_plants; col_index ++)
  {
    GRBLinExpr expr = 0.0;
    for(auto row_index = 0; row_index < no_of_plants; row_index ++)
    {
      double data = 1.0;
      expr.addTerms( & data, & matrix_vars[row_index][col_index], 1);
    }
    model_ptr->addConstr(expr, GRB_EQUAL, 1.0, "_col_constr_");
  }


  // Create the new sorted plant variables
  sorted_rel_plant_states.clear();
  for(auto each_plant : rel_plant_states)
  {
    Name_to_Var plant_states;
    plant_states.clear();
    for(auto each_state : each_plant.second)
    {
      GRBVar new_var = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS,
        "_sorted_state_vars_" + to_string(each_plant.first) + "_" + each_state.first);
      plant_states[each_state.first] = new_var;
    }
    sorted_rel_plant_states[each_plant.first] = plant_states;
  }

  // Expressing the multiplication here
  /* Each plant is a linear combo of the plants states, and a corresponding binary variable */

  auto left_plant_id = 0;
  for(auto & each_plant : sorted_rel_plant_states)
  {
    uint32_t current_order = each_plant.first;
    Name_to_Var target_variable = each_plant.second;
    // Name_to_Expr rhs_expression;

    // Get the binary row vars
    vector< GRBVar > row_vector = matrix_vars[left_plant_id];
    assert(row_vector.size() == no_of_plants);

    // Set target variable equal to one of the plants
    equate_target_variable_to_one_of_rhs(row_vector, target_variable, rel_plant_states, model_ptr);

    left_plant_id++;
  }

}

void generate_permutation_block(GRBVar & input_var, int size,
                                vector< vector< GRBVar > > & output_block,
                                GRBModel * model_ptr)
{
  /*
    It basically implements the following, if input var is 1 => it's an idenity matrix ,
    if '0' it's a null matrix
  */

  output_block.clear();
  vector< GRBVar > row_vector;

  for(auto row_index = 0; row_index < size; row_index++)
  {
    row_vector.clear();
    for(auto col_index = 0; col_index < size; col_index++)
    {
      GRBVar new_var = model_ptr->addVar(0.0, 1.0, 0.0, GRB_BINARY,
        "_var_for_block_" + to_string(row_index) + "_" + to_string(col_index));

      if(row_index == col_index) /* Diagonal element equals input */
        model_ptr->addConstr(new_var, GRB_EQUAL, input_var, "diag_constr_for_block");
      else  /* Non diagonal element is 0 */
        model_ptr->addConstr(new_var, GRB_EQUAL, 0.0, "setting_to_0");

      row_vector.push_back(new_var);
    }
    output_block.push_back(row_vector);
  }

}

void generate_permutation_matrix(int size, vector< vector< GRBVar > > & output_block,
                                GRBModel * model_ptr)
{
  /*
    It basically implements the following, if input var is 1 => it's an idenity matrix ,
    if '0' it's a null matrix
  */

  output_block.clear();
  vector< GRBVar > row_vector;

  for(auto row_index = 0; row_index < size; row_index++)
  {
    row_vector.clear();
    for(auto col_index = 0; col_index < size; col_index++)
    {
      GRBVar new_var = model_ptr->addVar(0.0, 1.0, 0.0, GRB_BINARY,
        "_var_for_block_" + to_string(row_index) + "_" + to_string(col_index));
      row_vector.push_back(new_var);
    }
    output_block.push_back(row_vector);
  }

}

void equate_target_variable_to_one_of_rhs(vector < GRBVar > & binary_vars,
                                          Name_to_Var & target_var,
                                          Plant_index_to_States & rhs_plants,
                                          GRBModel * model_ptr)
{
  /*
    Here is what it implements :
    if I have to say y_1 = b_1 x_1 + b_2 x_2
    The MILP constraints are :
      y_1 >= x_1 - (1-b_1)M
      y_1 =< x_1 + (1-b_1)M
      y_1 >= x_2 - (1-b_2)M
      y_1 =< x_2 + (1-b_2)M
  */
  assert(binary_vars.size() == rhs_plants.size());

  GRBVar gurobi_one = model_ptr->addVar(1.0, 1.0, 0.0, GRB_CONTINUOUS, "_1_");
  uint32_t binary_index;
  for(auto & target_state : target_var)
  {
      binary_index = 0;
      for(auto & plant : rhs_plants)
      {
        GRBLinExpr expr = 0;
        double data = 1.0;
        expr.addTerms(& data, & plant.second[target_state.first], 1.0);
        data = -sherlock_parameters.MILP_M;
        expr.addTerms(& data, & gurobi_one, 1.0);
        data = sherlock_parameters.MILP_M;
        expr.addTerms(& data, & binary_vars[binary_index], 1.0);

        model_ptr->addConstr(target_state.second, GRB_GREATER_EQUAL, expr, "greater_than_constr");

        expr = 0;
        data = 1.0;
        expr.addTerms(& data, & plant.second[target_state.first], 1.0);
        data = sherlock_parameters.MILP_M;
        expr.addTerms(& data, & gurobi_one, 1.0);
        data = -sherlock_parameters.MILP_M;
        expr.addTerms(& data, & binary_vars[binary_index], 1.0);

        model_ptr->addConstr(target_state.second, GRB_LESS_EQUAL, expr, "less_than_constr");

        binary_index++;
      }
  }

}

void compute_norm(Name_to_Var & input, GRBVar & output, GRBModel * model_ptr)
{
  output = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "_norm_var_");

  GRBLinExpr expr = 0;
  double data = 1.0;
  GRBVar rel_x = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "abs_rel_x");
  model_ptr->addGenConstrAbs(rel_x, input["x"], "abs constr");
  expr.addTerms(& data, & rel_x, 1.0);

  data = 1.0;
  GRBVar rel_y = model_ptr->addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "abs_rel_y");
  model_ptr->addGenConstrAbs(rel_y, input["y"], "abs constr");
  expr.addTerms(& data, & rel_y, 1.0);

  model_ptr->addConstr(output, GRB_EQUAL, expr, "L1-norm");
}

void print(Name_to_Interval interval)
{
  cout << " ------ " << endl;
  for(auto each_val : interval)
  {
    cout << "  " << each_val.first << " = <" << each_val.second.first << " , "
    << each_val.second.second  << " > , " ;
  }
  cout << " ------ " << endl;
}
void print(Name_to_Value value)
{
  cout << " ------ " << endl;
  for(auto each_val : value)
  {
    cout << "  " << each_val.first << " = " << each_val.second  << " , " ;
  }
  cout << " ------ " << endl;
}

void print(Plant_index_to_Interval p_int)
{
  for(auto each_plant : p_int)
  {
    cout << " ------------------------- " << endl;
    cout << "Interval for plant index - " << each_plant.first << endl;
    print(each_plant.second);
    cout << " ------------------------- " << endl;
  }
}

void print(Plant_index_to_Value p_value)
{
  for(auto each_plant : p_value)
  {
    cout << " ------------------------- " << endl;
    cout << "Values for plant index - " << each_plant.first << endl;
    print(each_plant.second);
    cout << " ------------------------- " << endl;
  }
}
