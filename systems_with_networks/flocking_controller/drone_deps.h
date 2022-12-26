#ifndef drone_deps_h
#define drone_deps_h
#include "sherlock_poly.h"

using namespace std;
using namespace std::chrono;


typedef map< string, double > Name_to_Value;
typedef map< string, pair< double, double > > Name_to_Interval;

typedef map< uint32_t, Name_to_Value > Plant_index_to_Value;
typedef map< uint32_t, Name_to_Interval > Plant_index_to_Interval;

typedef map< string, GRBVar > Name_to_Var;
typedef map< uint32_t, Name_to_Var > Plant_index_to_States;

void controller_range_propagation(string);

bool check_safety(Plant_index_to_Interval initial_position_limits,
                  double min_initial_distance, string controller_filename,
                  int simulation_steps, int steps_per_control, double min_final_distance,
                  Plant_index_to_Value & witness);

bool check_system_encoding(Plant_index_to_Interval initial_position_limits,
                            string controller_filename,
                            Plant_index_to_Value & witness,
                            int simulation_steps, int steps_per_control);
bool check_network_encoding(Plant_index_to_Interval all_plant_values,
                            string controller_filename);


void simulate_plant_and_controller(Plant_index_to_States & initial_plant_states,
                                   Plant_index_to_States & final_plant_states,
                                   string controller_filename, int simulation_steps, int steps_per_control,
                                   GRBModel * model_ptr);

void compute_dynamics(Name_to_Var & plant_current_state_,
                      int steps, Name_to_Var & plant_next_state,
                      double time_step, GRBModel * model_ptr);

void control_action( Plant_index_to_States & current_plant_states,
                     Plant_index_to_States & control_actions, string network_name,
                     uint32_t time_stamp, uint32_t no_of_relatives, GRBModel * model_ptr);

void implement_network( Plant_index_to_States & closest_k_plant_rel_states ,
                       pair<GRBVar, GRBVar> & network_output,
                       string network_name, GRBModel * model_ptr);

void closest_k_rel_pos( Plant_index_to_States & current_plant_states, uint32_t current_plant_index,
                       uint32_t no_of_relatives, Plant_index_to_States & closest_k_plant_rel_states,
                       GRBModel * model_ptr);

void generate_order(Plant_index_to_States & rel_plant_states,
                   Plant_index_to_States & sorted_rel_plant_states,
                   GRBModel * model_ptr);

void generate_permuation_constraint(Plant_index_to_States & rel_plant_states,
                                   Plant_index_to_States & sorted_rel_plant_states,
                                   GRBModel * model_ptr);

void generate_permutation_block(GRBVar & input_var, int size,
                               vector< vector< GRBVar > > & output_block,
                               GRBModel * model_ptr);

void generate_permutation_matrix(int size, vector< vector< GRBVar > > & output_block,
                               GRBModel * model_ptr);

void equate_target_variable_to_one_of_rhs(vector < GRBVar > & binary_vars,
                                         Name_to_Var & target_var,
                                         Plant_index_to_States & rhs_plants,
                                         GRBModel * model_ptr);

void compute_norm(Name_to_Var & input, GRBVar & output, GRBModel * model_ptr);
void print(Name_to_Interval interval);
void print(Name_to_Value value);
void print(Plant_index_to_Interval p_int);
void print(Plant_index_to_Value p_value);


#endif
