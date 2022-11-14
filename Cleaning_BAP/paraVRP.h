#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <map>



class Node
{
public:
	Node(std::string node_id_, int node_no_, std::string node_type_, float demand_, float service_time_, float ready_time_, float due_time_)
	{
		node_id = node_id_;
		node_no = node_no_;
		node_type = node_type_;
		demand = demand_;
		service_time = service_time_;
		ready_time = ready_time_;
		due_time = due_time_;
	}

	std::string node_id;
	int node_no;
	std::string node_type;
	float demand;
	float service_time;
	float ready_time;
	float due_time;
};



class Instance
{
public:
	Instance(const std::string instance_filename_, int time_limit_, int ub_strategy_)
	{
		instance_filename = instance_filename_;
		time_limit = time_limit_;
		ub_strategy = ub_strategy_;

		number_of_nodes = 0;
		very_big = 1e10;
		max_c2c_interlinks = 0;

		readInstanceData();
		initialization();
	}

	int time_limit;
	int ub_strategy;
	std::string instance_filename;

	float vehicle_water_capacity;
	float vehicle_acquisition_cost;
	float due_time;

	int number_of_nodes;
	std::vector<Node> node_vector;
	int start_depot_node_no, end_depot_node_no;
	int number_customers;
	std::vector<int> F_vector, C_vector, F0_vector, C0_vector, N_vector, N0_vector, Nn1_vector, N0n1_vector;
	std::map<int, int> customer_index_dict;			// customer_node_no: customer_index

	int max_c2c_interlinks;

	std::map<std::string, std::map<std::string, float>> travel_time_dict;
	std::map<std::string, std::map<std::string, int>> intermediate_links_dict;

	float **travel_time_array;
	int **intermediate_links_array;
	float **cost_array;
	
	std::string *node_type_array;
	bool *customer_node_flag_array;
	bool *water_node_flag_array;

	float *service_time_array;
	float *ready_time_array, *due_time_array;
	float *water_consumption_array;

	void readInstanceData();
	void initialization();

	float very_big;
};