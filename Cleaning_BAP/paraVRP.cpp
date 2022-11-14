#include <regex>
#include "paraVRP.h"
#include "utils.h"



void Instance::readInstanceData()
{
	std::ifstream infile;
	infile.open(instance_filename.data());

	if (!infile.is_open())
	{
		std::cout << "Cannot open file " << instance_filename << std::endl;
		ProgramStop();
	}

	bool node_flag = false;
	bool edge_flag = false;
	std::vector<std::string> node_info, edge_info;

	std::string s;
	while (getline(infile, s))
	{
		if (s.empty() || s == "\n")
		{
			node_flag = false;
			edge_flag = false;
			continue;
		}

		if (node_flag)
		{
			node_info = split(s, " ");
			Node node(node_info[0], number_of_nodes++, node_info[1], std::stof(node_info[2]), std::stof(node_info[3]), std::stof(node_info[4]), std::stof(node_info[5]));
			node_vector.push_back(node);

			if (node.node_type == "d")
			{
				start_depot_node_no = node.node_no;

				Node node_dummy = node;
				node_dummy.node_no = number_of_nodes++;
				end_depot_node_no = node_dummy.node_no;
				node_vector.push_back(node_dummy);
			}
			else if (node.node_type == "w")
			{
				F_vector.push_back(node.node_no);
			}
			else if (node.node_type == "c")
			{
				C_vector.push_back(node.node_no);
			}
		}
		else if (edge_flag)
		{
			edge_info = split(s, " ");
			travel_time_dict[edge_info[0]][edge_info[1]] = std::stof(edge_info[2]);
			intermediate_links_dict[edge_info[0]][edge_info[1]] = std::stoi(edge_info[3]);
		}
		else if (s.rfind("C Vehicle water tank capacity", 0) == 0)
		{
			std::vector<std::string> capacity_vec = split(s, "/");
			vehicle_water_capacity = std::stof(capacity_vec[1]);
		}
		else if (s.rfind("U Vehicle acquisition cost", 0) == 0)
		{
			std::vector<std::string> vehicle_acquisition_cost_vec = split(s, "/");
			vehicle_acquisition_cost = std::stof(vehicle_acquisition_cost_vec[1]);
		}
		else if (s.rfind("StringID", 0) == 0)
		{
			node_flag = true;
		}
		else if (s.rfind("From", 0) == 0)
		{
			edge_flag = true;
		}
	}
	infile.close();

	number_customers = C_vector.size();
	for (int i = 0; i < number_customers; i++)
		customer_index_dict[C_vector[i]] = i;
}


void Instance::initialization()
{
	// create node sets
	F0_vector = { start_depot_node_no };
	for (int i : F_vector)
		F0_vector.push_back(i);

	C0_vector = { start_depot_node_no };
	for (int i : C_vector)
		C0_vector.push_back(i);

	N_vector = C_vector;
	for (int i : F_vector)
		N_vector.push_back(i);

	N0_vector = { start_depot_node_no };
	for (int i : N_vector)
		N0_vector.push_back(i);

	Nn1_vector = N_vector;
	Nn1_vector.push_back(end_depot_node_no);

	N0n1_vector = N0_vector;
	N0n1_vector.push_back(end_depot_node_no);

	// create dynamic array for quick accessing
	travel_time_array = AllocateDynamicArray <float>(number_of_nodes, number_of_nodes);
	intermediate_links_array = AllocateDynamicArray <int>(number_of_nodes, number_of_nodes);
	cost_array = AllocateDynamicArray <float>(number_of_nodes, number_of_nodes);

	std::map<std::string, std::map<std::string, float>>::iterator iter1;
	std::map<std::string, float>::iterator iter2;
	Node *from_node, *to_node;
	bool o_exist, d_exist;


	for (int i = 0; i < number_of_nodes; i++)
	{
		for (int j = 0; j < number_of_nodes; j++)
		{
			travel_time_array[i][j] = very_big;
			intermediate_links_array[i][j] = very_big;
		}
	}

	for (int i = 0; i < number_of_nodes; i++)
	{
		if (i == end_depot_node_no)
			continue;

		from_node = &node_vector[i];
		iter1 = travel_time_dict.find(from_node->node_id);
		o_exist = iter1 != travel_time_dict.end() ? true : false;

		if (!o_exist)
			continue;

		for (int j = 0; j < number_of_nodes; j++)
		{
			if (j == start_depot_node_no)
				continue;

			to_node = &node_vector[j];
			iter2 = travel_time_dict[from_node->node_id].find(to_node->node_id);
			d_exist = iter2 != travel_time_dict[from_node->node_id].end() ? true : false;

			if (d_exist)
			{
				travel_time_array[i][j] = travel_time_dict[from_node->node_id][to_node->node_id];
				intermediate_links_array[i][j] = intermediate_links_dict[from_node->node_id][to_node->node_id];
			}

		}
	}

	node_type_array = new std::string[number_of_nodes];
	customer_node_flag_array = new bool[number_of_nodes];
	water_node_flag_array = new bool[number_of_nodes];
	service_time_array = new float[number_of_nodes];
	ready_time_array = new float[number_of_nodes];
	due_time_array = new float[number_of_nodes];
	water_consumption_array = new float[number_of_nodes];

	for (int i = 0; i < number_of_nodes; i++)
	{
		node_type_array[i] = node_vector[i].node_type;
		if (node_vector[i].node_type == "c")
			customer_node_flag_array[i] = true;
		else
			customer_node_flag_array[i] = false;

		if (node_vector[i].node_type == "w")
			water_node_flag_array[i] = true;
		else
			water_node_flag_array[i] = false;

		service_time_array[i] = node_vector[i].service_time;
		ready_time_array[i] = node_vector[i].ready_time;
		due_time_array[i] = node_vector[i].due_time;
		water_consumption_array[i] = node_vector[i].demand;
	}

	for (int i = 0; i < number_of_nodes; i++)
	{
		for (int j = 0; j < number_of_nodes; j++)
		{
			//if (customer_node_flag_array[i] && customer_node_flag_array[j])
			//{
				if (intermediate_links_array[i][j] > max_c2c_interlinks)
					max_c2c_interlinks = intermediate_links_array[i][j];
			//}
		}
	}

}
