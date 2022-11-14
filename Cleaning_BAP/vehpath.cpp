#include "vehpath.h"

void Path::initialization(Instance * instance)
{
	for (int node_no : node_no_sequence)
	{
		if (instance->customer_node_flag_array[node_no])
			customer_node_no_sequence.push_back(node_no);

		path_key += instance->node_vector[node_no].node_id;
	}

	cost = instance->vehicle_acquisition_cost;
	int from_node_no, to_node_no;
	from_node_no = node_no_sequence[0];
	for (int i = 0; i < node_no_sequence.size() - 1; i++)
	{
		to_node_no = node_no_sequence[i + 1];
		cost += instance->travel_time_array[from_node_no][to_node_no];
		//if (instance->water_node_flag_array[to_node_no])
			cost += instance->service_time_array[to_node_no];
		from_node_no = to_node_no;
	}


}


