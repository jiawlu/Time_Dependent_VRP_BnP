#include "dynamicprogramming.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <list>
#include <algorithm>



int DPSolver::checkDominance(const Label *label_i, const Label *label_j)
{
	// current_label is dominated
	bool nodedom = true;
	for (int i = 0; i < instance->number_of_nodes; i++)
	{
		if (label_i->vertexVisited[i] && !label_j->vertexVisited[i])
		{
			nodedom = false;
			break;
		}
	}
	if (nodedom && label_i->cost <= label_j->cost && label_i->arrival_time <= label_j->arrival_time && label_i->water_level >= label_j->water_level)
		return -1;


	// existing_label is dominated
	nodedom = true;
	for (int i = 0; i < instance->number_of_nodes; i++)
	{
		if (label_j->vertexVisited[i] && !label_i->vertexVisited[i])
		{
			nodedom = false;
			break;
		}
	}
	if (nodedom && label_j->cost <= label_i->cost && label_j->arrival_time <= label_i->arrival_time && label_j->water_level >= label_i->water_level)
		return 1;

	return 0;
}


struct tmp2
{
	bool operator() (Label *la, Label *lb)
	{
		return la->cost > lb->cost;
	}
};



void DPSolver::printCostMatrix()
{

	std::ofstream myout("cost.txt");

	for (int i = 0; i < instance->number_of_nodes; i++)
	{
		for (int j = 0; j < instance->number_of_nodes; j++)
		{
			myout << std::setw(12) << std::left << instance->travel_time_array[i][j];
		}

		myout << std::endl;
	}

	myout << std::endl;


	for (int i = 0; i < instance->number_of_nodes; i++)
	{
		for (int j = 0; j < instance->number_of_nodes; j++)
		{
			myout << std::setw(12) << std::left << cost_array[i][j];
		}

		myout << std::endl;
	}

	myout.close();
}


void DPSolver::startDynamicProgramming(std::vector<Path*>& new_path_vector, const int max_new_columns, const int intlink_limit)
{
	//printCostMatrix();

	std::cout << "h\n";

	std::priority_queue<Label*, std::vector<Label*>, tmp2> unprocessed_label_queue;
	std::set<Label*> processed_label_set;
	std::list<Label*> all_labels;

	std::map<int, std::set<Label*>> node_labels_map;
	for (int i = 0; i < instance->number_of_nodes; i++)
	{
		std::set<Label*> node_labels;
		node_labels_map[i] = node_labels;
	}

	bool *visited_nodes = new bool[instance->number_of_nodes];
	visited_nodes[0] = true;
	for (int i = 1; i < instance->number_of_nodes; i++)
		visited_nodes[i] = false;

	Label *first_label = new Label{ instance->start_depot_node_no, NULL, instance->vehicle_acquisition_cost, 0.0, instance->vehicle_water_capacity, false, visited_nodes, false };
	unprocessed_label_queue.push(first_label);
	node_labels_map[instance->start_depot_node_no].insert(first_label);
	all_labels.push_back(first_label);

	
	Label *current_label, *new_label;
	int current_node_no;
	float arrival_time, water_level, cost, arrival_time_n, arrival_time_c1, arrival_time_c2;
	int dom_flag;
	int end_depot_node_no = instance->end_depot_node_no;
	float min_c_cost, min_path_cost;

	while (!unprocessed_label_queue.empty() && processed_label_set.size() < max_new_columns)
	{
		current_label = unprocessed_label_queue.top();
		unprocessed_label_queue.pop();

		if (current_label->dominated)
		{	
			// delete current_label;
			continue;
		}

		current_node_no = current_label->node_no;

		// node_labels_map[current_node_no].erase(current_label);			// should we remove the current label from node_label_map?
		// node_labels_map[current_node_no].insert(current_label);

		if (current_node_no == end_depot_node_no)
		{
			if (current_label->cost < -1e-3)
			{
				for (std::set<Label*>::iterator iter = processed_label_set.begin(); iter != processed_label_set.end();)
				{
					dom_flag = checkDominance(*iter, current_label);

					if (dom_flag == 1)
					{
						// existing_label is dominated
						// (*iter)->processed = false;
						node_labels_map[end_depot_node_no].erase(*iter);
						processed_label_set.erase(iter++);
						// delete la;
					}
					else if (dom_flag == -1)
					{
						// current_label is dominated
						current_label->dominated = true;
						break;
					}
					else if (dom_flag == 0)
					{
						iter++;
					}

				}

				if (current_label->dominated)
				{
					node_labels_map[end_depot_node_no].erase(current_label);
					// delete current_label;
				}
				else
				{
					current_label->processed = true;
					processed_label_set.insert(current_label);
				}
			}

		}
		else
		{
			for (int next_node_no : instance->Nn1_vector)
			{
				// to be update: consider water station
				if (instance->intermediate_links_array[current_node_no][next_node_no] > intlink_limit)
					continue;

				if (current_label->vertexVisited[next_node_no])
					continue;
				if (cost_array[current_node_no][next_node_no] > instance->very_big - 1e-6)
					continue;
				if ((current_label->water_level >= instance->vehicle_water_capacity - 1e-6) && instance->water_node_flag_array[next_node_no])
					continue;
				if (instance->water_node_flag_array[current_node_no] && next_node_no == end_depot_node_no)
					continue;

				arrival_time = current_label->arrival_time + instance->service_time_array[current_node_no] + instance->travel_time_array[current_node_no][next_node_no];
				if (arrival_time > instance->due_time_array[next_node_no])
					continue;
				if (arrival_time < instance->ready_time_array[next_node_no])
					arrival_time = instance->ready_time_array[next_node_no];

				if (next_node_no != end_depot_node_no)
				{
					if (arrival_time + instance->service_time_array[next_node_no] + instance->travel_time_array[next_node_no][end_depot_node_no] > instance->due_time_array[end_depot_node_no])
						continue;
				}

				if (instance->water_node_flag_array[next_node_no])
					water_level = instance->vehicle_water_capacity;
				else
					water_level = current_label->water_level - instance->water_consumption_array[next_node_no];
				if (water_level < 0)
					continue;

				cost = current_label->cost + cost_array[current_node_no][next_node_no];
				//if (instance->water_node_flag_array[next_node_no])
					cost += instance->service_time_array[next_node_no];

				if (next_node_no == end_depot_node_no && cost > 0)
					continue;
				
				visited_nodes = new bool[instance->number_of_nodes];
				for (int i = 0; i < instance->number_of_nodes; i++)
					visited_nodes[i] = current_label->vertexVisited[i];
				if (!instance->water_node_flag_array[next_node_no])
					visited_nodes[next_node_no] = true;

				if (next_node_no != end_depot_node_no)
				{
					for (int nn_node_no : instance->Nn1_vector)
					{
						if (visited_nodes[nn_node_no])
							continue;
						arrival_time_n = arrival_time + instance->service_time_array[next_node_no] + instance->travel_time_array[next_node_no][nn_node_no];

						if (arrival_time_n > instance->due_time_array[nn_node_no])
							visited_nodes[nn_node_no] = true;

						if (!visited_nodes[nn_node_no] && nn_node_no != end_depot_node_no)
						{
							if (arrival_time_n + instance->service_time_array[nn_node_no] + instance->travel_time_array[nn_node_no][end_depot_node_no] > instance->due_time_array[end_depot_node_no])
								visited_nodes[nn_node_no] = true;
						}
					}

					if (cost > 0)
					{
						min_path_cost = cost;
						for (int nn_node_no : instance->C_vector)
						{
							if (visited_nodes[nn_node_no])
								continue;

							min_c_cost = 1.0;
							for (int i = 0; i < instance->number_of_nodes; i++)
							{
								if (visited_nodes[i])		//add
									continue;

								arrival_time_c1 = arrival_time + instance->service_time_array[next_node_no] + instance->travel_time_array[next_node_no][i];
								if (arrival_time_c1 < instance->ready_time_array[i])
									arrival_time_c1 = instance->ready_time_array[i];
								arrival_time_c2 = arrival_time_c1 + instance->service_time_array[i] + instance->travel_time_array[i][nn_node_no];
								if (arrival_time_c2 > instance->due_time_array[nn_node_no])
									continue;

								if (cost_array[i][nn_node_no] < min_c_cost)		// cost_array[nn_node_no][i]
									min_c_cost = cost_array[i][nn_node_no];		// cost_array[nn_node_no][i]
							}

							min_c_cost += instance->service_time_array[nn_node_no];		//add

							if (min_c_cost < 0)
								min_path_cost += min_c_cost;
						}

						if (min_path_cost > 0)
						{
							delete []visited_nodes;
							continue;
						}
							
					}

				}

				new_label = new Label{ next_node_no, current_label, cost, arrival_time, water_level, false, visited_nodes, false };
				all_labels.push_back(new_label);
	
				for (std::set<Label*>::iterator iter = node_labels_map[next_node_no].begin(); iter != node_labels_map[next_node_no].end();)
				{
					dom_flag = checkDominance(*iter, new_label);

					if (dom_flag == 1)
					{
						// existing_label is dominated
						(*iter)->dominated = true;

						if ((*iter)->processed)
							processed_label_set.erase(*iter);

						node_labels_map[next_node_no].erase(iter++);
					}
					else if (dom_flag == -1)
					{
						// current_label is dominated
						new_label->dominated = true;
						break;
					}
					else if (dom_flag == 0)
					{
						iter++;
					}
				}

				if (!new_label->dominated)
				{
					node_labels_map[next_node_no].insert(new_label);
					unprocessed_label_queue.push(new_label);
				}

			}		// for next node, expand the current label
		}		// current label
	}		// outer while loop

	Label *pre_la;

	for (Label *la : processed_label_set)
	{
		//std::cout << " " << la->cost;

		if (la->dominated)
		{
			std::cout << "warning: dominated label detected in the processed label set\n";
			continue;
		}
		
		if (la->cost >= 0)
		{
			std::cout << "warning: label with positive cost detected in the processed label set\n";
			continue;
		}

		std::vector<int> node_no_sequence = { la->node_no };
		pre_la = la->previous_label;
		while (pre_la != NULL)
		{
			node_no_sequence.push_back(pre_la->node_no);
			pre_la = pre_la->previous_label;
		}
		std::reverse(node_no_sequence.begin(), node_no_sequence.end());

		Path *path = new Path(node_no_sequence, false);
		path->initialization(instance);
		new_path_vector.push_back(path);
	}


	// delete lables: processed, unprocessed, visited_nodes in labels
	for (Label* la : all_labels)
	{
		delete[] (la->vertexVisited);
		delete la;
	}

}
