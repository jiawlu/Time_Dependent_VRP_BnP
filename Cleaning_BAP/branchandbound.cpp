#include "branchandbound.h"
#include "paraVRP.h"
#include "vehpath.h"
#include "columngeneration.h"
#include "utils.h"
#include <iostream>
#include <cmath>
// #include <omp.h>
#include <algorithm>




BABNode * BABTree::createRootNode(std::set<Path*> &initial_paths)
{
	BABNode *root_node = new BABNode;
	root_node->solution_status = -1;
	root_node->travel_time_array = AllocateDynamicArray <float>(instance->number_of_nodes, instance->number_of_nodes);
	root_node->cost_array = AllocateDynamicArray <float>(instance->number_of_nodes, instance->number_of_nodes);
	for (int i = 0; i < instance->number_of_nodes; i++)
	{
		for (int j = 0; j < instance->number_of_nodes; j++)
		{
			root_node->travel_time_array[i][j] = instance->travel_time_array[i][j];
		}
	}

	float customer_arrival_time, depot_arrival_time;
	int start_depot_node_no = instance->start_depot_node_no;
	int end_depot_node_no = instance->end_depot_node_no;
	float depot_due_time = instance->due_time_array[end_depot_node_no];
	float total_cost = 0.0;

	for (int customer_node_no : instance->C_vector)
	{
		if (instance->water_consumption_array[customer_node_no] > instance->vehicle_water_capacity)
		{
			std::cout << "Problem is infeasible. Customer " << instance->node_vector[customer_node_no].node_id << " cannot be served (water).\n";
			ProgramStop();
		}

		customer_arrival_time = std::max(instance->travel_time_array[start_depot_node_no][customer_node_no], instance->ready_time_array[customer_node_no]);
		depot_arrival_time = customer_arrival_time + instance->service_time_array[customer_node_no] + instance->travel_time_array[customer_node_no][end_depot_node_no];

		if (depot_arrival_time > depot_due_time)
		{
			std::cout << "Problem is infeasible. Customer " << instance->node_vector[customer_node_no].node_id << " cannot be served (time).\n";
			ProgramStop();
		}


		//cost = travel_time + instance->vehicle_acquisition_cost;
		std::vector<int> node_no_sequence = { start_depot_node_no, customer_node_no , end_depot_node_no };

		Path *path = new Path(node_no_sequence, true);
		path->initialization(instance);
		total_cost += path->cost;

		initial_paths.insert(path);
	}

	root_node->ub = total_cost;

	return root_node;
}



void BABTree::conductBranch(BABNode *current_node, const int from_node_no, const int to_node_no)
{
	BABNode *left_node = new BABNode;
	BABNode *right_node = new BABNode;

	left_node->solution_status = -1;
	right_node->solution_status = -1;

	left_node->travel_time_array = AllocateDynamicArray <float>(instance->number_of_nodes, instance->number_of_nodes);
	left_node->cost_array = AllocateDynamicArray <float>(instance->number_of_nodes, instance->number_of_nodes);

	right_node->travel_time_array = AllocateDynamicArray <float>(instance->number_of_nodes, instance->number_of_nodes);
	right_node->cost_array = AllocateDynamicArray <float>(instance->number_of_nodes, instance->number_of_nodes);

	for (int i = 0; i < instance->number_of_nodes; i++)
	{
		for (int j = 0; j < instance->number_of_nodes; j++)
		{
			left_node->travel_time_array[i][j] = current_node->travel_time_array[i][j];
			right_node->travel_time_array[i][j] = current_node->travel_time_array[i][j];
		}
	}

	// left node, do not allow
	left_node->travel_time_array[from_node_no][to_node_no] = instance->very_big;

	// right node, must pass
	if (from_node_no != instance->start_depot_node_no)
	{
		for (int node_no : instance->N0n1_vector)
		{
			if (node_no != to_node_no)
				right_node->travel_time_array[from_node_no][node_no] = instance->very_big;
		}
	}
	if (to_node_no != instance->end_depot_node_no)
	{
		for (int node_no : instance->N0n1_vector)
		{
			if (node_no != from_node_no)
				right_node->travel_time_array[node_no][to_node_no] = instance->very_big;
		}
	}
	right_node->travel_time_array[to_node_no][from_node_no] = instance->very_big;

	current_node->left_node = left_node;
	current_node->right_node = right_node;
}


void BABTree::startBranchAndBound(int time_start_)
{
	time_start = time_start_;
	max_time = time_start_ + instance->time_limit;

	//startBranchAndBoundBF();
	//startBranchAndBoundDF();

	std::thread bdf(&BABTree::startBranchAndBoundBF, this);
	std::thread tdf(&BABTree::startBranchAndBoundDF, this);

	bdf.join();
	tdf.join();

	outputSolution();
}



void BABTree::solveNode(BABNode * current_node, const bool &bf_branch)
{
	// get valid path
	bool accept_path;
	int fn, tn;

	const std::set<Path*> *path_set_ = bf_branch ? (&path_set_BF) : (&path_set_DF);

	for (Path *path : *path_set_)
	{
		if (path->initial_path)
		{
			current_node->path_vector.push_back(path);
		}
		else
		{
			accept_path = true;
			fn = path->node_no_sequence[0];
			for (int i = 1; i < path->node_no_sequence.size(); i++)
			{
				tn = path->node_no_sequence[i];
				if (current_node->travel_time_array[fn][tn] > (instance->very_big - 1e-6))
				{
					accept_path = false;
					break;
				}
				fn = tn;
			}

			if (accept_path)
				current_node->path_vector.push_back(path);
		}
	}

	int bestEdge1, bestEdge2;
	float dev, dev_tmp, coeff;

	ColumnGenSolver cgsolver(instance, time_start);
	cgsolver.startColumnGen(current_node);

	if (current_node->solution_status == 0 && current_node->lb < upper_bound - 1e-6)
	{
		float **edge_use = AllocateDynamicArray <float>(instance->number_of_nodes, instance->number_of_nodes);
		for (int i = 0; i < instance->number_of_nodes; i++)
			for (int j = 0; j < instance->number_of_nodes; j++)
				edge_use[i][j] = 0.0;

		Path *path;
		float current_path_use;
		for (int i = 0; i < current_node->path_vector.size(); i++)
		{
			path = current_node->path_vector[i];
			current_path_use = current_node->path_uses[i];
			for (int j = 0; j < path->node_no_sequence.size() - 1; j++)
				edge_use[path->node_no_sequence[j]][path->node_no_sequence[j + 1]] += current_path_use;
		}
		delete[](current_node->path_uses);

		dev = 100.0;
		bestEdge1 = -1;
		bestEdge2 = -1;

		for (int i = 0; i < instance->number_of_nodes; i++)
		{
			for (int j = 0; j < instance->number_of_nodes; j++)
			{
				if (i == instance->start_depot_node_no || j == instance->end_depot_node_no)
					continue;

				coeff = edge_use[i][j];
				if ((coeff > 1e-6) && ((coeff < 1 - 1e-6) || (coeff > 1 + 1e-6)))
				{
					dev_tmp = std::abs(0.5 - coeff);
					if (dev_tmp < dev)
					{
						dev = dev_tmp;
						bestEdge1 = i;
						bestEdge2 = j;
					}
				}
			}
		}

		conductBranch(current_node, bestEdge1, bestEdge2);

		DeallocateDynamicArray(edge_use, instance->number_of_nodes, instance->number_of_nodes);
	}

}


void BABTree::startBranchAndBoundBF()
{
	std::set<Path*> initial_paths;
	BABNode *root_node = createRootNode(initial_paths);

	for (Path *path : initial_paths)
	{
		path_set_BF.insert(path);
		path_key_set_BF.insert(path->path_key);
	}

	mtx.lock();
	upper_bound = root_node->ub;
	mtx.unlock();

	std::vector<BABNode*> current_layer_nodes, next_layer_nodes;
	next_layer_nodes.push_back(root_node);
	int layer_no = 0;
	float gap;

	// int num_of_procs = omp_get_num_procs();
	bool solving_root = true;

	while (next_layer_nodes.size() > 0)
	{
// #pragma omp parallel for num_threads(num_of_procs-1)
		for (int i = 0; i < current_layer_nodes.size(); i++)
		{
			DeallocateDynamicArray(current_layer_nodes[i]->travel_time_array, instance->number_of_nodes, instance->number_of_nodes);
			DeallocateDynamicArray(current_layer_nodes[i]->cost_array, instance->number_of_nodes, instance->number_of_nodes);
		}

		current_layer_nodes = next_layer_nodes;
		next_layer_nodes.clear();

		std::cout << "*BF  Solving layer " << layer_no << ", number of nodes " << current_layer_nodes.size() << std::endl;

// #pragma omp parallel for num_threads(num_of_procs-1) schedule(dynamic)
		for (int i = 0; i < current_layer_nodes.size(); i++)
		{
			if ((time(NULL) <= max_time) && (!df_finished))
				solveNode(current_layer_nodes[i], true);
		}

		if (df_finished)
			break;

		for (BABNode *current_node : current_layer_nodes)
		{
			mtx.lock();
			if (current_node->ub < upper_bound - 1e-6)
			{
				upper_bound = current_node->ub;
				best_node = current_node;
			}
			mtx.unlock();
		}


		if (time(NULL) <= max_time || solving_root)
		{
			lower_bound = upper_bound;
			for (BABNode *current_node : current_layer_nodes)
			{
				if (current_node->solution_status == 0)
				{
					if (current_node->lb < upper_bound - 1e-6)
					{
						next_layer_nodes.push_back(current_node->left_node);
						next_layer_nodes.push_back(current_node->right_node);

						mtxlb.lock();
						if (current_node->lb < lower_bound - 1e-6)
							lower_bound = current_node->lb;
						mtxlb.unlock();
					}
				}
			}
		}
		else
		{
			time_limit_reached = true;
		}

		gap = (upper_bound - lower_bound) / upper_bound * 100.0;
		std::cout << "*BF    lb:" << lower_bound << ", ub:" << upper_bound << ", gap:" << gap << ", time:" << time(NULL) - time_start << std::endl;

		if (gap < 1e-6)
			break;

		if (time_limit_reached)
		{
			std::cout << "*BF  BF stopped as time limit has reached\n";
			break;
		}

		for (BABNode *current_node : current_layer_nodes)
		{
			for (Path* path : current_node->new_path_set)
			{
				if (path_key_set_BF.find(path->path_key) == path_key_set_BF.end())
				{
					path_set_BF.insert(path);
					path_key_set_BF.insert(path->path_key);
				}
			}

		}

		solving_root = false;
		layer_no++;
	}

	if (!(df_finished || time_limit_reached))
	{
		bf_finished = true;
		std::cout << "*BF  BF finished\n";
	}
		
}


void BABTree::startBranchAndBoundDF()
{
	std::set<Path*> initial_paths;
	BABNode *root_node = createRootNode(initial_paths);
	for (Path *path : initial_paths)
		path_set_DF.insert(path);

	mtx.lock();
	upper_bound = root_node->ub;
	mtx.unlock();

	std::vector<BABNode*> unprocessed_nodes;
	unprocessed_nodes.push_back(root_node);

	//clock_t current_time;
	BABNode *current_node;
	float gap;
	bool solving_root = true;

	while (true)
	{
		if (bf_finished)
		{
			std::cout << "-DF  DF stopped as BF has finished\n";
			break;
		}

		if (time(NULL) > max_time)
		{
			time_limit_reached = true;
			std::cout << "-DF  DF stopped as time limit has reached\n";
			break;
		}

		current_node = unprocessed_nodes.back();
		unprocessed_nodes.pop_back();

		solveNode(current_node, false);

		if (current_node->solution_status != -1)
		{
			mtx.lock();
			if (current_node->ub < upper_bound - 1e-6)
			{
				upper_bound = current_node->ub;
				best_node = current_node;
				gap = (upper_bound - lower_bound) / upper_bound * 100.0;
				std::cout << "-DF    lb:" << lower_bound << ", ub:" << upper_bound << ", gap:" << gap << ", time:" << time(NULL) - time_start << std::endl;

				if (gap < 1e-6)
					break;
			}
			mtx.unlock();

			if (current_node->solution_status == 0)
			{
				if (current_node->lb < upper_bound - 1e-6)
				{
					unprocessed_nodes.push_back(current_node->left_node);
					unprocessed_nodes.push_back(current_node->right_node);

					if (solving_root)
					{
						mtxlb.lock();
						if (current_node->lb > lower_bound + 1e-6)
						{
							lower_bound = current_node->lb;
							gap = (upper_bound - lower_bound) / upper_bound * 100.0;
							std::cout << "-DF    lb:" << lower_bound << ", ub:" << upper_bound << ", gap:" << gap << ", time:" << time(NULL) - time_start << std::endl;
						}
							
						mtxlb.unlock();
					}
				}

			}
		}

		if (unprocessed_nodes.size() > 0)
		{
			for (Path* path : current_node->new_path_set)
			{
				path_set_DF.insert(path);
			}
		}
		else
		{
			break;
		}

		solving_root = false;
	}

	if (!(bf_finished || time_limit_reached))
	{
		df_finished = true;
		std::cout << "-DF  DF finished\n";
	}
		
}



void BABTree::outputSolution()
{
	if (best_node != NULL)
	{
		std::cout << "\nSolution:\n";
		std::cout << "Objective: " << best_node->ub << std::endl;

		int path_no = 0;
		std::list<int>::iterator iter;
		Path *path;

		for (iter = best_node->used_paths.begin(); iter != best_node->used_paths.end(); iter++)
		{
			std::cout << "path " << path_no << ":";
			path = best_node->path_vector[*iter];
			for (int served_node_no : path->node_no_sequence)
				std::cout << " " << instance->node_vector[served_node_no].node_id;
			std::cout << std::endl;
			path_no++;
		}



		//for (int i = 0; i < best_node->path_vector.size(); i++)
		//{
		//	if (std::abs(best_node->path_uses[i] - 1) < 1e-6)
		//	{
		//		std::cout << "path " << path_no << ":";
		//		for (int served_node_no : best_node->path_vector[i]->node_no_sequence)
		//			std::cout << " " << instance->node_vector[served_node_no].node_id;
		//		std::cout << std::endl;
		//		path_no++;
		//	}
		//}
	}
	else
	{
		std::cout << "\nNo feasible solution found\n";
	}

}


