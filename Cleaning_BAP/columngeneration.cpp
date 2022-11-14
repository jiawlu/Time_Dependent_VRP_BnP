#include "columngeneration.h"
#include "dynamicprogramming.h"
#include "utils.h"
#include <algorithm>
#include <ilcplex/ilocplex.h>



void ColumnGenSolver::generateNewColumn(BABNode* current_babnode, double* pis)
{
	bool updating_customer;
	int customer_idx;
	float dual_pi;

	for (int j = 0; j < instance->number_of_nodes; j++)
	{
		if (instance->customer_node_flag_array[j])
		{
			updating_customer = true;
			customer_idx = instance->customer_index_dict[j];
			dual_pi = pis[customer_idx];
		}
		else
		{
			updating_customer = false;
		}
			
		for (int i = 0; i < instance->number_of_nodes; i++)
		{
			current_babnode->cost_array[i][j] = current_babnode->travel_time_array[i][j];

			if (current_babnode->cost_array[i][j] >= instance->very_big - 1e-6)
				continue;

			if (updating_customer)
				current_babnode->cost_array[i][j] -= dual_pi;
		}
	}

	std::vector<Path*> new_path_vector;
	DPSolver dpsolver(instance, current_babnode->cost_array);


	// dpsolver.startDynamicProgramming(new_path_vector, instance->number_customers, 2);

	if (reduced_graph)
	{
		dpsolver.startDynamicProgramming(new_path_vector, instance->number_customers, intlink_limit);		//instance->number_customers*2
		//std::cout << "intlink limit: " << intlink_limit << std::endl;

		if (new_path_vector.size() == 0)
		{
			dpsolver.startDynamicProgramming(new_path_vector, instance->number_customers, instance->max_c2c_interlinks);		//instance->number_customers*2
			//std::cout << "intlink limit: " << instance->max_c2c_interlinks << std::endl;
			reduced_graph = false;
		}
	}
	else
	{
		dpsolver.startDynamicProgramming(new_path_vector, instance->number_customers, instance->max_c2c_interlinks);		//instance->number_customers*2
		//std::cout << "intlink limit: " << instance->max_c2c_interlinks << std::endl;
	}


	for (Path *la : new_path_vector)
	{
		current_babnode->path_vector.push_back(la);
		current_babnode->new_path_set.insert(la);
	}
		
}



void ColumnGenSolver::startColumnGen(BABNode * current_babnode)
{
	if (current_babnode->path_vector.empty())
	{
		std::cout << "no routes found in the current node\n";
		ProgramStop();
	}
		
	/* BUILD INITIAL LP MODEL */
	int number_customers = instance->number_customers;
	int number_of_paths = current_babnode->path_vector.size();

	IloInt  ii;
	IloEnv env;
	IloModel cutOpt(env);

	IloNumArray amount(env, number_customers);
	for (ii = 0; ii < number_customers; ii++)
		amount[ii] = 1;

	IloObjective   RollsUsed = IloAdd(cutOpt, IloMinimize(env));
	IloRangeArray  Fill = IloAdd(cutOpt, IloRangeArray(env, amount, amount));
	IloNumVarArray Cut(env);

	IloNumArray newPatt(env, number_customers);
	for (Path *path : current_babnode->path_vector)
	{
		for (ii = 0; ii < number_customers; ii++)
			newPatt[ii] = 0;
		for (int cnn : path->customer_node_no_sequence)
			newPatt[instance->customer_index_dict[cnn]] = 1;
		Cut.add(IloNumVar(RollsUsed(path->cost) + Fill(newPatt)));
	}

	//for (ii = 0; ii < number_of_paths; ii++)
	//	Cut.add(IloNumVar(RollsUsed(current_babnode->path_vector[ii]->cost) + Fill[ii](1)));

	IloCplex cutSolver(cutOpt);
	cutSolver.setOut(env.getNullStream());
	cutSolver.setParam(IloCplex::Param::Threads, 1);

	/* COLUMN GENERATION */
	int cg_iter = 0;
	Path *path;
	double *pis = new double[number_customers];
	float obj;
	

	while (true)
	{
		cutSolver.solve();

		//obj = cutSolver.getObjValue();
		//std::cout << "obj: " << cutSolver.getObjValue() << std::endl;

		for (ii = 0; ii < number_customers; ii++)
			pis[ii] = cutSolver.getDual(Fill[ii]);

		generateNewColumn(current_babnode, pis);

		if (current_babnode->path_vector.size() > number_of_paths)
		{
			// new columns found, add them to the LP model
			for (int i = number_of_paths; i < current_babnode->path_vector.size(); i++)
			{
				path = current_babnode->path_vector[i];
				for (ii = 0; ii < number_customers; ii++)
					newPatt[ii] = 0;
				for (int cnn : path->customer_node_no_sequence)
					newPatt[instance->customer_index_dict[cnn]] = 1;
				Cut.add(IloNumVar(RollsUsed(path->cost) + Fill(newPatt)));
			}

			number_of_paths = current_babnode->path_vector.size();
		}
		else
		{
			// no new column
			break;
		}

		cg_iter++;
	}

	obj = cutSolver.getObjValue();

	//std::cout << "cg_finished! Obj: " << obj << std::endl;

	/* GET PATH USE */
	bool integer = true;
	float path_use;
	float *path_uses = new float[number_of_paths];
	std::list<int> used_paths;

	for (int i = 0; i < number_of_paths; i++)
	{
		path_use = cutSolver.getValue(Cut[i]);
		path_uses[i] = path_use;
		if ((path_use > 1e-6) && (path_use < 1 - 1e-6))
			integer = false;
		if (integer && (path_use > 1 - 1e-6))
			used_paths.push_back(i);
	}

	current_babnode->solution_status = integer ? 1 : 0;

	if (integer)
	{
		current_babnode->ub = obj;
		current_babnode->used_paths = used_paths;
		delete []path_uses;
	}
	else
	{
		current_babnode->lb = obj;
		current_babnode->path_uses = path_uses;

		if (instance->ub_strategy == 1)			// 0-no, 1-heuristic, 2-mip
		{
			/* calculate ub using heuristic */
			std::vector<float> reduced_costs;
			std::vector<int> path_nos;
			float reduced_cost;
			int node_no;
			for (int i = 0; i < current_babnode->path_vector.size(); i++)
			{
				path_nos.push_back(i);
				path = current_babnode->path_vector[i];
				reduced_cost = path->cost;
				for (int j = 1; j < path->node_no_sequence.size() - 1; j++)
				{
					node_no = path->node_no_sequence[j];
					if (instance->customer_node_flag_array[node_no])
						reduced_cost -= pis[instance->customer_index_dict[node_no]];
				}
				reduced_costs.push_back(reduced_cost);
			}
			std::sort(path_nos.begin(), path_nos.end(), [=](int i, int j) -> bool { return reduced_costs[i] < reduced_costs[j]; });
			int number_of_served_cusotmers = 0;
			bool *customer_status = new bool[instance->number_customers];
			for (int i = 0; i < instance->number_customers; i++)
				customer_status[i] = false;
			bool path_valid;
			//std::vector<Path*> used_paths;
			for (int path_no : path_nos)
			{
				path = current_babnode->path_vector[path_no];
				path_valid = true;
				for (int c_no : path->customer_node_no_sequence)
				{
					if (customer_status[instance->customer_index_dict[c_no]])
					{
						path_valid = false;
						break;
					}
				}
				if (path_valid)
				{
					for (int c_no : path->customer_node_no_sequence)
					{
						customer_status[instance->customer_index_dict[c_no]] = true;
						number_of_served_cusotmers++;
					}
					current_babnode->used_paths.push_back(path_no);
					//used_paths.push_back(path);
				}

				if (number_of_served_cusotmers == instance->number_customers)
					break;
			}

			float total_cost = 0.0;
			for (std::list<int>::iterator iter = current_babnode->used_paths.begin(); iter != current_babnode->used_paths.end(); iter++)
				total_cost += current_babnode->path_vector[*iter]->cost;

			current_babnode->ub = total_cost;
			delete[] customer_status;

		}
		else if (instance->ub_strategy == 2)
		{
			/* calculate ub using mip */
			cutOpt.add(IloConversion(env, Cut, ILOINT));
			cutSolver.setParam(IloCplex::Param::TimeLimit, 5);
			cutSolver.solve();

			IloAlgorithm status;
			IloAlgorithm::Status mip_status = cutSolver.getStatus();
			//std::cout << "Solution status: " << mip_status << std::endl;

			if (mip_status == status.Feasible || mip_status == status.Optimal)
			{
				current_babnode->ub = cutSolver.getObjValue();
				for (int i = 0; i < number_of_paths; i++)
				{
					if (cutSolver.getValue(Cut[i]) > 1 - 1e-6)
						current_babnode->used_paths.push_back(i);
				}
			}
		}
	}

	env.end();
	delete[] pis;	
}
