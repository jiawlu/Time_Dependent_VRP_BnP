#pragma once
#include <vector>
#include <queue>
#include "paraVRP.h"
#include "vehpath.h"

typedef struct label
{
	int node_no;
	struct label *previous_label;
	float cost;
	float arrival_time;
	float water_level;
	bool dominated;
	bool *vertexVisited;			// size: number_of_nodes, keep water station always as false
	bool processed;

}Label;


class DPSolver
{
public:
	DPSolver(Instance *instance_, float **cost_array_)
	{
		instance = instance_;
		cost_array = cost_array_;
	}


	Instance *instance;
	float **cost_array;


	int checkDominance(const Label *label_i, const Label *label_j);


	void printCostMatrix();

	void startDynamicProgramming(std::vector<Path*> &new_path_vector, const int max_new_columns, const int intlink_limit);


};