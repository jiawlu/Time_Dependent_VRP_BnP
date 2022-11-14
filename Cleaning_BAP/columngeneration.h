#pragma once
#include "vehpath.h"
#include "branchandbound.h"


class ColumnGenSolver
{
public:
	ColumnGenSolver(Instance *instance_, int time_start_)
	{
		instance = instance_;
		time_start = time_start_;
		reduced_graph = true;
		intlink_limit = ((instance->max_c2c_interlinks / 2) > 1) ? (instance->max_c2c_interlinks / 2) : 1;
	}

	Instance *instance;
	int time_start;

	bool reduced_graph;
	int intlink_limit;

	//void generateInitialPaths(std::vector<Path*> &path_vector);
	void generateNewColumn(BABNode* current_babnode, double* pis);
	void startColumnGen(BABNode* current_babnode);
};