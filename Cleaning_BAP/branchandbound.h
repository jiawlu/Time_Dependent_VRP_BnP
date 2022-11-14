#pragma once

#include <set>
#include <thread>
#include <mutex>
#include <list>
#include "paraVRP.h"
#include "vehpath.h"


typedef struct babnode
{
	int solution_status;
	//float obj;
	float ub = 1e10;
	float lb;

	float **travel_time_array;
	float **cost_array;
	std::vector<Path*> path_vector;
	std::set<Path*> new_path_set;
	float *path_uses;
	
	std::list<int> used_paths;

	//float *ub_soltion;

	struct babnode *left_node;
	struct babnode *right_node;


}BABNode;


class BABTree
{
public:
	BABTree(Instance *instance_)
	{
		instance = instance_;

		upper_bound = instance->very_big;
		lower_bound = 0.0;

		best_node = NULL;

		time_limit_reached = false;
		bf_finished = false;
		df_finished = false;

		//env = new GRBEnv();
		//env->set("OutputFlag", "0");
	}
	Instance *instance;

	float upper_bound;
	float lower_bound;

	BABNode* best_node;

	int time_start;
	//int time_limit;
	int max_time;
	bool time_limit_reached;
	bool bf_finished;
	bool df_finished;

	std::set<std::string> path_key_set_BF;
	std::set<Path*> path_set_BF;

	std::set<Path*> path_set_DF;

	std::mutex mtx;
	std::mutex mtxlb;

	//GRBEnv* env;

	BABNode* createRootNode(std::set<Path*> &initial_paths);
	void conductBranch(BABNode *current_node, const int from_node_no, const int to_node_no);
	void startBranchAndBound(int time_start);
	void solveNode(BABNode *current_node, const bool &bf_branch);

	void startBranchAndBoundBF();
	void startBranchAndBoundDF();

	void outputSolution();
};



