#pragma once
#include <set>
#include "paraVRP.h"



class Path
{
public:
	Path(std::vector<int> node_no_sequence_, bool initial_path_ = false)
	{
		node_no_sequence = node_no_sequence_;
		initial_path = initial_path_;
		path_key = "";
	}

	bool initial_path;
	float cost;
	std::string path_key;

	std::vector<int> node_no_sequence;
	std::vector<int> customer_node_no_sequence;

	void initialization(Instance *instance);

};