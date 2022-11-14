#include <iostream>
#include <ctime>
#include "paraVRP.h"
#include "branchandbound.h"
#include "utils.h"



int main(int argc, char *argv[])
{
	std::string instance_filename = "../Datasets/small_geo_B1.txt";
	int time_limit = 3600;
	int ub_strategy = 0;

	if (argc > 1)
		instance_filename = argv[1];
	if (argc > 2)
		time_limit = std::stoi(argv[2]);
	if (argc > 3)
		ub_strategy = std::stoi(argv[3]);

	std::cout << "INSTANCE: " << instance_filename << "\n";
	std::cout << "TIME LIMIT: " << time_limit << "\n";
	std::cout << "UB STRATEGY: " << ub_strategy << "\n\n";

	Instance instanceVRP(instance_filename, time_limit, ub_strategy);

	BABTree babTree(&instanceVRP);

	clock_t time_start_ms = clock();
	clock_t time_start = time(NULL);
	babTree.startBranchAndBound(time_start);
	clock_t time_end_ms = clock();
	clock_t time_end = time(NULL);

	std::cout << "Total time ms: " << (time_end_ms - time_start_ms) / 1000.0 << " seconds.\n\n";
	std::cout << "Total time: " << time_end - time_start << " seconds.\n\n";
}




