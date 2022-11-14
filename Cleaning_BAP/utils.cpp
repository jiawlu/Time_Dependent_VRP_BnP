#include "utils.h"

void ProgramStop()
{
	getchar();
	exit(0);
}

std::vector<std::string> split(const std::string &str, const std::string &pattern)
{
	std::vector<std::string> res;
	if (str == "")
		return res;

	std::string strs = str + pattern;
	size_t pos = strs.find(pattern);

	while (pos != strs.npos)
	{
		std::string temp = strs.substr(0, pos);
		if (!temp.empty())
			res.push_back(temp);
		strs = strs.substr(pos + 1, strs.size());
		pos = strs.find(pattern);
	}

	return res;
}
