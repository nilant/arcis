#pragma once

#include "Instance.hpp"
#include "local_search.hpp"
#include <set>

struct Preprocessing {

	explicit Preprocessing(Instance const& inst);
	std::map<int, std::set<int>> carpMap;
	std::vector<std::set<int>> contains;
	std::set<int> elementarySP;
	void run(Instance const& inst, RandomGenerator& rand_gen);
};