#pragma once
#include <gurobi_c++.h>
#include "Instance.hpp"
#include "local_search.hpp"
#include <set>

struct CarpInstance{
	CarpInstance();
	std::set<int> link_to_visit;
	int nVehicle;

	explicit CarpInstance(int v_nVehicle);
};

struct Preprocessing {

	Preprocessing(Instance const& inst);
	std::map<int, CarpInstance> carpMap;
	std::vector<std::set<int>> contains;
	std::set<int> elementarySP;
	void run(Instance const& inst, RandomGenerator& rand_gen);
};