#pragma once
#include <gurobi_c++.h>
#include "Instance.hpp"

struct CarpInstance{
	CarpInstance();
	std::vector<int> link_to_visit;
	int nVehicle;

	explicit CarpInstance(int v_nVehicle);
};

struct Preprocessing {

	Preprocessing();
	std::map<int, CarpInstance> carpMap;
	void run(Instance const& inst);
};