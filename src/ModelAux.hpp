#pragma once
#include <gurobi_c++.h>

#include "Instance.hpp"
#include "mdarray.hpp"
#include "cli.hpp"


struct AuxResult {
	long empty_days{0};
	mdarray<int, 1> services_per_day;
	// indices are k, l(k = day, l = required link). 1 if l served in day k 
	mdarray<int, 2> x_val; 

	AuxResult(mdarray<GRBVar, 2> const& x, mdarray<GRBVar, 1> const& z);
	AuxResult(int n_required_link, int horizon);
};

struct AuxModel {
	GRBModel model;
	mdarray<GRBVar, 1> z; // indices is k (k = day)
	mdarray<GRBVar, 2> x; // indices are k, l (k = day, l = required link in the graph)

	AuxModel(GRBEnv& env, Instance const& inst, Args const& args);

    std::vector<AuxResult> optimize();
	double runtime();
};