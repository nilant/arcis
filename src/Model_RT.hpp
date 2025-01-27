#pragma once

#include "Instance.hpp"
#include "ArcRoute.hpp"
#include "timer.hpp"
#include "cli.hpp"
#include <gurobi_c++.h>
#include <limits>


struct RTResult {
    mdarray<int, 2> y_val{0, 0};
    mdarray<int, 2> x_val{0, 0};
    int cost{std::numeric_limits<int>::max()};
    double time{0};

	// RTResult(int y_dim1, int y_dim2, int x_dim1, int x_dim2): y_val{y_dim1,y_dim2},x_val{x_dim1, x_dim2}{};

    RTResult() = default;
    RTResult(mdarray<GRBVar, 2> const& y, mdarray<GRBVar, 2> const& x, std::vector<ArcRoute> const& routes, int obj, double time);
	RTResult(mdarray<GRBVar, 2> const& y, mdarray<GRBVar, 3> const& x, std::vector<ArcRoute> const& routes, int obj, double time);
};

struct RTModel {
	GRBModel model;
    // indices are r, t (r = route, t = period). 1 iff route r is used in period t 
    mdarray<GRBVar, 2> y;
	// indices are l, t (l = link, t = period). 1 iff link l is served in period t
	mdarray<GRBVar, 2> x;
    Timer timer{};

    RTModel(GRBEnv& env, Instance const& inst, std::vector<ArcRoute> const& routes);
    RTResult optimize(std::vector<ArcRoute> const& routes);
	bool check_feasibility(std::vector<ArcRoute> const& routes, Instance const& inst);
};