#pragma once

#include "Instance.hpp"
#include "ArcRoute.hpp"
#include "timer.hpp"
#include "cli.hpp"
#include <gurobi_c++.h>
#include <limits>


struct RTResult {
    mdarray<int, 2> y_val;
    mdarray<int, 2> x_val;
    int cost{std::numeric_limits<int>::max()};
    double time{0};

    RTResult() = default;
    RTResult(Instance const& inst, mdarray<GRBVar, 2> const& y, mdarray<GRBVar, 2> const& x, std::vector<ArcRoute> const& routes, int obj, double time);
};

struct RTModel {
	GRBModel model;
    // indices are l, t (l = link, t = period). 1 iff link l is served in period t 
    mdarray<GRBVar, 2> x;
    // indices are r, t (r = route, t = period). 1 iff route r is used in period t 
    mdarray<GRBVar, 2> y;

    Timer timer{};

    RTModel(GRBEnv& env, Instance const& inst, std::vector<ArcRoute> const& routes, double timelimit, int threads);
    RTResult optimize(Instance const& inst, std::vector<ArcRoute> const& routes);
};
