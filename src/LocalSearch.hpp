#pragma once
#include "ArcRoute.hpp"
#include "ModelRT.hpp"
#include "best.hpp"


void local_search(GRBEnv& env, Instance const& inst, Args const& args, BestSolution& curr_best, RTResult& rt_res, int timelimit, int iterlimit);

std::vector<ArcRoute> generate_new_routes(Instance const& inst, BestSolution const& best_sol, RTResult const& rt_res);

// try to insert link l BEFORE each link in route and select the position
// with lower (heuristically calculated) cost.
ArcRoute insert(Instance const& inst, std::pair<int, int> link, ArcRoute const& route);

// return a route without l and with (heuristically calculated) new cost.
ArcRoute remove(Instance const& inst, std::pair<int, int> link, ArcRoute const& route);