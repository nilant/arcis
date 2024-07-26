#pragma once

#include "Instance.hpp"
#include "Preprocessing.hpp"
#include "ArcRoute.hpp"


struct RouteSolver {

    static double call_time;
    double _runtime;
    double runtime() { return _runtime; }
    std::vector<ArcRoute> solve_routes(Instance const& inst, int t, CarpInstance const& carp_inst, int timelimit, int iterlimit);
};