#pragma once

#include "Instance.hpp"
#include "Preprocessing.hpp"
#include "ArcRoute.hpp"


struct RouteSolver {

    static double call_time;
    double _runtime;
    double runtime() { return _runtime; }
    std::vector<ArcRoute> solve_routes(Instance const& inst, int t, CarpInstance const& carp_inst, int timelimit, int iterlimit, bool multi);

};

struct VidalResult {
    std::vector<ArcRoute> all_routes;
    double cost{0};
    double time;
    const int nroutes() const { return all_routes.size();}
};

VidalResult solve_route_vidal(Instance const& inst, std::map<int, CarpInstance> const& carp_map, int vidal_iterlimit, bool multi);