#pragma once

#include "Instance.hpp"
#include "Preprocessing.hpp"
#include "ArcRoute.hpp"

std::vector<ArcRoute> solve_routes(Instance const& inst, int t, /*CarpInstance const& carp_inst*/ std::set<int> const& link_to_visit, int timelimit, int iterlimit, bool multi);

struct VidalResult {
    std::vector<ArcRoute> all_routes;
    double cost{0};
    double time;
};

VidalResult solve_route_vidal(Instance const& inst, std::map<int, /*CarpInstance*/std::set<int>> const& carp_map, int vidal_iterlimit, bool multi) ;