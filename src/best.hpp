#pragma  once

#include "Model_RT.hpp"
#include "ArcRoute.hpp"


struct BestSolution {

    double cost{0};
    double time{0};
    int iter{0};

    std::vector<std::vector<ArcRoute>> best_routes;
	std::vector<std::vector<int>> req_link_visited;

    BestSolution() = default;
    BestSolution(Instance const& inst, std::vector<ArcRoute> const& routes, RTResult const& rt_res);
};