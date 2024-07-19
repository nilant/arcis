#pragma  once

#include "ModelRT.hpp"
#include "ArcRoute.hpp"


struct BestSolution {

    double cost{0};
    double time{0};
    int iter{0};

    double best_time{0};
    double best_iter{0};

    double gurobi_time{0};

    std::vector<std::vector<ArcRoute>> best_routes;

    BestSolution(Instance const& inst, std::vector<ArcRoute> const& routes, RTResult const& rt_res);
};