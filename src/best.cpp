#include "best.hpp"
#include "ArcRoute.hpp"
#include "Instance.hpp"
#include "ModelRT.hpp"


BestSolution::BestSolution(Instance const& inst, std::vector<ArcRoute> const& routes, 
                                    RTResult const& rt_res) {

    cost = rt_res.cost;
    best_routes.reserve(rt_res.y_val.dimension(1));
    for (int t = 0; t < rt_res.y_val.dimension(1); ++t) {
        std::vector<ArcRoute> best;
        best.reserve(rt_res.y_val.dimension(0));
        for (int r = 0; r < rt_res.y_val.dimension(0); ++r) {
            if (rt_res.y_val(r, t)) {
                best.push_back(routes[r]);
            }
        }

		//if(!best.empty())
		best_routes.push_back(best);
    }
}