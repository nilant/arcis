#include "best.hpp"
#include "ArcRoute.hpp"
#include "Instance.hpp"
#include "ModelRT.hpp"


BestSolution::BestSolution(Instance const& inst, std::vector<ArcRoute> const& routes, 
                                    RTResult const& rt_res) {

    cost = rt_res.cost;
    best_routes.resize(rt_res.y_val.dimension(1));
	for(int r = 0; r < rt_res.y_val.dimension(0); ++r){
		int t = routes[r].period;
		if (rt_res.y_val(r, t)) {
			best_routes[t].push_back(routes[r]);
		}
	}

	req_link_visited.resize(inst.horizon);
	for(int l = 0; l < inst.nreq_links; ++l){
		for(int t = 0; t < inst.horizon; ++t){
			if (rt_res.x_val(l, t)) {
				req_link_visited[t].push_back(l);
			}
		}
	}
}