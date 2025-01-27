#include "best.hpp"

#include <fstream>

#include "ArcRoute.hpp"
#include "Instance.hpp"
#include "Model_RT.hpp"


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

void BestSolution::writeFile(Instance const& inst, bool multi)
{
	std::ofstream fout;
	if(multi)
		fout.open("cap_solutions/sol_cap_" + inst.name +".txt");
	else
		fout.open("solutions/sol_" + inst.name +".txt");

	for(int t = 0; t < inst.horizon; ++t)
	{
		if(!req_link_visited[t].empty())
		{
			fout << "routes in period " << t << ":" << std::endl;
			for(auto r : best_routes[t])
			{
				fout << "cost " << r.cost << ": ";
				for(auto l : r.full_path)
					fout << "(" << l.first << ", " << l.second << ") ";
				fout << std::endl;
			}
			fout << "visits in period " << t << ":" << std::endl;
			for(auto l : req_link_visited[t])
			{
				fout << "(" << inst.links[l].first << ", " << inst.links[l].second << ") ";
			}
			fout << std::endl << std::endl;
		}
	}
	fout << "total cost: " << cost << std::endl;
	fout.close();

}
