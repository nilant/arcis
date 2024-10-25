#include <limits>
#include <vector>
#include <fmt/core.h>
#include "local_search.hpp"
#include "ArcRoute.hpp"
#include "Model_RT.hpp"
#include "best.hpp"
#include "Preprocessing.hpp"
#include "RouteSolver.hpp"

ArcRoute inserts_all(Instance const& inst, std::vector<std::pair<int, int>> vecLinks, ArcRoute const& route){

	int firstU = vecLinks[0].first;
	int lastV = vecLinks.back().second;

	int insert_point = 0;
	int best_insert_point = 0;
	int best_cost = std::numeric_limits<int>::max();
	int bestNode = -1;
	for(auto l: route.full_path){
		int node = l.first;
		int new_cost = inst.dist(node, firstU) + inst.dist(lastV, node);
		if(new_cost < best_cost){
			best_cost = new_cost;
			best_insert_point = insert_point;
			bestNode = node;
		}
		insert_point++;
	}

	ArcRoute new_route{route};
	new_route.mipStart = false;
	new_route.insert_links(inst, vecLinks, bestNode, best_insert_point);
	return new_route;
}

ArcRoute removes(Instance const& inst, int fromLink, int toLink, ArcRoute const& route){
	ArcRoute new_route{route};
	new_route.mipStart = false;
	new_route.remove_links(inst, fromLink, toLink);
	return new_route;
}

std::vector<ArcRoute> generate_new_routes(Instance& inst, const BestSolution& best_sol, int multi){

	std::vector<ArcRoute> new_routes;
	for(int t = 0; t < inst.horizon; ++t){
		for(auto& route: best_sol.best_routes[t]){
			new_routes.push_back(route);
			new_routes.back().mipStart = true;
			new_routes.back().period = t;

			for(int fromIndex = 0; fromIndex < route.full_path.size(); fromIndex++){
				std::vector<int> toIndexVector;
				int prevNewRouteCost = route.cost;
				int toIndexSize = 0;
				std::vector<ArcRoute> r1;
				for(int toIndex = fromIndex; toIndex < route.full_path.size(); toIndex++){
					auto new_route1 = removes(inst, fromIndex, toIndex, route);
					if(new_route1.cost < prevNewRouteCost && !new_route1.full_path.empty()){
						toIndexVector.push_back(toIndex);
						toIndexSize++;
						r1 = split_route_at_depot(inst, new_route1);
						new_routes.insert(new_routes.end(), r1.begin(), r1.end());
						prevNewRouteCost = new_route1.cost;
					}
					if((toIndexSize >= 2 && multi == 2) || toIndexSize >= 10)
						break;
				}

				for(int toIndex : toIndexVector){
					auto end_it = route.full_path.begin() + toIndex + 1; // +1 per includere to_index
					std::vector<std::pair<int, int>> vecLinks;
					std::copy(route.full_path.begin() + fromIndex, end_it, std::back_inserter(vecLinks));
					for(int tt = 0; tt < inst.horizon; ++tt){
						if(t != tt){
							int indexRoute = 0;
							int bestIndexRoute = -1;
							int bestDiffCost = std::numeric_limits<int>::max();
							for(auto const& second_route: best_sol.best_routes[tt]){
								auto new_route2 = inserts_all(inst, vecLinks, second_route);
								if(new_route2.cost - second_route.cost < bestDiffCost){
									bestDiffCost = new_route2.cost - second_route.cost;
									bestIndexRoute = indexRoute;
								}
								indexRoute++;
							}
							if(bestIndexRoute > -1){
								auto new_route2 = inserts_all(inst, vecLinks, best_sol.best_routes[tt][bestIndexRoute]);
								auto r2 = split_route_at_depot(inst, new_route2);
								new_routes.insert(new_routes.end(), r2.begin(), r2.end());
							}
							else if(best_sol.best_routes[tt].empty()){
								auto new_route2 = ArcRoute(inst, vecLinks, tt, false);
								auto r2 = split_route_at_depot(inst, new_route2);
								new_routes.insert(new_routes.end(), r2.begin(), r2.end());
							}
						}
					}
				}
			}
		}
	}

	if(multi == 1){
		RandomGenerator rand_gen{};
		for(auto& r: new_routes){
			if(!r.mipStart && r.residual_capacity < 0){
				std::vector<int> rand_index(r.full_path.size());
				for(int id = 0; id < r.full_path.size(); id++)
					rand_index[id] = id;
				std::shuffle(rand_index.begin(), rand_index.end(), rand_gen.gen);

				for(auto l : rand_index){
					int id_l = inst.id(r.full_path[l].first, r.full_path[l].second);
					if(r.links(id_l)){
						r.links(id_l) = false;
						r.residual_capacity += inst.demand(r.full_path[l].first, r.full_path[l].second);
					}
					if(r.residual_capacity >= 0)
						break;
				}
				r.make_maximal(inst);
			}
		}
	}

	return new_routes;
}

std::pair<double, int>
local_search(GRBEnv& env, Instance& inst, BestSolution& curr_best, RTResult& curr_rt_res, double& gurobi_time, double& vidal_time, Args const& args){

	Timer timer{};


	double iter_ls_time = 0;
	double best_cost = curr_best.cost;

	int iter = 1;
	int best_iter = 0;
	double last_vidal_cost = curr_best.cost;
	while(iter - best_iter <= 1){

		timer.start("local");
		auto new_routes = generate_new_routes(inst, curr_best, args.multi);
		fmt::print("{} new routes generated. \n", new_routes.size());
		if(args.multi > 1){
			mRTModel rt_model{env, inst, new_routes};
			timer.stop("local");
			iter_ls_time += timer.duration("local");
			curr_rt_res = rt_model.optimize(new_routes);
		}
		else{
			RTModel rt_model{env, inst, new_routes, args.multi};
			timer.stop("local");
			iter_ls_time += timer.duration("local");
			curr_rt_res = rt_model.optimize(new_routes, inst, args.multi);
		}
		gurobi_time += curr_rt_res.time;
		fmt::print("gurobi time={}\n", curr_rt_res.time);

		curr_best = BestSolution(inst, new_routes, curr_rt_res);
		if(curr_best.cost < best_cost){
			best_cost = curr_best.cost;
			best_iter = iter;
		}

		fmt::print("curr_iter={}, curr_best={}\n", iter, best_cost);

		iter++;
		if(iter - best_iter > 1 && best_cost < last_vidal_cost){
			std::map<int, std::set<int>> lastCarpMap;
			for(int t = 0; t < inst.horizon; ++t)
				for(auto& l: curr_best.req_link_visited[t])
					lastCarpMap[t].insert(l);

			auto last_vidal_res = solve_route_vidal(inst, lastCarpMap, args.vidal_iterlimit, args.multi);
			vidal_time += last_vidal_res.time;
			std::vector<ArcRoute> last_routes;
			last_routes.insert(last_routes.end(), last_vidal_res.all_routes.begin(), last_vidal_res.all_routes.end());
			RTResult last_rt_res;
			if(args.multi > 1){
				mRTModel last_rt_model{env, inst, last_routes};
				last_rt_res = last_rt_model.optimize(last_routes);
			}
			else{
				RTModel last_rt_model{env, inst, last_routes, args.multi};
				last_rt_res = last_rt_model.optimize(last_routes, inst, args.multi);
			}
			curr_best = BestSolution(inst, last_routes, last_rt_res);
			last_vidal_cost = curr_best.cost;
			if(curr_best.cost < best_cost){
				best_cost = curr_best.cost;
				best_iter = iter;
			}
			fmt::print("curr_best_after_vidal={}\n", best_cost);
		}
		else if(best_cost == last_vidal_cost)
			break;
	}
	curr_best.iter = best_iter;
	return {iter_ls_time, iter};
}
