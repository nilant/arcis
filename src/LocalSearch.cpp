#include <limits>
#include <vector>
#include <fmt/core.h>
#include "LocalSearch.hpp"
#include "ArcRoute.hpp"
#include "ModelRT.hpp"
#include "best.hpp"
#include "utils.hpp"

ArcRoute inserts(Instance const& inst, std::vector<std::pair<int, int>> vecLinks, ArcRoute const& route){

	int firstU = vecLinks[0].first;
	int lastV = vecLinks.back().second;
	int l_cost = 0;
	for(auto link: vecLinks)
		l_cost += inst.trav_cost(link.first, link.second);

	int insert_point = 0;
	int best_insert_point = 0;
	int best_cost = std::numeric_limits<int>::max();
	int new_cost = best_cost;
	int bestNode = -1;
	for(auto l: route.full_path){
		int node = l.first;
		new_cost = inst.dist(node, firstU) + inst.dist(lastV, node);
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

std::vector<ArcRoute> generate_new_routes(Instance& inst, BestSolution const& best_sol, RandomGenerator& rand_gen){

	std::vector<ArcRoute> new_routes;
	rand_floyd_warshall(inst.trav_cost, inst.dist, inst.prev, rand_gen, inst.required);

	for(int t = 0; t < inst.horizon; ++t){
		for(auto& route: best_sol.best_routes[t]){
			new_routes.push_back(route);
			new_routes.back().mipStart = true;
			new_routes.back().period = t;
			for(int fromIndex = 0; fromIndex < route.full_path.size(); fromIndex++){
				int fromU = route.full_path[fromIndex].first;
				int fromV = route.full_path[fromIndex].second;
				if(!inst.required(fromU, fromV))
					continue;

				std::vector<int> toIndexVector;
				int prevNewRouteCost = route.cost;
				for(int toIndex = fromIndex; toIndex < route.full_path.size(); toIndex++){
					int toU = route.full_path[fromIndex].first;
					int toV = route.full_path[fromIndex].second;
					if(!inst.required(toU, toV))
						continue;

					auto new_route1 = removes(inst, fromIndex, toIndex, route);
					if(new_route1.cost < prevNewRouteCost && !new_route1.full_path.empty()){
						toIndexVector.push_back(toIndex);
						// de-comment if you want to add all these routes (as before...)
						// auto r1 = split_route_at_depot(inst, new_route1);
						// new_routes.insert(new_routes.end(), r1.begin(), r1.end());
						prevNewRouteCost = new_route1.cost;
					}
				}

				if(!toIndexVector.empty()){
					// std::shuffle(toIndexVector.begin(), toIndexVector.end(), rand_gen.gen);
					int size = (int) toIndexVector.size();
					int maxTo = ceil(size*0.001);
					for(int index = 0; index < maxTo; index++){
						int toIndex = toIndexVector[index];

						///////////////////////////////////////////////////////////////////////////////////
						auto new_route1 = removes(inst, fromIndex, toIndex, route);
						auto r1 = split_route_at_depot(inst, new_route1);
						new_routes.insert(new_routes.end(), r1.begin(), r1.end());
						///////////////////////////////////////////////////////////////////////////////////

						auto end_it = route.full_path.begin() + toIndex + 1; // +1 per includere to_index
						std::vector<std::pair<int, int>> vecLinks;
						std::copy(route.full_path.begin() + fromIndex, end_it, std::back_inserter(vecLinks));

						for(int tt = 0; tt < inst.horizon; ++tt){
							if(t != tt){
								int indexRoute = 0;
								int bestIndexRoute = -1;
								int bestDiffCost = INF;
								for(auto const& second_route: best_sol.best_routes[tt]){
									bool checkIfContainOrNotRequired = true;
									for(auto l: vecLinks){
										bool doNotContain = !second_route.contains(inst.id(l.first, l.second));
										bool isRequired = inst.t_l_matrix(tt, inst.id(l.first, l.second)) > 0;
										if(isRequired && doNotContain){
											checkIfContainOrNotRequired = false;
											break;
										}
									}
									if(checkIfContainOrNotRequired){
										indexRoute++;
										continue;
									}
									auto new_route2 = inserts(inst, vecLinks, second_route);
									if(new_route2.cost - second_route.cost < bestDiffCost){
										bestDiffCost = new_route2.cost - second_route.cost;
										bestIndexRoute = indexRoute;
									}else if(new_route2.cost - second_route.cost == bestDiffCost){
										int coin = (int) round(rand_gen.rand_0_1());
										if(coin){
											bestDiffCost = new_route2.cost - second_route.cost;
											bestIndexRoute = indexRoute;
										}
									}
									indexRoute++;
								}
								if(bestIndexRoute > -1){
									auto new_route2 = inserts(inst, vecLinks, best_sol.best_routes[tt][bestIndexRoute]);
									auto r2 = split_route_at_depot(inst, new_route2);
									new_routes.insert(new_routes.end(), r2.begin(), r2.end());
								}
							}
						}
					}
				}
			}
		}
	}

	return new_routes;
}

std::pair<double, int> local_search(GRBEnv& env, Instance& inst, Args const& args, RandomGenerator& rand_gen, BestSolution& curr_best, RTResult& curr_rt_res,
                  int timelimit, int iterlimit, double& gurobi_time){

	Timer timer{};
	timer.start("iter");

	fmt::print("iterlimit={}, timelimit={}\n", iterlimit, timelimit);

	double runtime = 0;
	int best_cost = curr_best.cost;
	int prev_cost = std::numeric_limits<int>::max();

	int iter = 1;
	int best_iter = 0;
	double best_time = 0;
	while (iter - best_iter <= iterlimit) {

		timer.start("local");
		
		auto new_routes = generate_new_routes(inst, curr_best, rand_gen);
		fmt::print("New route generated\n");
		RTModel rt_model{env, inst, args, new_routes};
		
		timer.stop("local");
		runtime += timer.duration("local");

		curr_rt_res = rt_model.optimize(inst, new_routes);
		gurobi_time += curr_rt_res.runtime;
		curr_best = BestSolution(inst, new_routes, curr_rt_res);

		fmt::print("curr_iter={}, curr_best={}\n", iter, best_cost);

		if(curr_best.cost < best_cost){
			timer.stop("iter");
			best_cost = curr_best.cost;
			best_iter = iter;
			best_time = timer.duration("iter");
		}
		iter++;
	}

	curr_best.iter = best_iter;
	curr_best.time = best_time;

	fmt::print("best_iter={}, best_cost={}\n", best_iter, best_cost);

	timer.stop("local");
	return {runtime, iter};
}