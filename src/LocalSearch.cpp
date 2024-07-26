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
		l_cost += inst.serv_cost(link.first, link.second);

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
	new_route.original = false;
	new_route.insert_links(inst, vecLinks, bestNode, best_insert_point);
	return new_route;
}

ArcRoute removes(Instance const& inst, int fromLink, int toLink, ArcRoute const& route){
	ArcRoute new_route{route};
	new_route.original = false;
	new_route.remove_links(inst, fromLink, toLink);
	return new_route;
}

std::vector<ArcRoute> generate_new_routes(Instance& inst, BestSolution const& best_sol, RandomGenerator& rand_gen){

	std::vector<ArcRoute> new_routes;
	rand_floyd_warshall(inst.trav_cost, inst.dist, inst.prev, rand_gen);

	for(int t = 0; t < inst.horizon; ++t){
		for(auto& route: best_sol.best_routes[t]){
			new_routes.push_back(route);
			new_routes.back().mipStart.clear();
			new_routes.back().mipStart.insert(t);
			for(int fromIndex = 0; fromIndex < route.full_path.size(); fromIndex++){

				std::vector<int> randToIndex;
				int prevNewRouteCost = route.cost;
				for(int toIndex = fromIndex; toIndex < route.full_path.size(); toIndex++){
					auto new_route1 = removes(inst, fromIndex, toIndex, route);
					if(new_route1.cost < prevNewRouteCost && !new_route1.full_path.empty()){
						randToIndex.push_back(toIndex);
						auto r1 = split_route_at_depot(inst, new_route1);
						new_routes.insert(new_routes.end(), r1.begin(), r1.end());
						prevNewRouteCost = new_route1.cost;
						/*auto end_it = route.full_path.begin() + toIndex + 1; // +1 per includere to_index
						std::vector<std::pair<int, int>> vecLinks;
						std::copy(route.full_path.begin() + fromIndex, end_it, std::back_inserter(vecLinks));
						for(int tt = 0; tt < inst.horizon; ++tt){
							// if(t != tt){
								int bestIndexRoute = -1;
								int bestDiffCost = INF;
								int indexRoute = 0;
								for(auto const& second_route: best_sol.best_routes[tt]){
									bool checkIfContain = true;
									for(auto l: vecLinks){
										if(!second_route.contains(inst.id(l.first, l.second))){
											checkIfContain = false;
											break;
										}
									}
									if(checkIfContain){
										indexRoute++;
										continue;
									}
									auto new_route2 = inserts(inst, vecLinks, second_route);
									if(new_route2.cost - second_route.cost < bestDiffCost){
										bestDiffCost = new_route2.cost - second_route.cost;
										bestIndexRoute = indexRoute;
									}
									indexRoute++;
								}
								if(bestIndexRoute > -1){
									auto new_route2 = inserts(inst, vecLinks,
									                          best_sol.best_routes[tt][bestIndexRoute]);
									auto r2 = split_route_at_depot(inst, new_route2);
									new_routes.insert(new_routes.end(), r2.begin(), r2.end());
								}
							// }
						}*/
					}
				}

				if(!randToIndex.empty()){
					int maxRand = randToIndex.size() - 1;
					std::vector<int>::size_type random_index = rand_gen.getRandom(maxRand);
					int toIndex = randToIndex[random_index];
					auto end_it = route.full_path.begin() + toIndex + 1; // +1 per includere to_index
					std::vector<std::pair<int, int>> vecLinks;
					std::copy(route.full_path.begin() + fromIndex, end_it, std::back_inserter(vecLinks));

					for (int tt = 0; tt < inst.horizon; ++tt) {
						if(t != tt){
							int indexRoute = 0;
							int bestIndexRoute = -1;
							int bestDiffCost = INF;
							for(auto const& second_route: best_sol.best_routes[tt]){
								bool checkIfContain = true;
								for(auto l: vecLinks){
									if(!second_route.contains(inst.id(l.first, l.second))){
										checkIfContain = false;
										break;
									}
								}
								if(checkIfContain){
									indexRoute++;
									continue;
								}
								auto new_route2 = inserts(inst, vecLinks, second_route);
								if(new_route2.cost - second_route.cost < bestDiffCost){
									bestDiffCost = new_route2.cost - second_route.cost;
									bestIndexRoute = indexRoute;
								}else if(new_route2.cost - second_route.cost == bestDiffCost){
									int coin = rand_gen.coin();
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

	return new_routes;
}

void local_search(GRBEnv& env, Instance& inst, Args const& args, BestSolution& curr_best, RTResult& curr_rt_res,
                  int timelimit, int iterlimit){

	Timer timer{};
	timer.start("local");

	fmt::print("iterlimit={}, timelimit={}\n", iterlimit, timelimit);
	auto t0 = std::chrono::high_resolution_clock::now();

	double runtime = 0;
	int best_cost = curr_best.cost;
	double gurobi_time = curr_rt_res.runtime;
	int prev_cost = std::numeric_limits<int>::max();
	RandomGenerator rand_gen;

	int iter = 0;
	int best_iter = 0;
	double best_time = 0;
	for(; iter <= iterlimit && runtime <= timelimit; ++iter){

		auto new_routes = generate_new_routes(inst, curr_best, rand_gen);
		std::cout << "New route generated" << std::endl;
		RTModel rt_model{env, inst, args, new_routes};

		curr_rt_res = rt_model.optimize(inst, new_routes);
		curr_best = BestSolution(inst, new_routes, curr_rt_res);

		gurobi_time += curr_best.gurobi_time;

		fmt::print("iter={}, best={}\n", iter, best_cost);
		auto t1 = std::chrono::high_resolution_clock::now();
		runtime = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count()/1000.0;

		if(curr_best.cost < best_cost){
			best_cost = curr_best.cost;
			best_iter = iter;
			best_time = runtime;
		}
		prev_cost = best_cost;
	}

	curr_best.iter = iter;
	curr_best.time = runtime;

	curr_best.best_time = best_time;
	curr_best.best_iter = best_iter;

	curr_best.gurobi_time = gurobi_time;

	fmt::print("best_iter={}, best_cost={}\n", best_iter, best_cost);
	timer.stop("local");

	local_search_time += timer.duration("local");
}