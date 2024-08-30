#include <fmt/core.h>

#include "RouteSolver.hpp"
#include "../libs/vidal/Genetic.h"
#include "ArcRoute.hpp"
#include "Preprocessing.hpp"


double RouteSolver::call_time = 0;

std::vector<ArcRoute> RouteSolver::solve_routes(Instance const& inst, int t, CarpInstance const& carp_inst, int timelimit, int iterlimit, bool multi) {

	auto t0 = std::chrono::high_resolution_clock::now();

	std::vector<ArcRoute> routes;

	std::vector<bool> required(inst.nreq_links);
	for (auto l : carp_inst.link_to_visit) {
		required[l] = true;
	}

	bool minFleetSize ;
	bool minMaxTour ;
	int nbpop = 0;
	vector < Population * > populationTab ;
	vector < Params * > mesParametresTab ;
	Population * lastPop = NULL ;
	Population * population2 ; 
	Population * population ; 
	Params * mesParametres ;
	Params * mesParametres2 ;
	clock_t nb_ticks_allowed;
	double distConstraint ;
	int nbOverallLoop = 0 ;

	// Number of clock ticks allowed for the program
	nb_ticks_allowed = timelimit * CLOCKS_PER_SEC;

	// calculate an upper bound for the number of vehicle
	int nVeh_UB = 1;
	if(multi){
		int residual_cap = inst.capacity;
		for(auto link: carp_inst.link_to_visit){
			if(inst.demand(inst.links[link].first, inst.links[link].second) > residual_cap){
				nVeh_UB++;
				residual_cap = inst.capacity;
			}
			residual_cap -= inst.demand(inst.links[link].first, inst.links[link].second);
		}
	}

	// initialisation of the Parameters
	mesParametres = new Params(0, inst, nVeh_UB /*carp_inst.nVehicle*/, required, multi);

	// Running the algorithm
	population = new Population(mesParametres) ;
	Genetic solver(mesParametres,population,nb_ticks_allowed, false);

	solver.evolve(iterlimit, 1); // First parameter controls the number of iterations without improvement before termination
	
	auto sol = population->BestSolution();

	for (int veh = 0; veh < sol.second.size(); ++veh) {
		ArcRoute route{inst, sol.second[veh], t};
		auto splitted_routes = split_route_at_depot(inst, route);
		int cost_splitted = 0;
		int nserv = 0;
		for (auto const& r : splitted_routes) {
			cost_splitted += r.cost;
			nserv += r.full_path.size();
		}
		assert(cost_splitted == sol.first[veh]);
		assert(nserv == route.full_path.size());
		routes.insert(routes.end(), splitted_routes.begin(), splitted_routes.end());
	}

	delete mesParametres ;

	auto t1 = std::chrono::high_resolution_clock::now();
	_runtime = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count(); 

	RouteSolver::call_time += _runtime / 1000;

    return routes;
}

VidalResult solve_route_vidal(Instance const& inst, std::map<int, CarpInstance> const& carp_map, int vidal_iterlimit, bool multi) {

	Timer timer{};
	timer.start("vidal");

	std::vector<ArcRoute> all_routes;
	// all_routes.reserve(inst.horizon * inst.nvehicles);
	double vidal_cost{0.0};
	for (auto& [k, carp_inst] : carp_map) {
		if(!carp_inst.link_to_visit.empty()){

			int n_link_to_visit = (int) carp_inst.link_to_visit.size();
			auto solver = RouteSolver{};
      		auto routes = solver.solve_routes(inst, k, carp_inst, static_cast<int>(n_link_to_visit - 1), vidal_iterlimit, multi);
			// auto routes = solver.solve_routes(inst, k, carp_inst, 3600, 10000);
			all_routes.insert(all_routes.end(), routes.begin(), routes.end());
			for(auto const& route: routes)
				vidal_cost += route.cost;

		}
	}

	for (auto& route : all_routes) {
		route.mipStart = true;
	}
	timer.stop("vidal");

	return VidalResult{.all_routes = all_routes, .cost = vidal_cost, .time = timer.duration("vidal")};
}

VidalResult solve_route_fast(Instance const& inst, std::map<int, CarpInstance> const& carp_map) {

	Timer timer{};
	timer.start("vidal");

	std::vector<ArcRoute> all_routes;
	// all_routes.reserve(inst.horizon * inst.nvehicles);
	double vidal_cost{0.0};
	for (auto& [k, carp_inst] : carp_map) {
		if(!carp_inst.link_to_visit.empty()){

			auto must_be_visited = carp_inst.link_to_visit;
			std::vector<std::pair<int, int>> new_route;
			int from = 0;
			while(!must_be_visited.empty()){

				int min_dist = std::numeric_limits<int>::max();
				int next_link = -1;
				bool is_reverse = false;
				for(auto link: must_be_visited){
					int check_dist = inst.dist(from, inst.links[link].first);
					bool reverse = false;
					if(inst.type(inst.links[link].first, inst.links[link].second) == EDGE && inst.dist(from, inst.links[link].second) < check_dist){
						check_dist = inst.dist(from, inst.links[link].second);
						reverse = true;
					}
					if(check_dist < min_dist){
						next_link = link;
						min_dist = check_dist;
						is_reverse = reverse;
					}
				}
				if(!must_be_visited.erase(next_link)){
					std::cout << "Something wrong when removing next link" << std::endl;
					exit(1);
				}
				if(is_reverse){
					new_route.emplace_back(inst.links[next_link].second+1, inst.links[next_link].first+1);
					from = inst.links[next_link].first;
				}
				else{
					new_route.emplace_back(inst.links[next_link].first+1, inst.links[next_link].second+1);
					from = inst.links[next_link].second;
				}
			}
			new_route.emplace_back(1,1);
			ArcRoute route{inst, new_route, k};
			auto split_routes = split_route_at_depot(inst, route);
			all_routes.insert(all_routes.end(), split_routes.begin(), split_routes.end());

			for(auto const& r: split_routes){
				vidal_cost += r.cost;
			}
		}
	}

	for (auto& route : all_routes) {
		route.mipStart = true;
	}
	timer.stop("vidal");

	return VidalResult{.all_routes = all_routes, .cost = vidal_cost, .time = timer.duration("vidal")};
}