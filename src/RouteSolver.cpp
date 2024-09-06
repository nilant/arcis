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

	vector < Population * > populationTab ;
	vector < Params * > mesParametresTab ;

	Population * population ; 
	Params * mesParametres ;
	clock_t nb_ticks_allowed;

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
	mesParametres = new Params(1, inst, nVeh_UB, required, multi);

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
	double vidal_cost{0.0};
	for (auto& [k, carp_inst] : carp_map) {
		if(!carp_inst.link_to_visit.empty()){

			int n_link_to_visit = (int) carp_inst.link_to_visit.size();
			auto solver = RouteSolver{};
      		auto routes = solver.solve_routes(inst, k, carp_inst, n_link_to_visit, vidal_iterlimit, multi);
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