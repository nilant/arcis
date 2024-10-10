#include <fmt/core.h>

#include "RouteSolver.hpp"
#include "../libs/vidal/Genetic.h"
#include "ArcRoute.hpp"
#include "Preprocessing.hpp"

std::vector<ArcRoute> solve_routes(Instance const& inst, int t, CarpInstance const& carp_inst, int timelimit, int iterlimit, bool multi) {

	std::vector<ArcRoute> routes;

	std::vector<bool> required(inst.nreq_links, false);
	for (auto l : carp_inst.link_to_visit) {
		required[l] = true;
	}

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
	Genetic solver(mesParametres, population, nb_ticks_allowed, false);

	solver.evolve(iterlimit, 1); // First parameter controls the number of iterations without improvement before termination
	
	auto sol = population->BestSolution();

	for (const auto & vidal_links : sol.second) {
		ArcRoute route{inst, vidal_links, t};
		auto splitted_routes = split_route_at_depot(inst, route);
		if(multi == 1){
			int next = 1;
			for(auto& r: splitted_routes){
				std::fill(r._links.begin(), r._links.end(), false);
				r.residual_capacity = inst.capacity;
				for(auto l: r.full_path){
					int id_next = inst.id(vidal_links[next].first - 1, vidal_links[next].second - 1);
					int id_l = inst.id(l.first, l.second);
					if(id_l == id_next){
						r.links(id_l) = true;
						r.residual_capacity -= inst.demand(l.first, l.second);
						next++;
					}
				}
			}
		}
		routes.insert(routes.end(), splitted_routes.begin(), splitted_routes.end());
	}

	delete mesParametres;
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
      		auto routes = solve_routes(inst, k, carp_inst, n_link_to_visit, vidal_iterlimit, multi);
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