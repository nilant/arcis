#include <fmt/core.h>

#include "RouteSolver.hpp"
#include "../libs/vidal/Genetic.h"
#include "ArcRoute.hpp"
#include "Preprocessing.hpp"


double RouteSolver::call_time = 0;

std::vector<ArcRoute> RouteSolver::solve_routes(Instance const& inst, int t, CarpInstance const& carp_inst, int timelimit, int iterlimit) {

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

	// initialisation of the Parameters
	mesParametres = new Params(42, inst, carp_inst.nVehicle, required) ;

	// Running the algorithm
	population = new Population(mesParametres) ;
	Genetic solver(mesParametres,population,nb_ticks_allowed, false);

	solver.evolve(iterlimit, 1); // First parameter controls the number of iterations without improvement before termination
	
	auto sol = population->BestSolution();

	for (int veh = 0; veh < sol.second.size(); ++veh) {
		ArcRoute route{inst, sol.second[veh], veh, t};
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

VidalResult solve_route_vidal(Instance const& inst, std::map<int, CarpInstance> const& carp_map, double timelimit, int vidal_iterlimit) {

	Timer timer{};
	timer.start("vidal");

	std::vector<ArcRoute> all_routes;
	all_routes.reserve(inst.horizon * inst.nvehicles);

	double vidal_cost{0.0};
	for (auto& [k, carp_inst] : carp_map) {
		if(!carp_inst.link_to_visit.empty()){

			int n_link_to_visit = (int) carp_inst.link_to_visit.size();
			double instance_time_limit = ((double) n_link_to_visit/inst.tot_services)*timelimit;
			auto solver = RouteSolver{};
			auto routes = solver.solve_routes(inst, k, carp_inst, static_cast<int>(instance_time_limit - 1), vidal_iterlimit);
			all_routes.insert(all_routes.end(), routes.begin(), routes.end());

			for(auto const& route: routes){
				vidal_cost += route.cost;
			}
		}
	}

	int nroutes = (int) all_routes.size();

	for (auto& route : all_routes) {
		route.mipStart = true;
	}
	timer.stop("vidal");

	return VidalResult{.all_routes = all_routes, .cost = vidal_cost, .time = timer.duration("vidal")};
}