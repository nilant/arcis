#include "RouteSolver.hpp"
#include "../libs/vidal/Genetic.h"
#include "ArcRoute.hpp"
#include "Preprocessing.hpp"


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

    return routes;
}