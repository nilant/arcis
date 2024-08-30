#pragma once
#include "ArcRoute.hpp"
#include "ModelRT.hpp"
#include "best.hpp"

struct RandomGenerator {
	// std::random_device rd;
	std::mt19937 gen; // Generatore di numeri casuali
	std::uniform_real_distribution<> distrib01; // Distribuzione uniforme in 0 1
	std::uniform_int_distribution<> distribInt;

	// Costruttore
	RandomGenerator() : gen(0), distrib01(0.0, 1.0), distribInt(0.0, std::numeric_limits<int>::max()) {}

	// Metodo per generare un numero casuale tra 0 e max
	int getRandomInt(int max) {
		int output = distribInt(gen) % (max+1);
		return output;
	}
};

std::pair<double, int> local_search(GRBEnv& env, Instance& inst, BestSolution& curr_best, RTResult& rt_res, double& gurobi_time, double& vidal_time, bool multi);

std::vector<ArcRoute> generate_new_routes(Instance& inst, BestSolution const& best_sol);

// try to insert path
ArcRoute inserts(Instance const& inst, std::vector<std::pair<int, int>> vecLinks, ArcRoute const& route);

// return a route
ArcRoute removes(Instance const& inst, int fromLink, int toLink, ArcRoute const& route);