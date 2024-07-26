#pragma once
#include "ArcRoute.hpp"
#include "ModelRT.hpp"
#include "best.hpp"

struct RandomGenerator {
	// std::random_device rd;
	std::mt19937 gen; // Generatore di numeri casuali
	std::uniform_int_distribution<> distrib; // Distribuzione uniforme in 0 1

	// Costruttore
	RandomGenerator() : gen(0), distrib(0.0, 1.0) {}

	// Metodo per generare un numero casuale
	int coin() {
		return distrib(gen);
	}

	// Metodo per generare un numero casuale tra 0 e max
	int getRandom(int max) {
		std::uniform_int_distribution<> distrib_int(0, max);
		return distrib_int(gen);
	}
};

static double local_search_time = 0;
void local_search(GRBEnv& env, Instance& inst, Args const& args, BestSolution& curr_best, RTResult& rt_res, int timelimit, int iterlimit);

std::vector<ArcRoute> generate_new_routes(Instance& inst, BestSolution const& best_sol);

// try to insert path
ArcRoute inserts(Instance const& inst, std::vector<std::pair<int, int>> vecLinks, ArcRoute const& route);

// return a route
ArcRoute removes(Instance const& inst, int fromLink, int toLink, ArcRoute const& route);