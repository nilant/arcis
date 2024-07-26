#pragma once
#include "ArcRoute.hpp"
#include "ModelRT.hpp"
#include "best.hpp"

struct RandomGenerator {
	// std::random_device rd;
	std::mt19937 gen; // Generatore di numeri casuali
	std::uniform_int_distribution<> distribCoin; // Distribuzione uniforme in 0 1
	std::uniform_int_distribution<> distribInt;

	// Costruttore
	RandomGenerator() : gen(0), distribCoin(0.0, 1.0), distribInt(0.0, std::numeric_limits<int>::max()) {}

	// Metodo per generare un numero casuale
	int coin() {
		return distribCoin(gen);
	}

	// Metodo per generare un numero casuale tra 0 e max
	int getRandomInt(int max) {
		int output = distribInt(gen) % (max+1);
		return output;
	}
};

static double local_search_time = 0;
double local_search(GRBEnv& env, Instance& inst, Args const& args, BestSolution& curr_best, RTResult& rt_res, int timelimit, int iterlimit);

std::vector<ArcRoute> generate_new_routes(Instance& inst, BestSolution const& best_sol);

// try to insert path
ArcRoute inserts(Instance const& inst, std::vector<std::pair<int, int>> vecLinks, ArcRoute const& route);

// return a route
ArcRoute removes(Instance const& inst, int fromLink, int toLink, ArcRoute const& route);