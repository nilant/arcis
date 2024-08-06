#include <fmt/printf.h>
#include <limits>
#include <vector>

#include "Heuristic.hpp"
#include "ArcRoute.hpp"
#include "local_search.hpp"
#include "ModelRT.hpp"
#include "Result.hpp"
#include "RouteSolver.hpp"
#include "best.hpp"
#include "Preprocessing.hpp"
#include "bounds.hpp"
#include "timer.hpp"



Result heur(Instance& inst, Args const& args) {

// -------------------------------------------------------- //
	Timer timer{};
	timer.start("total");
	timer.start("best_time_ls");

	RandomGenerator rand_gen{};

	GRBEnv env{};

	#ifdef NDEBUG
		env.set(GRB_IntParam_OutputFlag, 0);
	#else
		env.set(GRB_IntParam_OutputFlag, 1);
	#endif

	Result total_result;
	total_result.name = "arcis";

	std::vector<ArcRoute> all_routes;

	int restart = 0;
	double best_cost = std::numeric_limits<double>::max();
	double best_vidal_cost = std::numeric_limits<double>::max();
	double residual_timelimit = args.timelimit;
	
	while (residual_timelimit > 5.0) {
		fmt::print("restart with residual timelimit {}\n", residual_timelimit);
		
		Preprocessing prepro(inst);
		prepro.run(inst, rand_gen);

		auto vidal_res = solve_route_vidal(inst, prepro.carpMap, residual_timelimit, args.vidal_iterlimit);
		total_result.vidal_time += vidal_res.time;
		residual_timelimit -= vidal_res.time;

		all_routes.insert(all_routes.end(), vidal_res.all_routes.begin(), vidal_res.all_routes.end());

		RTModel rt_model{env, inst, all_routes, std::max(10.0, residual_timelimit), args.threads};	
		auto rt_res = rt_model.optimize(inst, all_routes);
		double rt_start_cost = rt_res.cost;
		total_result.gurobi_time += rt_res.time;

		residual_timelimit -= rt_res.time;

		fmt::print("rt_start_cost={}, gurobi_time={}\n", rt_start_cost, rt_res.time);

		if (rt_res.cost < best_vidal_cost) {

			best_vidal_cost = rt_res.cost;

			fmt::print("starting local search...\n");
			auto best = BestSolution(inst, all_routes, rt_res);
			if (residual_timelimit > 5.0) {

				auto [ls_time, ls_iter] = local_search(env, inst, rand_gen, best, rt_res, residual_timelimit, 1, args.threads, total_result.gurobi_time);
				residual_timelimit -= ls_time;
				total_result.ls_iter += ls_iter;
				total_result.ls_time += ls_time;
			}


			if (best.cost < best_cost) {
				best_cost = best.cost;
				total_result.vidal_obj = vidal_res.cost;
				total_result.rt_obj = rt_start_cost;
				total_result.ls_obj = best.cost;
				total_result.best_time_ls = best.time;
				total_result.best_iter_ls = best.iter;

				total_result.nroutes = all_routes.size();
				total_result.best_restart = restart;
			}

			all_routes.clear();
		}

		restart++;
	}

	total_result.total_restart = restart;
	total_result.lb = lower_bound(inst);

	timer.stop("total");
	total_result.total_time = timer.duration("total");

// -------------------------------------------------------- //

	return total_result;
}
