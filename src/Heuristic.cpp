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
#include "timer.hpp"


Result heur(Instance& inst, Args const& args){

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
	total_result.best_restart = 0;

	std::vector<ArcRoute> all_routes;

	int restart = 0;
	double best_cost = std::numeric_limits<double>::max();
	double total_time = 0;
	while(total_time < args.timelimit && restart - total_result.best_restart < args.mls_iterlimit){
		timer.start("iteration");

		fmt::print("restart after {} seconds.\n", total_time);

		Preprocessing prepro(inst);
		prepro.run(inst, rand_gen);

		auto vidal_res = solve_route_vidal(inst, prepro.carpMap, args.vidal_iterlimit, args.multi);
		total_result.vidal_time += vidal_res.time;

		all_routes.insert(all_routes.end(), vidal_res.all_routes.begin(), vidal_res.all_routes.end());

		RTResult rt_res; // ((int) all_routes.size(), inst.horizon, inst.nreq_links, inst.horizon);
		if(args.multi > 1){
			mRTModel rt_model{env, inst, all_routes};
			rt_res = rt_model.optimize(all_routes);
		}
		else{
			RTModel rt_model{env, inst, all_routes, args.multi};
			rt_res = rt_model.optimize(all_routes, inst, args.multi);
		}
		double rt_start_cost = rt_res.cost;
		total_result.gurobi_time += rt_res.time;
		fmt::print("vidal_cost={}, rt_start_cost={}, gurobi_time={}\n", vidal_res.cost, rt_start_cost, rt_res.time);
		fmt::print("starting local search...\n");
		std::cout << std::endl;

		auto best = BestSolution(inst, all_routes, rt_res);
		auto [ls_time, ls_iter] = local_search(env, inst, best, rt_res, total_result.gurobi_time,
		                                       total_result.vidal_time, args);
		total_result.ls_iter += ls_iter;
		total_result.ls_time += ls_time;
		all_routes.clear();
		restart++;
		timer.stop("iteration");
		double it_duration = timer.duration("iteration");
		if(it_duration > total_result.longest_it_time)
			total_result.longest_it_time = it_duration;

		if(best.cost < best_cost){
			timer.stop("best_time_ls");
			best_cost = best.cost;
			total_result.vidal_obj = vidal_res.cost;
			total_result.start_obj = rt_start_cost;
			total_result.ls_obj = best.cost;
			total_result.best_time_ls = timer.duration("best_time_ls");
			total_result.best_iter_ls = best.iter;
			total_result.best_it_time = it_duration;
			total_result.best_restart = restart;
		}

		timer.stop("total");
		total_time = timer.duration("total");
	}

	total_result.total_restart = restart;

// -------------------------------------------------------- //

	timer.stop("total");
	total_result.total_time = timer.duration("total");

	return total_result;
}
