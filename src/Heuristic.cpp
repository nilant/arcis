#include <fmt/printf.h>
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



Result run(Instance& inst, Args const& args, GRBEnv& env, RandomGenerator& rand_gen, int timelimit) {

	Timer timer{};
	timer.start("total");

	Preprocessing prepro(inst);
	prepro.run(inst,rand_gen);

// -------------------------------------------------------- //
	timer.start("vidal");

	std::vector<ArcRoute> all_routes;
	all_routes.reserve(inst.horizon * inst.nvehicles);

	double vidal_cost{0.0};
	for (auto& [k, carp_inst] : prepro.carpMap) {
		if(!carp_inst.link_to_visit.empty()){

		auto solver = RouteSolver{};
		auto routes = solver.solve_routes(inst, k, carp_inst, static_cast<int>(args.timelimit*0.1), args.vidal_iterlimit);
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
	fmt::print("vidal_cost={}\nVIDAL END!\n", vidal_cost);
// -------------------------------------------------------- //

	RTModel rt_model{env, inst, all_routes, args.timelimit, args.threads};	
	RTResult rt_res = rt_model.optimize(inst, all_routes);

	double gurobi_time = rt_res.runtime;

	BestSolution best(inst, all_routes, rt_res);
	int rt_cost = best.cost;

// -------------------------------------------------------- //

	int timelimit_ls = args.timelimit - timer.duration("vidal");
	auto [ls_time, ls_iter] = local_search(env, inst, args, rand_gen, best, rt_res, timelimit_ls, 1, gurobi_time);

	timer.stop("total");

	Result res{};
	res.name = "arcis";

	res.vidal_obj = vidal_cost;
	res.rt_obj = rt_cost;
	res.ls_obj = best.cost;

	res.vidal_time = RouteSolver::call_time;

	res.gurobi_time = gurobi_time;
	res.total_time = timer.duration("total");

	res.ls_time = ls_time;
	res.ls_iter = ls_iter;

	res.best_iter_ls = best.iter;
	res.best_time_ls = best.time;

	res.nroutes = nroutes;
	res.lb = lower_bound(inst);

	return res;
}

void update_total_result(Result& total_res, Result& res, Timer& timer) {

	if (res.ls_obj < total_res.ls_obj) {
		total_res.ls_obj = res.ls_obj;
		total_res.vidal_obj = res.vidal_obj;
		total_res.rt_obj = res.rt_obj;
		total_res.best_restart = res.restart;
		total_res.best_iter_ls = res.best_iter_ls;

		timer.stop("best_time_ls");
		total_res.best_time_ls = timer.duration("best_time_ls");
	}

	total_res.vidal_time = RouteSolver::call_time;
	total_res.gurobi_time += res.gurobi_time;
	total_res.total_time += res.total_time;

	total_res.ls_time += res.ls_time;
	total_res.ls_iter += res.ls_iter;

	total_res.nroutes = res.nroutes;
	total_res.lb = res.lb;
}

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
	double timelimit = args.timelimit;

	int restart = 0;
	double residual_timelimit = timelimit;
	
	while (residual_timelimit > 5.0) {
		fmt::print("restart with residual timelimit {}\n", residual_timelimit);
		auto res = run(inst, args, env, rand_gen, timelimit);
		residual_timelimit -= res.total_time;
		restart++;
		res.restart = restart;
		update_total_result(total_result, res, timer);
	}

	total_result.total_restart = restart;

	timer.stop("total");
	total_result.total_time = timer.duration("total");

// -------------------------------------------------------- //

	return total_result;
}
