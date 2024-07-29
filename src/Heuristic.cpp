#include <fmt/printf.h>
#include <vector>

#include "Heuristic.hpp"
#include "ArcRoute.hpp"
#include "LocalSearch.hpp"
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

	std::vector<ArcRoute> all_routes;
	all_routes.reserve(inst.horizon * inst.nvehicles);

	double vidal_cost{0.0};
	for (auto& [k, carp_inst] : prepro.carpMap) {
		if(!carp_inst.link_to_visit.empty()){

		auto solver = RouteSolver{};
		auto routes = solver.solve_routes(inst, k, carp_inst, static_cast<int>(args.timelimit*0.1), args.iterlimit);
		all_routes.insert(all_routes.end(), routes.begin(), routes.end());

			for(auto const& route: routes){
				vidal_cost += route.cost;
			}
		}
	}
	// print_routes(inst, "data/sol/"+inst.name+"_sol.txt", all_routes);
	int nroutes = (int) all_routes.size();
	timer.stop("vidal");
	std::cout << "VIDAL END!" << std::endl;
// -------------------------------------------------------- //

	RTModel rt_model{env, inst, args, all_routes};	
	RTResult rt_res = rt_model.optimize(inst, all_routes);

	BestSolution best(inst, all_routes, rt_res);
	int rt_cost = best.cost;


// -------------------------------------------------------- //

	int timelimit_ls = args.timelimit - timer.duration("vidal");
	local_search(env, inst, args, best, rt_res, timelimit_ls, 3);

	timer.stop("total");

	Result res{};
	res.name = "arcis";

	res.vidal_obj = vidal_cost;
	res.rt_obj = rt_cost;
	res.ls_obj = best.cost;

	res.vidal_time = RouteSolver::call_time;
	res.rt_time = RTModel::call_time;
	res.ls_time = local_search_time;

	res.gurobi_time = RTModel::grb_time;
	res.total_time = timer.duration("total");

	res.time_ls = best.time;
	res.iter_ls = best.iter;

	res.best_iter_ls = best.best_iter;
	res.best_time_ls = best.best_time;

	res.nroutes = nroutes;
	res.lb = lower_bound(inst);

	return res;
}


Result heur(Instance& inst, Args const& args) {

// -------------------------------------------------------- //
	Timer timer{};
	timer.start("total");

	RandomGenerator rand_gen{};

	GRBEnv env{};

	#ifdef NDEBUG
		env.set(GRB_IntParam_OutputFlag, 0);
	#else
		env.set(GRB_IntParam_OutputFlag, 1);
	#endif

	double timelimit = args.timelimit;
	auto res = run(inst, args, env, rand_gen, timelimit);
	if (std::abs(res.total_time - timelimit) > 10) {
		timelimit -= res.total_time;
		fmt::print("restart with residual timelimit {}\n", timelimit);
		res = run(inst, args, env, rand_gen, timelimit);
	}

	timer.stop("total");

// -------------------------------------------------------- //

	return res;
}
