#include "Model_RT.hpp"
#include "ArcRoute.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "mdarray.hpp"

#include <fmt/core.h>
#include <stdexcept>
#include <set>


RTResult::RTResult(mdarray<GRBVar, 2> const& y, mdarray<GRBVar, 2> const& x,
                   std::vector<ArcRoute> const& routes, int obj, double runtime) : y_val{y.dimension(0),
                                                                                         y.dimension(1)},
                                                                                   x_val{x.dimension(0),
                                                                                         x.dimension(1)}{

	try{
		for(int r = 0; r < y_val.dimension(0); ++r){
			int t = routes[r].period;
			y_val(r, t) = std::lrint(y(r, t).get(GRB_DoubleAttr_X));
		}

		for(int l = 0; l < x_val.dimension(0); ++l){
			for(int t = 0; t < x_val.dimension(1); ++t){
				x_val(l, t) = std::lrint(x(l, t).get(GRB_DoubleAttr_X));
			}
		}
	}catch(GRBException& e){
		fmt::print("Error code={}\n", e.getErrorCode());
		fmt::print("Error message={}\n", e.getMessage());
		std::exit(1);
	}
	cost = obj;
	time = runtime;
}

RTModel::RTModel(GRBEnv& env, Instance const& inst, std::vector<ArcRoute> const& routes, int multi) : model{env},
                                                                                           x{inst.nreq_links,
                                                                                             inst.horizon},
                                                                                           y{routes.size(),
                                                                                             inst.horizon}{

	try{
		model.set(GRB_StringAttr_ModelName, "RTModel");
		model.set(GRB_DoubleParam_TimeLimit, 600);
		model.set(GRB_IntParam_Threads, 1);
		// model.set(GRB_IntParam_MIPFocus, 1);

		// y's variables

		for(int r = 0; r < routes.size(); ++r){
			int t = routes[r].period;
			y(r, t) = model.addVar(0, 1, routes[r].cost, GRB_BINARY, fmt::format("y_{}_{}", r, t));
			if(routes[r].mipStart)
				y(r, t).set(GRB_DoubleAttr_Start, 1.0);

		}

		// x's variables
		for(int l = 0; l < inst.nreq_links; ++l){
			for(int t = 0; t < inst.horizon; ++t){
				int u = inst.links[l].first;
				int v = inst.links[l].second;
				if(inst.t_l_matrix(t,l)){
					x(l, t) = model.addVar(0, 1, (inst.serv_cost(u, v) - inst.trav_cost(u, v)), GRB_BINARY,
					                       fmt::format("x_{}_{}", l, t));
				}
			}
		}

		GRBLinExpr expr{0};
		for(int l = 0; l < inst.nreq_links; ++l){
			for(int s = 0; s < inst.nsubperiods; ++s){
				if(inst.frequencies(l, s) != 0){
					for(int t: inst.subperiods[s]){
						expr += x(l, t);
					}
					model.addConstr(expr == inst.frequencies(l, s), fmt::format("6_{}_{}", l, s));
					expr.clear();
				}
			}
		}

		for(int l = 0; l < inst.nreq_links; ++l){
			for(int t = 0; t < inst.horizon; ++t){
				if(inst.t_l_matrix(t,l)){
					for(int r = 0; r < routes.size(); ++r){
						if(routes[r].contains(l) && (t == routes[r].period)){
							expr += y(r, t);
						}
					}
					model.addConstr(x(l, t) <= expr, fmt::format("7_{}_{}", l, t));
					expr.clear();
				}
			}
		}

		/*if(multi){
			for(int r = 0; r <routes.size(); r++){
				int t = routes[r].period;
				int bigM = 0;
				for(int l = 0; l < inst.nreq_links; ++l){
					if(routes[r].contains(l)){
						expr += inst.demand(inst.links[l].first, inst.links[l].second)*x(l,t);
						bigM += inst.demand(inst.links[l].first, inst.links[l].second);
					}
				}
				for(int rr = 0; rr <routes.size(); rr++){
					int tt = routes[rr].period;
					if(t == tt){
						for(int l = 0; l < inst.nreq_links; ++l){
							if(routes[r].contains(l) && routes[rr].contains(l)){
								expr -= std::min(inst.capacity, bigM)*y(rr,t);
								break;
							}
						}
					}
				}
				model.addConstr(expr <= 0, fmt::format("r_capacity_{}_{}", r, t));
				expr.clear();
			}
			std::vector<GRBLinExpr> vec_expr(inst.horizon);
			for(int r = 0; r < routes.size(); ++r){
				int t = routes[r].period;
				vec_expr[t] -= inst.capacity*y(r, t);
			}
			for(int t = 0; t < inst.horizon; ++t){
				for(int l = 0; l < inst.nreq_links; ++l){
					if(inst.t_l_matrix(t, l)){
						int u = inst.links[l].first;
						int v = inst.links[l].second;
						vec_expr[t] += inst.demand(u, v)*x(l, t);
						expr += inst.demand(u, v)*x(l, t);
					}
				}
				model.addConstr(vec_expr[t] <= 0, fmt::format("capacity_{}", t));
				expr.clear();
			}
		}*/
		// model.update();

	}catch(GRBException& e){
		fmt::print("Error code={}\n", e.getErrorCode());
		fmt::print("Error message={}\n", e.getMessage());
		std::exit(1);
	}
}

bool RTModel::check_feasibility(std::vector<ArcRoute> const& routes, Instance const& inst){

	std::vector link_choices(inst.horizon, std::vector<std::pair<int, std::set<int>>>(inst.nreq_links));
	std::map<int, int> residual_capacity;
	for(int r = 0; r < routes.size(); ++r){
		int t = routes[r].period;
		if(y(r, t).get(GRB_DoubleAttr_Xn) > 0.5){
			residual_capacity[r] = inst.capacity;
			for(int l=0; l<inst.nreq_links; l++){
				if(routes[r].contains(l) && x(l, t).get(GRB_DoubleAttr_Xn) > 0.5){
					link_choices[t][l].first = l;
					link_choices[t][l].second.insert(r);
				}
			}
		}
	}

	for (auto& vec : link_choices) {
		std::sort(vec.begin(), vec.end(), [](const std::pair<int, std::set<int>>& a, const std::pair<int, std::set<int>>& b) {
			return a.second.size() < b.second.size();
		});
	}

	for(int t = 0; t < inst.horizon; t++){
		for(const auto& link_rset : link_choices[t]){
			if(!link_rset.second.empty()){
				int best_r = -1;
				int max_res = -std::numeric_limits<int>::max();
				for(auto r: link_rset.second){
					if(residual_capacity[r] > max_res){
						best_r = r;
						max_res = residual_capacity[r];
					}
				}
				int u = inst.links[link_rset.first].first;
				int v = inst.links[link_rset.first].second;
				int demand = inst.demand(u, v);
				residual_capacity[best_r] -= demand;
				if(residual_capacity[best_r] < 0)
					return false;
			}
		}
	}
	return true;
}

RTResult RTModel::optimize(std::vector<ArcRoute> const& routes, Instance const& inst, int multi){

	int cost{-1};
	try{

		timer.start("gurobi");
		model.optimize();
		timer.stop("gurobi");

		int nsol = model.get(GRB_IntAttr_SolCount);
		int status = model.get(GRB_IntAttr_Status);

		if(status == GRB_INFEASIBLE){
			model.computeIIS();
			model.write("model.ilp");
			throw std::runtime_error("No feasible solution found\n");
		}else if(nsol > 0){
			cost = std::lrint(model.get(GRB_DoubleAttr_ObjVal));
			if(multi){
				for(int sol = 0; sol < nsol; sol++){
					model.set(GRB_IntParam_SolutionNumber, sol);
					bool isFeasible = check_feasibility(routes, inst);
					std::cout << "solution " << sol;
					if(!isFeasible)
						std::cout << " is not feasible!" << std::endl;
					else
						std::cout << " is feasible!" << std::endl;
				}
			}
		}else{
			throw std::runtime_error("No solution found\n");
		}

	}catch(GRBException& e){
		fmt::print("Error code={}\n", e.getErrorCode());
		fmt::print("Error message={}\n", e.getMessage());
		std::exit(1);
	}
	return RTResult(y, x, routes, cost, timer.duration("gurobi"));
}
