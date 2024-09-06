//
// Created by Paolo on 03/09/2024.
//
#include "ModelRT.hpp"
#include "ArcRoute.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "mdarray.hpp"

#include <fmt/core.h>
#include <stdexcept>

RTResult::RTResult(mdarray<GRBVar, 2> const& y, mdarray<GRBVar, 3> const& x,
                   std::vector<ArcRoute> const& routes, int obj, double runtime) : y_val{y.dimension(0),
                                                                                         y.dimension(1)},
                                                                                   x_val{x.dimension(0),
                                                                                         x.dimension(2)}{

	try{
		for(int r = 0; r < y_val.dimension(0); ++r){
			int t = routes[r].period;
			y_val(r, t) = std::lrint(y(r, t).get(GRB_DoubleAttr_X));
		}

		for(int r = 0; r < routes.size(); ++r){
			int t = routes[r].period;
			for(int l = 0; l < x_val.dimension(0); ++l)
				if(routes[r].contains(l))
					x_val(l, t) = (int) fmax(x_val(l, t), std::lrint(x(l, r, t).get(GRB_DoubleAttr_X)));
		}
	}catch(GRBException& e){
		fmt::print("Error code={}\n", e.getErrorCode());
		fmt::print("Error message={}\n", e.getMessage());
		std::exit(1);
	}
	cost = obj;
	time = runtime;
}

mRTModel::mRTModel(GRBEnv& env, Instance const& inst, std::vector<ArcRoute> const& routes) : model{env},
                                                                                           x{inst.nreq_links,
																							 routes.size(),
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
		for(int r = 0; r < routes.size(); ++r){
			int t = routes[r].period;
			for(int l = 0; l < inst.nreq_links; ++l){
				if(routes[r].contains(l)){
					int u = inst.links[l].first;
					int v = inst.links[l].second;
					x(l, r, t) = model.addVar(0, 1, (inst.serv_cost(u, v) - inst.trav_cost(u, v)), GRB_BINARY,
					                          fmt::format("x_{}_{}_{}", l, r, t));
					model.addConstr(x(l, r, t) <= y(r, t),
					                fmt::format("xyLink_{}_{}_{}", l, r, t));
				}
			}
		}


		GRBLinExpr expr{0};
		for(int l = 0; l < inst.nreq_links; ++l){
			for(int s = 0; s < inst.nsubperiods; ++s){
				if(inst.frequencies(l, s) > 0){
					for(int r = 0; r < routes.size(); ++r){
						int t = routes[r].period;
						if(routes[r].contains(l) && inst.sp_matrix(s, t))
							expr += x(l, r, t);
					}
					model.addConstr(expr == inst.frequencies(l, s), fmt::format("freq_{}_{}", l, s));
					expr.clear();
				}
			}
		}
		for(int l = 0; l < inst.nreq_links; ++l){
			for(int s = 0; s < inst.nsubperiods; ++s){
				if(inst.frequencies(l, s) > 0){
					for(int r = 0; r < routes.size(); ++r){
						int t = routes[r].period;
						if(routes[r].contains(l) && inst.sp_matrix(s, t))
							expr += y(r, t);
					}
					model.addConstr(expr >= inst.frequencies(l, s), fmt::format("r_freq_{}_{}", l, s));
					expr.clear();
				}
			}
		}

		std::vector<std::vector<GRBLinExpr>> vec_expr(inst.horizon, std::vector<GRBLinExpr>(inst.nreq_links));
		for(int r = 0; r < routes.size(); ++r){
			int t = routes[r].period;
			for(int l = 0; l < inst.nreq_links; ++l)
				if(routes[r].contains(l))
					vec_expr[t][l] += x(l, r, t);
		}
		for(int t = 0; t < inst.horizon; ++t){
			for(int l = 0; l < inst.nreq_links; ++l){
				model.addConstr(vec_expr[t][l] <= 1, fmt::format("oneVisit_{}_{}", l, t));
			}
		}

		for(int r = 0; r < routes.size(); ++r){
			int t = routes[r].period;
			int totDem = 0;
			for(int l = 0; l < inst.nreq_links; ++l){
				if(routes[r].contains(l)){
					int u = inst.links[l].first;
					int v = inst.links[l].second;
					expr += inst.demand(u, v)*x(l, r, t);
					totDem += inst.demand(u, v);
				}
			}
			if(totDem > inst.capacity){
				model.addConstr(expr <= std::min(totDem, inst.capacity)*y(r, t), fmt::format("capacity_{}_{}", r, t));
			}
			expr.clear();
		}
		// model.update();

	}catch(GRBException& e){
		fmt::print("Error code={}\n", e.getErrorCode());
		fmt::print("Error message={}\n", e.getMessage());
		std::exit(1);
	}
}

RTResult mRTModel::optimize(std::vector<ArcRoute> const& routes){

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
			/*int tot_QTY = 0;
			for(int r = 0; r < routes.size(); ++r){
				int t = routes[r].period;
				std::cout << "period " << t << " r " << r << ": ";
				int tot_qty = 0;
				for(int l = 0; l < x.dimension(0); ++l){
					if(routes[r].contains(l)){
						if(x(l, r, t).get(GRB_DoubleAttr_X ) > 0.5){
							std::cout << model.getCoeff(model.getConstrByName(fmt::format("capacity_{}_{}", r, t)),
							                            x(l, r, t)) << " ";
							tot_qty += model.getCoeff(model.getConstrByName(fmt::format("capacity_{}_{}", r, t)),
							                          x(l, r, t));
						}
					}
				}
				std::cout << ": " << tot_qty << std::endl;
				tot_QTY += tot_qty;
			}
			std::cout << "tot_QTY: " << tot_QTY << std::endl;*/
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

