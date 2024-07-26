#include "ModelRT.hpp"
#include "ArcRoute.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "mdarray.hpp"

#include <fmt/core.h>

RTResult::RTResult(Instance const& inst, mdarray<GRBVar, 2> const& y, mdarray<GRBVar, 2> const& x, 
                   std::vector<ArcRoute> const& routes, int obj, double time) : y_val{y.dimension(0), y.dimension(1)}, x_val{x.dimension(0), x.dimension(1)} {

    for (int r = 0; r < y_val.dimension(0); ++r) {
        int t = routes[r].period;
		y_val(r, t) = std::lrint(y(r, t).get(GRB_DoubleAttr_X));
    }

    for (int l = 0; l < x_val.dimension(0); ++l) {
        for (int t = 0; t < x_val.dimension(1); ++t) {
            x_val(l, t) = std::lrint(x(l, t).get(GRB_DoubleAttr_X));
        }
    }

    cost = obj;
    runtime = time;
}

RTResult::RTResult(Instance const& inst, mdarray<GRBVar, 2> const& y, mdarray<GRBVar, 2> const& x,
                   std::vector<ArcRoute> const& routes, int obj, double time, bool uno) : y_val{y.dimension(0), y.dimension(1)}, x_val{x.dimension(0), x.dimension(1)} {

	for (int r = 0; r < y_val.dimension(0); ++r) {
		int t = routes[r].period;
		y_val(r, t) = std::lrint(y(r, t).get(GRB_DoubleAttr_Xn));
	}

	for (int l = 0; l < x_val.dimension(0); ++l) {
		for (int t = 0; t < x_val.dimension(1); ++t) {
			x_val(l, t) = std::lrint(x(l, t).get(GRB_DoubleAttr_Xn));
		}
	}

	cost = obj;
	runtime = time;
}

RTModel::RTModel(GRBEnv& env, Instance const& inst, Args const& args, std::vector<ArcRoute> const& routes) : model{env}, 
                                                                                           x{inst.nreq_links, inst.horizon},
                                                                                           y{routes.size(), inst.horizon} {
    try {
        model.set(GRB_StringAttr_ModelName, "RTModel");                                                                                 

        // y's variables
        for (int r = 0; r < routes.size(); ++r) {
            int t = routes[r].period;
                    y(r, t) = model.addVar(0, 1, 0, GRB_BINARY, fmt::format("y_{}_{}", r, t));
					if(routes[r].mipStart.contains(t))
						y(r, t).set(GRB_DoubleAttr_Start, 1.0);

        }

        // x's variables
        for (int l = 0; l < inst.nreq_links; ++l) {
            for (int t = 0; t < inst.horizon; ++t) {
                x(l, t) = model.addVar(0, 1, 0, GRB_BINARY, fmt::format("x_{}_{}", l, t));
            }
        }

        //(5)
        GRBLinExpr expr{0};
        for (int t = 0; t < inst.horizon; ++t) {
            for (int r = 0; r < routes.size(); ++r)
				expr += routes[r].cost * y(r, t);
            for (int l = 0; l < inst.nreq_links; ++l) {
                int u = inst.links[l].first;
                int v = inst.links[l].second;
                expr += (inst.serv_cost(u, v) - inst.trav_cost(u, v)) * x(l, t);
            }
        }
        model.setObjective(expr, GRB_MINIMIZE);
	    expr.clear();

        //(6)
        for (int l = 0; l < inst.nreq_links; ++l) {
            for (int s = 0; s < inst.nsubperiods; ++s) {
                if (inst.frequencies(l, s) != 0) {
                    for (int t : inst.subperiods[s]) {
                        expr += x(l, t);
                    }
                    model.addConstr(expr == inst.frequencies(l, s), fmt::format("6_{}_{}", l, s));
	                expr.clear();
                }
            }
        }

        //(7)
        for (int l = 0; l < inst.nreq_links; ++l) {
            for (int t = 0; t < inst.horizon; ++t) {
                for (int r = 0; r < routes.size(); ++r) {
                        if (routes[r].contains(l)) {
                            expr += y(r, t);
                        }
                }
                model.addConstr(x(l, t) <= expr, fmt::format("7_{}_{}", l, t));
	            expr.clear();
            }
        }

        // //(8)
        // for (int t = 0; t < inst.horizon; ++t) {
        //     GRBLinExpr expr{0};
        //     for (int r = 0; r < routes.size(); ++r) {
        //         expr += y(r, t);
        //     }
        //     model.addConstr(expr <= 4000, fmt::format("8_{}", t));
        // }

        // //(9)
        // for (int t = 0; t < inst.horizon; ++t) {
        //     GRBLinExpr expr1{0};
        //     GRBLinExpr expr2{0};
        //     for (int l = 0; l < inst.nreq_links; ++l) {
        //         int u = inst.links[l].first;
        //         int v = inst.links[l].second;
        //         expr1 += inst.demand(u, v) * x(l, t);
        //     }
        //     for (int r = 0; r < routes.size(); ++r) {
        //         expr2 += y(r, t);
        //     }
        //     model.addConstr(expr1 <= inst.quantity * expr2, fmt::format("9_{}", t));
        // }
    
        model.set(GRB_IntParam_Threads, args.threads);
		// model.update();

    } catch (GRBException& e) {
        fmt::print("Error code={}\n", e.getErrorCode());
        fmt::print("Error message={}\n", e.getMessage());
        std::exit(1);
    }
}

RTResult RTModel::optimize(Instance const& inst, std::vector<ArcRoute> const& routes) {
    
    int cost;
    try {
        model.optimize();

        int status = model.get(GRB_IntAttr_Status);
        if (status == GRB_INFEASIBLE) {
            model.computeIIS();
            model.write("model.ilp");
            throw std::runtime_error("No feasible solution found");
        }

        if (status == GRB_OPTIMAL) { 
            cost = std::lrint(model.get(GRB_DoubleAttr_ObjVal));
        }

    } catch (GRBException& e) {
        fmt::print("Error code={}\n", e.getErrorCode());
        fmt::print("Error message={}\n", e.getMessage());
        std::exit(1);
    }

	int nSol = model.get(GRB_IntAttr_SolCount);
	if(nSol > 1){
		double obj = model.get(GRB_DoubleAttr_ObjVal);
		model.set(GRB_IntParam_SolutionNumber,1);
		if(model.get(GRB_DoubleAttr_PoolObjVal) > obj + 0.5)
			model.set(GRB_IntParam_SolutionNumber,0);
		else
			return RTResult(inst, y, x, routes, cost, this->runtime(), true);
	}
    
    return RTResult(inst, y, x, routes, cost, this->runtime());
}

double RTModel::runtime() {
    return model.get(GRB_DoubleAttr_Runtime);
}