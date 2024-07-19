#include <cmath>
#include <fmt/printf.h>

#include "ModelAux.hpp"
#include "Instance.hpp"
#include "gurobi_c++.h"
#include "gurobi_c.h"
#include "mdarray.hpp"


AuxModel::AuxModel(GRBEnv& env, Instance const& inst, Args const& args) : model{env}, z{inst.horizon}, x{inst.nreq_links, inst.horizon} {

    int M = inst.nreq_links;

    // variables
    for (int t = 0; t < inst.horizon; ++t) {
        z(t) = model.addVar(0, 1, 1, GRB_BINARY, fmt::format("z^{}", t));
        for (int l = 0; l < inst.nreq_links; ++l) {
            x(l, t) = (model.addVar(0, 1, 0, GRB_BINARY, fmt::format("x_{}_{}", l, t)));
        }
    }

    // constraints frequencies
    for (int l = 0; l < inst.nreq_links; ++l) {
        for (int s = 0; s < inst.nsubperiods; ++s) {
            if (inst.frequencies(l, s) != 0) {
                GRBLinExpr expr{0};
                for (auto const t : inst.subperiods[s]) {
                    expr += x(l, t); 
                }
                model.addConstr(expr == inst.frequencies(l, s), fmt::format("13_{}_{}", l, s));
            }
        }
    }

    // constraints big-M
    for (int t = 0; t < inst.horizon; ++t) {
        GRBLinExpr expr{0};
        for (int l = 0; l < inst.nreq_links; ++l) {
            expr += x(l, t);
        } 
        model.addConstr(expr <= z(t) * M, fmt::format("14_{}", t));
    }


    model.set(GRB_IntParam_PoolSolutions, 1);
    model.set(GRB_IntParam_PoolSearchMode, 2);
    model.set(GRB_IntParam_Threads, args.threads);
}

double AuxModel::runtime() {
    return model.get(GRB_DoubleAttr_Runtime);
}

AuxResult::AuxResult(mdarray<GRBVar, 2> const& x, mdarray<GRBVar, 1> const& z) : x_val{x.dimension(0), x.dimension(1)}, services_per_day{x.dimension(1)} {

    for (int l = 0; l < x.dimension(0); ++l) {
        for (int t = 0; t < x.dimension(1); ++t) {
            x_val(l, t) = std::lrint(x(l, t).get(GRB_DoubleAttr_Xn));
            if (x_val(l, t) == 1) {
                services_per_day(t)++;
            }
        }
    }

    empty_days = x.dimension(1);
    for (int t = 0; t < x.dimension(1); ++t) {
        empty_days -= std::lrint((z(t).get(GRB_DoubleAttr_Xn)));
    }
}

AuxResult::AuxResult(int n_required_link, int horizon) : x_val{n_required_link, horizon}, services_per_day{horizon} {}

std::vector<AuxResult> AuxModel::optimize() {

    int nsols = model.get(GRB_IntParam_PoolSolutions);
    std::vector<AuxResult> results;
    results.reserve(nsols);

    model.optimize();
    int status = model.get(GRB_IntAttr_Status);
    if (status != GRB_OPTIMAL) {
        throw std::runtime_error("No feasible solution found");
    }
    auto solutions = model.get(GRB_IntAttr_SolCount);
    for (int s = 0; s < solutions; s++) {
        model.set(GRB_IntParam_SolutionNumber, s);
        auto res = AuxResult(x, z);
        results.push_back(res);
    }

    return results;
}

