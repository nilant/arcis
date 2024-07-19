#include <limits>
#include <vector>
#include <fmt/core.h>

#include "LocalSearch.hpp"
#include "ArcRoute.hpp"
#include "ModelRT.hpp"
#include "best.hpp"


ArcRoute insert(Instance const& inst, std::pair<int, int> link, ArcRoute const& route) {

    int u = link.first;
    int v = link.second;
    int l_cost = inst.serv_cost(u, v);

    int best_insert_point = 1;
    int best_cost = std::numeric_limits<int>::max();
    int new_cost = best_cost;
    for (int k = 1; k < route.full_path.size(); ++k) {
        
        int pre = route.full_path[k-1].second;
        int post = route.full_path[k].first;

        new_cost = route.cost + l_cost + inst.dist(pre, u) + inst.dist(v, post);

        if (new_cost < best_cost) {
            best_cost = new_cost;
            best_insert_point = k;
        }
    }

    ArcRoute new_route{route};
    new_route.original = false;

    new_route.insert_link(inst, link, best_insert_point);
    
    return new_route;
}

ArcRoute remove(Instance const& inst, std::pair<int, int> link, ArcRoute const& route) {

    ArcRoute new_route{route};
    new_route.original = false;
    new_route.remove_link(inst, link);

    return new_route;
} 

std::vector<ArcRoute> generate_new_routes(Instance const& inst, BestSolution const& best_sol, RTResult const& rt_res) {

    std::vector<ArcRoute> new_routes;
    for (int t = 0; t < inst.horizon; ++t) {
        for (auto const& route : best_sol.best_routes[t]) {
            new_routes.push_back(route);
    		for (auto const [i, j] : route.full_path) {
            	auto new_route = remove(inst, {i, j}, route);
                if (new_route.cost < route.cost) {
                    auto rr = split_route_at_depot(inst, new_route);
                    new_routes.insert(new_routes.begin(), rr.begin(), rr.end());
                }
                for (int s = 0; s < inst.nsubperiods; ++s) {
                    if (inst.sp_matrix(s, t)) {
                    	for (auto const tt : inst.subperiods[s]) {
                            for (auto const& second_route : best_sol.best_routes[tt]) {
                                if (second_route.contains(inst.id(i, j))) {
                                    continue;
                                }

                                auto new_route2 = insert(inst, {i, j}, second_route);
                                auto rr = split_route_at_depot(inst, new_route2);
                                new_routes.insert(new_routes.begin(), rr.begin(), rr.end());

                                if (inst.type(i, j) == EDGE) {
                                    auto new_route2 = insert(inst, {j, i}, second_route);
                                    auto rr = split_route_at_depot(inst, new_route2);
                                    new_routes.insert(new_routes.begin(), rr.begin(), rr.end());
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    return new_routes;
}

void local_search(GRBEnv& env, Instance const& inst, Args const& args, BestSolution& curr_best, RTResult& curr_rt_res, int timelimit, int iterlimit) {
    
    fmt::print("iterlimit={}, timelimit={}\n", iterlimit, timelimit);
    auto t0 = std::chrono::high_resolution_clock::now();

    double runtime = 0;
    int best_cost = curr_best.cost;
    double gurobi_time = curr_rt_res.runtime;
    int prev_cost = std::numeric_limits<int>::max();

    int iter = 0;
    int best_iter = 0;
    double best_time = 0;
    for (; iter <= iterlimit && runtime <= timelimit; ++iter) {
        auto new_routes = generate_new_routes(inst, curr_best, curr_rt_res);
        RTModel rt_model{env, inst, args, new_routes};
	    curr_rt_res = rt_model.optimize(inst, new_routes);
        curr_best = BestSolution(inst, new_routes, curr_rt_res);

        gurobi_time += curr_best.gurobi_time;

        fmt::print("iter={}, best={}\n", iter, best_cost);
        auto t1 = std::chrono::high_resolution_clock::now();
        runtime = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0; 
        
        if (curr_best.cost < best_cost) {
            best_cost = curr_best.cost;
            best_iter = iter;
            best_time = runtime;
        }
        prev_cost = best_cost;
    }

    curr_best.iter = iter;
    curr_best.time = runtime;

    curr_best.best_time = best_time;
    curr_best.best_iter = best_iter;

    curr_best.gurobi_time = gurobi_time;

    fmt::print("best_iter={}, best_cost={}\n", best_iter, best_cost);
}