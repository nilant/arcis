#include <fstream>
#include <iomanip>
#include <iostream>
#include <fmt/printf.h>

#include "Result.hpp"

void Result::write_json(fs::path const& file_path) {
    
    std::ifstream file(file_path);
    auto j = nlohmann::ordered_json::parse(file);

    std::ofstream file_ss{file_path};
    auto& out = file_path.empty() ? std::cout : file_ss;

    out << std::setprecision(2) << std::setw(4) << std::fixed;

    nlohmann::ordered_json jsol;
    jsol["vidal_obj"] = vidal_obj;
    jsol["rt_obj"] = rt_obj;
    jsol["ls_obj"] = ls_obj;
    jsol["vidal_time"] = vidal_time;
    jsol["rt_time"] = rt_time;
    jsol["ls_time"] = ls_time;
    jsol["gurobi_time"] = gurobi_time;
    jsol["total_time"] = total_time;
    jsol["nroutes"] = nroutes;
    jsol["iter_ls"] = iter_ls;
    jsol["time_ls"] = time_ls;
    jsol["best_time_ls"] = best_time_ls;
    jsol["best_iter_ls"] = best_iter_ls;
    jsol["lb"] = lb;

    jsol["best_restart"] = best_restart;

    j[name] = jsol;

    out << j << std::endl;
}

void Result::print() {
    fmt::print("pre_obj={}, post_obj={}, best_obj={}, runtime={}\n", vidal_obj, rt_obj, ls_obj, total_time);
}
