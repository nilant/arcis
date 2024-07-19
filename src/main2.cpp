#include <filesystem>
#include <fmt/printf.h>

#include "Instance.hpp"
#include "solver_test.hpp"

#include "cli.hpp"


int main(int argc, char* argv[]) {

    Args args{argc, argv};

    if (!fs::exists(args.input_file) || args.input_file.extension() != ".json") {
        fmt::print("ERROR: wrong input file {}\n", args.input_file.string());
        std::exit(1);
    }

    Instance inst = read_json(args.input_file); 
    
    fmt::print("\nOptimizing {} with timelimit={}, iterlimit={}, memlimit={}, threads={}, seed={}...\n", 
                inst.name, args.timelimit, args.iterlimit, args.memlimit, args.threads, args.seed);

    route_solver_test(inst, args.timelimit, args.iterlimit);

    return 0;
}