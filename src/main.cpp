#include <filesystem>
#include <fmt/printf.h>

#include "Instance.hpp"
#include "Heuristic.hpp"

#include "cli.hpp"


int main(int argc, char* argv[]) {

    Args args{argc, argv};

    if (!fs::exists(args.input_file) || args.input_file.extension() != ".json") {
        fmt::print("ERROR: wrong input file {}\n", args.input_file.string());
        std::exit(1);
    }

    Instance inst = read_json(args.input_file);
	inst.generateRandomDemand();

	// args.timelimit = inst.tot_services;
    
    fmt::print("\nOptimizing {} with timelimit={}, vidal_iterlimit={}, ls_iterlimit={}, memlimit={}, threads={}, seed={}, multi={}\n",
                inst.name, args.timelimit, args.vidal_iterlimit, args.ls_iterlimit, args.memlimit, args.threads, args.seed, args.multi);

    auto result = heur(inst, args);
    fmt::print("done\n\n");

    result.print();
    result.write_json(args.input_file);

    return 0;
}