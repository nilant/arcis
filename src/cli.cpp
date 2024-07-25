#include "cli.hpp"
#include "../libs/argh.h"

Args::Args(int argc, char* argv[]) {
    argh::parser cmdl({"--timelimit", "--memlimit", "--iterlimit",
                       "--threads", "--seed"});
    cmdl.parse(argc, argv, argh::parser::SINGLE_DASH_IS_MULTIFLAG);

    input_file = fs::path(cmdl[1]);
    cmdl("--timelimit", 3600) >> timelimit;
    cmdl("--iterlimit", 500) >> iterlimit;
    cmdl("--memlimit", 0) >> memlimit;
    cmdl("--threads", 1) >> threads;
    cmdl("--seed", 42) >> seed;
}