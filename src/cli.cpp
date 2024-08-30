#include "cli.hpp"
#include "../libs/argh.h"

Args::Args(int argc, char* argv[]) {
    argh::parser cmdl({"--timelimit", "--memlimit", "--vidal_iterlimit",
                       "ls_iterlimit", "--threads", "--seed"});
    cmdl.parse(argc, argv, argh::parser::SINGLE_DASH_IS_MULTIFLAG);

    input_file = fs::path(cmdl[1]);
    cmdl("--timelimit", 900) >> timelimit;
    cmdl("--vidal_iterlimit", 1000) >> vidal_iterlimit;
    cmdl("--ls_iterlimit", 1) >> ls_iterlimit;
    cmdl("--memlimit", 0) >> memlimit;
    cmdl("--threads", 1) >> threads;
    cmdl("--seed", 0) >> seed;
	cmdl("--multi", 1) >> multi;
}