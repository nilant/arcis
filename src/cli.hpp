#pragma  once
#include <filesystem>

namespace fs = std::filesystem;

struct Args {

    fs::path input_file;
    int ls_iterlimit;
    int vidal_iterlimit;
    int timelimit;
    int memlimit;
    int threads;
    int seed;

    Args(int argc, char* argv[]);
};