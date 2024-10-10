#pragma  once
#include <filesystem>

namespace fs = std::filesystem;

struct Args {

    fs::path input_file;
    int mls_iterlimit;
    int vidal_iterlimit;
    int memlimit;
    int threads;
    int seed;
    double timelimit;
	int multi;

    Args(int argc, char* argv[]);
};