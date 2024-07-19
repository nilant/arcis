#pragma  once 

#include <limits>
#include <string>
#include <filesystem>

#include "../libs/json.hpp"

using json = nlohmann::json;
namespace fs = std::filesystem;

struct Result {

    std::string name;

    double vidal_obj{std::numeric_limits<double>::max()};
    double rt_obj{std::numeric_limits<double>::max()};
    double ls_obj{std::numeric_limits<double>::max()};

    double vidal_time{0};
    double rt_time{0};
    double ls_time{0};

    double gurobi_time{0};
    double total_time{0};

    double time_ls{0};
    int iter_ls{0};

    double best_time_ls{0};
    int best_iter_ls{0};

    int nroutes{0};
    int lb{0};

    void write_json(fs::path const& file_path); 
    void print();
};