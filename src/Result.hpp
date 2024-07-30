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

    double gurobi_time{0};
    double total_time{0};

    double ls_time{0};
    int ls_iter{0}; // total iter of all restart

    double best_time_ls{0}; // best for all resatrt
    int best_iter_ls{0}; // best for all resatrt

    int nroutes{0};
    int lb{0};

    int restart{0};
    int best_restart{0};
    int total_restart{0};

    void write_json(fs::path const& file_path); 
    void print();
};