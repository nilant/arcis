#include "solver_test.hpp"
#include "Preprocessing.hpp"
#include "RouteSolver.hpp"

#include <fstream>
#include <fmt/core.h>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

using namespace std;

// Function to parse a single line of the input string
vector<pair<int, int>> parseLine(const string& input) {
    vector<pair<int, int>> result;
    string cleanedInput = input.substr(input.find('{') + 1, input.size() - input.find('{') - 3); // Extract the part within {}

    stringstream ss(cleanedInput);
    string item;
    
    while (getline(ss, item, '(')) {
        if (item.find(')') != string::npos) {
            stringstream pairStream(item.substr(0, item.find(')')));
            string first, second;
            getline(pairStream, first, ',');
            getline(pairStream, second, ',');
            
            int num1 = stoi(first) - 1;
            int num2 = stoi(second) - 1;
            
            result.emplace_back(num1, num2);
        }
    }
    
    return result;
}


struct ParsedFile {
    vector<vector<pair<int, int>>> all_pairs;
    double cost;
    double time;
};

ParsedFile parsefile(std::string input) {
    ifstream infile(input);
    string line;
    vector<vector<pair<int, int>>> allPairs;

    if (!infile) {
        cerr << "Unable to open file input.txt";
        exit(1);   // Call system to stop
    }

    getline(infile, line);
    double cost = stof(line);

    getline(infile, line);
    double time = stof(line);

    while (getline(infile, line)) {
            vector<pair<int, int>> pairs = parseLine(line);
            allPairs.push_back(pairs);
    }

    infile.close();

    ParsedFile pf;
    pf.all_pairs = allPairs;
    pf.cost = cost;
    pf.time = time;

    return pf;
}

CarpInstance create_carp_instance(Instance const& inst, std::vector<std::pair<int, int>>& links) {

    CarpInstance carp{inst.nvehicles};

    for (auto const& link : links) {
        int u = link.first;
        int v = link.second;

        carp.link_to_visit.insert(inst.id(u, v));
    }
    return carp;
}

int route_solver_test(const Instance &inst, int timelimit, int iterlimit) {

    std::string input_file = "data/CleanDemSol/DemSol_" + inst.name + ".txt";
    auto pf = parsefile(input_file);

    int vidal_cost = 0;
    for (int i = 0; i < pf.all_pairs.size(); ++i) {
        CarpInstance carp = create_carp_instance(inst, pf.all_pairs[i]);

        auto solver = RouteSolver{};

        int timelimit = static_cast<int>(std::ceil(pf.time / pf.all_pairs.size()));
		auto routes = solver.solve_routes(inst, 0, carp, timelimit, iterlimit);

        for (auto const& route : routes) {
			vidal_cost += route.cost;
		}
    }

    fmt::print("dem_obj={}, dem_time={}, vidal_obj={}\n", pf.cost, pf.time, vidal_cost);

    return vidal_cost;
}
