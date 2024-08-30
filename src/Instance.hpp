# pragma once

#include <filesystem>
#include <vector>
#include <random>

#include "../libs/json.hpp"
#include "mdarray.hpp"

namespace fs = std::filesystem;
using json = nlohmann::json;

enum LinkType {NONE, EDGE, ARC};

struct Instance {
    std::string name;
    int tot_services;
    int horizon;
    int nvertices;
    int nreq_links;
    int nnot_req_links;
    int nlinks;
    int nsubperiods;
    // int nvehicles;
    int capacity;
    
    std::vector<std::pair<int, int>> links;
    mdarray<int, 2> required; 
    mdarray<LinkType, 2> type;
    mdarray<int, 2> id;
    mdarray<int, 2> serv_cost;
    mdarray<int, 2> trav_cost;
    mdarray<int, 2> demand;
	mdarray<int, 2> t_l_matrix;

    //(nreq_links, nsubperiods)
    mdarray<int, 2> frequencies;

    std::vector<std::vector<int>> subperiods;
    mdarray<int, 2> sp_matrix;

    mdarray<int, 2> dist;
    mdarray<int, 2> prev;

	void generateRandomDemand();

    explicit Instance(int nvertices, int nreq_links, int nsubperiods, int horizon, int nlinks);
};

Instance read_json(fs::path input_path);