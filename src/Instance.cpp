#include <fmt/printf.h>
#include <fstream>

#include "Instance.hpp"
#include "mdarray.hpp"
#include "utils.hpp"


Instance::Instance(int nvertices, int nreq_links, int nsubperiods, int horizon, int nlinks) : required{nvertices, nvertices},
                                                                     type{nvertices, nvertices}, 
                                                                     id{nvertices, nvertices},
                                                                     serv_cost{nvertices, nvertices},
                                                                     trav_cost{nvertices, nvertices},
                                                                     demand{nvertices, nvertices},
                                                                     dist{nvertices, nvertices},
                                                                     prev{nvertices, nvertices},
                                                                     frequencies{nreq_links, nsubperiods}, 
                                                                     sp_matrix{nsubperiods, horizon},
																	 t_l_matrix{horizon, nlinks}
																	 {

    
} 


Instance read_json(fs::path input_path) {

    std::ifstream file(input_path);
    json j = json::parse(file)["instance"];
    
    std::vector<std::vector<int>> subperiods;
    for (auto const& jsubs : j["subperiods"]) {
        subperiods.push_back(std::vector<int>());
        for (auto day : jsubs) {
            subperiods[subperiods.size()-1].push_back(day);
        }
    }
    int nsubperiods = subperiods.size();


    int nvertices = j["nvertices"];
    int nreq_links = j["nreq_links"];
    int nnot_req_links = j["nnot_req_links"];
	int nlinks = nreq_links + nnot_req_links;
	int horizon = j["horizon"];


    Instance inst(nvertices, nreq_links, nsubperiods, horizon, nlinks);
    inst.nvertices = nvertices;
    inst.nreq_links = nreq_links;
    inst.nsubperiods = nsubperiods;
    inst.subperiods = subperiods;

    inst.name = j["name"];
    inst.horizon = horizon;

    inst.tot_services = j["tot_services"];
    inst.nnot_req_links = nnot_req_links;
    inst.nlinks = nlinks;

    //FIXIT
    inst.nvehicles = 1;
    inst.quantity = inst.nlinks;

    for (int s = 0; s < inst.nsubperiods; ++s) {
        for (int const t : subperiods[s]) {
            inst.sp_matrix(s, t) = true;
        }
    }

    for (int i = 0; i < nvertices; ++i) {
        for (int jj = 0; jj < nvertices; ++jj) {
            inst.required(i, jj) = false;
            inst.type(i, jj) = NONE;
            inst.id(i, jj) = -1;
            inst.serv_cost(i, jj) = (i == jj) ? 0 : INF;
            inst.trav_cost(i, jj) = (i == jj) ? 0 : INF;
            inst.demand(i, jj) = (i == jj) ? 0 : 1;
        }
    }

    inst.links.reserve(inst.nlinks);
    int i = 0;
    for (auto const& jreq_link : j["required_links"]) {
        int u = static_cast<int>(jreq_link["from_node"]) - 1;
        int v = static_cast<int>(jreq_link["to_node"]) - 1;
        inst.links.push_back({u, v});
        auto t = jreq_link["type"] == "edge" ? EDGE : ARC;
        if (t == EDGE) {
            inst.type(v, u) = t;
            inst.id(v, u) = i;
            inst.trav_cost(v, u) = jreq_link["trav_cost"];
            inst.serv_cost(v, u) = jreq_link["serv_cost"];
            inst.required(v, u) = true;

        }
        inst.type(u, v) = t;
        inst.id(u, v) = i;
        inst.trav_cost(u, v) = jreq_link["trav_cost"];
        inst.serv_cost(u, v) = jreq_link["serv_cost"];
        inst.required(u, v) = true;
        for (int f = 0; f < nsubperiods; ++f) {
            inst.frequencies(i, f) = jreq_link["frequencies"][f];
        }
        i++;
    }

	for (int s = 0; s < inst.nsubperiods; ++s) {
		for (int const t : subperiods[s]){
			for(int l = 0; l < inst.nreq_links; l++){
				inst.t_l_matrix(t, l) = 1;
			}
		}
	}

    for (auto const& jnot_req_link : j["not_required_links"]) {
        int u = static_cast<int>(jnot_req_link["from_node"]) - 1;
        int v = static_cast<int>(jnot_req_link["to_node"]) - 1;
        inst.links.push_back({u, v});
        auto t = jnot_req_link["type"] == "edge" ? EDGE : ARC;
        if (t == EDGE) {
            inst.type(v, u) = t;
            inst.id(v, u) = i;
            inst.trav_cost(v, u) = jnot_req_link["trav_cost"];
        }
        inst.type(u, v) = t;
        inst.id(u, v) = i;
        inst.trav_cost(u, v) = jnot_req_link["trav_cost"];
        inst.required(u, v) = false;
        i++;
    }

    floyd_warshall(inst.trav_cost, inst.dist, inst.prev);
    return inst;
}