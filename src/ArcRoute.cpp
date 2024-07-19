#include <algorithm>
#include <numeric>
#include <fmt/core.h>
#include <iterator>
#include <fstream>

#include "ArcRoute.hpp"
#include "Instance.hpp"


std::vector<std::pair<int,int>> build_path(mdarray<int, 2> const& prev, int u, int v) {
    std::vector<std::pair<int, int>> path;
    int prev_v = v;
    while (u != v) {
        v = prev(u, v);
        path.insert(path.begin(), {v, prev_v});
        prev_v = v;
    }
    return path;
}

std::vector<ArcRoute> split_route_at_depot(Instance const& inst, ArcRoute const& other) {
    std::vector<ArcRoute> routes;
    int start = 0;
    for (int end = 0; end < other.full_path.size(); ++end) {
        int u = other.full_path[end].first;
        int v = other.full_path[end].second;
        if (v == 0) {
            routes.push_back(ArcRoute(inst, other, start, end));
            start = end+1;
        }
    } 

    return routes;
}

ArcRoute::ArcRoute(Instance const& inst, ArcRoute const& other, int start, int end) : _links{inst.nlinks} {
    cost = 0;
    original = other.original;
    subperiod = other.subperiod;
    vehicle = other.vehicle;

    std::copy(other.full_path.begin() + start, other.full_path.begin() + (end + 1), std::back_inserter(full_path));
    for (auto const [i, j] : full_path) {
        cost += inst.trav_cost(i, j);
        links(inst.id(i, j)) = true;
    }
}

ArcRoute::ArcRoute(Instance const& inst, std::vector<std::pair<int,int>> const& route, int veh, int sp, bool ori) : _links{inst.nlinks} {

    original = ori;
    subperiod = sp;

	for (int l = 0; l < _links.dimension(0); ++l) {
		links(l) = false;
	}

    vehicle = veh;
    cost = 0;
    int prev_v = 0;
    full_path.reserve(inst.nlinks);
    
    for (int i = 0; i < route.size(); ++i) {
        int u = route[i].first-1;       // -1 because vidal nodes starts from 1
        int v = route[i].second-1;

        auto path = build_path(inst.prev, prev_v, u);
        for (auto const& [i, j] : path) {
            full_path.push_back({i, j});
            cost += inst.trav_cost(i, j);
            links(inst.id(i, j)) = true;
        }
        if (u != 0 || v != 0) {
            full_path.push_back({u, v});
            cost += inst.serv_cost(u, v);
            links(inst.id(u, v)) = true;
        }

        prev_v = v;
    }
}

int& ArcRoute::links(int l) {
    assert(l >= 0 && l < _links.size());
    return _links(l);
}

int ArcRoute::links(int l) const {
    assert(l >= 0 && l < _links.size());
    return _links(l);
}

void ArcRoute::insert_link(Instance const& inst, std::pair<int, int> link, int pos) {
    
    int u = link.first;
    int v = link.second;

    full_path.insert(full_path.begin() + pos, {u, v});
    int prev = pos == 0 ? 0 : full_path[pos - 1].second;
    int post = pos == full_path.size()-1 ? 0 : full_path[pos + 1].first;

    links(inst.id(u, v)) = true;

    cost += inst.serv_cost(u, v);

    auto path = build_path(inst.prev, prev, u);
    int path_length = path.size();
    full_path.insert(full_path.begin() + pos, path.begin(), path.end());
    for (auto const [i, j] : path) {
        cost += inst.trav_cost(i, j);
        links(inst.id(i, j)) = true;
    }

    path = build_path(inst.prev, v, post);
    full_path.insert(full_path.begin() + (pos + path_length + 1), path.begin(), path.end());
    for (auto const [i, j] : path) {
        cost += inst.trav_cost(i, j);
        links(inst.id(i,j)) = true;
    }
}

void ArcRoute::remove_link(Instance const& inst, std::pair<int, int> link) {

    int u = link.first;
    int v = link.second;

    auto it = std::find_if(full_path.begin(), full_path.end(), [&] (auto a) { return a.first == u && a.second == v; });
    int pos = std::distance(full_path.begin(), it);

    int prev = pos == 0 ? 0 : full_path[pos - 1].second;
    int post = pos == full_path.size()-1 ? 0 : full_path[pos + 1].first;

    full_path.erase(it);
    cost -= inst.serv_cost(u, v);
    links(inst.id(u, v)) = false;

    auto path = build_path(inst.prev, prev, post);
    full_path.insert(full_path.begin() + pos, path.begin(), path.end());
    for (auto const [i, j] : path) {
        cost += inst.trav_cost(i, j);
        links(inst.id(i, j)) = true;
    } 
}

    void ArcRoute::check_cost(Instance const& inst) const {
    
    int new_cost = 0;
    int prev_j = 0;
    for (auto const [i, j] : full_path) {
        new_cost += inst.trav_cost(i, j);
    }

    assert(new_cost == cost);
}

bool ArcRoute::contains(int id) const {
	return links(id);
}

void check_routes(Instance const& inst, std::vector<ArcRoute> const& routes) {
    std::vector<int> links(inst.nreq_links);
    std::iota(links.begin(), links.end(), 0);
    check_routes(links, routes);
}

void check_routes(std::vector<int> const& link_to_visit, std::vector<ArcRoute> const& routes) {
    for (int const l : link_to_visit) {
        bool flag = false;
        for (auto const& route : routes) {
            if (route.contains(l)) {
                flag = true;
            }
        }
        assert(flag);
     }
}

void print_routes(Instance const& inst, const std::string &file_name, std::vector<ArcRoute> &routes) {
    std::string print = "";
    for (auto const& route : routes) {
        std::string list_of_links = fmt::format("{} [", route.subperiod);
        for (auto const& link : route.full_path) {
            int u = link.first;
            int v = link.second;
            if (inst.type(u, v) == ARC && inst.required(u, v)) {
                list_of_links += fmt::format("({},{}), ", u, v);
            } else if (inst.type(u, v) == EDGE) {
                if (inst.required(u, v)) {
                    list_of_links += fmt::format("({},{}), ", u, v);    
                } else if (inst.required(v, u)) {
                    list_of_links += fmt::format("({},{}), ", v, u);  
                }
            }
        }
        list_of_links += "]\n";
        print += list_of_links;
    }

    std::ofstream file_ss{file_name};
    file_ss << print << std::endl;
}