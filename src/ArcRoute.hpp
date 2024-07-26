#pragma once 

#include <vector>
#include "Instance.hpp"
#include "mdarray.hpp"
#include "unordered_set"

struct ArcRoute {

    int cost{0};
    int vehicle{-1};
    int period{-1};
	std::unordered_set<int> mipStart;

    std::vector<std::pair<int, int>> full_path;
    mdarray<int, 1> _links;

    explicit ArcRoute(Instance const& inst, ArcRoute const& other, int start, int end);
    explicit ArcRoute(Instance const& inst, std::vector<std::pair<int,int>> const& vidal_route, int veh, int t, bool original);

	void insert_links(Instance const& inst, std::vector<std::pair<int, int>> vecLinks, int bestNode, int pos);
	void remove_links(Instance const& inst, int fromLink, int toLink);
    bool contains(int id) const;
    int& links(int id);
    int links(int id) const;
    void compute_full_path(Instance const& inst);
    void check_cost(Instance const& inst) const;
};

std::vector<ArcRoute> split_route_at_depot(Instance const& inst, ArcRoute const& other);

void check_routes(Instance const& inst, std::vector<ArcRoute> const& routes);
void check_routes(std::vector<int> const& link_to_visit, std::vector<ArcRoute> const& routes);
void print_routes(Instance const& inst, std::string const& file_name, std::vector<ArcRoute>& routes);