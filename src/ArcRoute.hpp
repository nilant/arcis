#pragma once 

#include <vector>
#include <set>
#include "Instance.hpp"
#include "mdarray.hpp"
#include "unordered_set"

struct ArcRoute {

    int cost{0};
    int period{-1};
	bool mipStart{false};

    std::vector<std::pair<int, int>> full_path;
    mdarray<int, 1> _links;
	std::set<int> visited_rlink;

    explicit ArcRoute(Instance const& inst, ArcRoute const& other, int start, int end);
    explicit ArcRoute(Instance const& inst, std::vector<std::pair<int,int>> const& vidal_route, int t);
	explicit ArcRoute(Instance const& inst, std::vector<std::pair<int,int>> const& my_route, int t, bool dummy);

	void insert_links(Instance const& inst, std::vector<std::pair<int, int>> vecLinks, int bestNode, int pos);
	void remove_links(Instance const& inst, int fromLink, int toLink);
    bool contains(int id) const;
    int& links(int id);
    int links(int id) const;
    void check_cost(Instance const& inst) const;
};

std::vector<ArcRoute> split_route_at_depot(Instance const& inst, ArcRoute const& other);

void check_routes(Instance const& inst, std::vector<ArcRoute> const& routes);
void check_routes(std::vector<int> const& link_to_visit, std::vector<ArcRoute> const& routes);
void print_routes(Instance const& inst, std::string const& file_name, std::vector<ArcRoute>& routes);