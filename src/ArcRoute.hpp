#pragma once 

#include <vector>
#include "Instance.hpp"
#include "mdarray.hpp"

struct ArcRoute {

    int cost{0};
    int vehicle{-1};
    int subperiod{-1};
    bool original{false};

    std::vector<std::pair<int, int>> full_path;
    mdarray<int, 1> _links;

    explicit ArcRoute(Instance const& inst, ArcRoute const& other, int start, int end);
    explicit ArcRoute(Instance const& inst, std::vector<std::pair<int,int>> const& vidal_route, int veh, int sp, bool original);
    void insert_link(Instance const& inst, std::pair<int, int> link, int pos);
    void remove_link(Instance const& inst, std::pair<int, int> link);
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