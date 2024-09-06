#include <algorithm>
#include <numeric>
#include <fmt/core.h>
#include <iterator>
#include <fstream>

#include "ArcRoute.hpp"
#include "Instance.hpp"


std::vector<std::pair<int, int>> build_path(mdarray<int, 2> const& prev, int u, int v){
	std::vector<std::pair<int, int>> path;
	int prev_v = v;
	while(u != v){
		v = prev(u, v);
		path.insert(path.begin(), {v, prev_v});
		prev_v = v;
	}
	return path;
}

std::vector<ArcRoute> split_route_at_depot(Instance const& inst, ArcRoute const& other){
	std::vector<ArcRoute> routes;
	int start = 0;
	for(int end = 0; end < other.full_path.size(); ++end){
		int u = other.full_path[end].first;
		int v = other.full_path[end].second;
		if(v == 0){
			routes.push_back(ArcRoute(inst, other, start, end));
			start = end + 1;
		}
	}

	return routes;
}

ArcRoute::ArcRoute(Instance const& inst, ArcRoute const& other, int start, int end) : _links{inst.nlinks}{
	cost = 0;
	period = other.period;

	std::copy(other.full_path.begin() + start, other.full_path.begin() + (end + 1), std::back_inserter(full_path));
	for(auto const [i, j]: full_path){
		if(inst.required(i,j)){
			links(inst.id(i, j)) = true;
			visited_rlink.insert(inst.id(i, j));
		}
		cost += inst.trav_cost(i, j);
	}
}

ArcRoute::ArcRoute(Instance const& inst, std::vector<std::pair<int, int>> const& route, int t)
		: _links{inst.nlinks}{

	period = t;

	for(int l = 0; l < _links.dimension(0); ++l){
		links(l) = false;
	}

	cost = 0;
	int prev_v = 0;
	full_path.reserve(inst.nlinks);

	for(const auto & link : route){
		int u = link.first - 1;       // -1 because vidal nodes starts from 1
		int v = link.second - 1;

		auto path = build_path(inst.prev, prev_v, u);
		for(auto const& [i, j]: path){
			full_path.push_back({i, j});
			cost += inst.trav_cost(i, j);
			if(inst.required(i,j)){
				links(inst.id(i, j)) = true;
				visited_rlink.insert(inst.id(i, j));
			}
		}
		if(u != 0 || v != 0){
			full_path.push_back({u, v});
			cost += inst.trav_cost(u, v);
			if(inst.required(u, v)){
				links(inst.id(u, v)) = true;
				visited_rlink.insert(inst.id(u, v));
			}
		}

		prev_v = v;
	}
}

ArcRoute::ArcRoute(Instance const& inst, std::vector<std::pair<int, int>> const& my_route, int t, bool dummy)
		: _links{inst.nlinks}{

	period = t;
	for(int l = 0; l < _links.dimension(0); ++l){
		links(l) = false;
	}
	cost = 0;
	int firstV = my_route[0].first;
	auto before_path = build_path(inst.prev, 0, firstV);
	for(auto const& [i, j]: before_path){
		full_path.emplace_back(i, j);
		cost += inst.trav_cost(i, j);
		if(inst.required(i,j)){
			links(inst.id(i, j)) = true;
			visited_rlink.insert(inst.id(i, j));
		}
	}
	for(const auto & link : my_route){
		int u = link.first;
		int v = link.second;
		full_path.emplace_back(u, v);
		cost += inst.trav_cost(u, v);
		if(inst.required(u, v)){
			links(inst.id(u, v)) = true;
			visited_rlink.insert(inst.id(u, v));
		}
	}
	int lastU = my_route.back().second;
	auto after_path = build_path(inst.prev, lastU, 0);
	for(auto const& [i, j]: after_path){
		full_path.emplace_back(i, j);
		cost += inst.trav_cost(i, j);
		if(inst.required(i,j)){
			links(inst.id(i, j)) = true;
			visited_rlink.insert(inst.id(i, j));
		}
	}
}

int& ArcRoute::links(int l){
	assert(l >= 0 && l < _links.size());
	return _links(l);
}

int ArcRoute::links(int l) const{
	assert(l >= 0 && l < _links.size());
	return _links(l);
}

void ArcRoute::insert_links(Instance const& inst, std::vector<std::pair<int, int>> vecLinks, int bestNode, int pos){


	int firstU = vecLinks[0].first;
	int lastV = vecLinks.back().second;

	auto postPath = build_path(inst.prev, lastV, bestNode);
	full_path.insert(full_path.begin() + pos, postPath.begin(), postPath.end());
	for(auto const [i, j]: postPath){
		links(inst.id(i, j)) = true;
		cost += inst.trav_cost(i, j);
	}

	full_path.insert(full_path.begin() + pos, vecLinks.begin(), vecLinks.end());
	for(auto const [i, j]: vecLinks){
			links(inst.id(i, j)) = true;
			cost += inst.trav_cost(i, j);
	}

	auto prevPath = build_path(inst.prev, bestNode, firstU);
	full_path.insert(full_path.begin() + pos, prevPath.begin(), prevPath.end());
	for(auto const [i, j]: prevPath){

			links(inst.id(i, j)) = true;
			cost += inst.trav_cost(i, j);
	}

	check_cost(inst);
}

void ArcRoute::remove_links(Instance const& inst, int fromLink, int toLink){


	int prev = full_path[fromLink].first;
	int post = full_path[toLink].second;

	for(int linkIndex = fromLink; linkIndex <= toLink; linkIndex++){
		int u = full_path[linkIndex].first;
		int v = full_path[linkIndex].second;

			links(inst.id(u, v)) = false;
			cost -= inst.trav_cost(u, v);
	}
	full_path.erase(full_path.begin() + fromLink, full_path.begin() + toLink + 1);

	auto path = build_path(inst.prev, prev, post);
	full_path.insert(full_path.begin() + fromLink, path.begin(), path.end());
	for(auto const [i, j]: path){

			links(inst.id(i, j)) = true;
			cost += inst.trav_cost(i, j);
	}
	check_cost(inst);
}

void ArcRoute::check_cost(Instance const& inst) const{

	int new_cost = 0;
	int prev_j = 0;
	int el = 0;
	for(auto const [i, j]: full_path){
		new_cost += inst.trav_cost(i, j);
		if(el>0){
			assert(i == prev_j);
		}
		el++;
		prev_j = j;
	}

	assert(new_cost == cost);
}

bool ArcRoute::contains(int id) const{
	return links(id);
}

void check_routes(Instance const& inst, std::vector<ArcRoute> const& routes){
	std::vector<int> links(inst.nreq_links);
	std::iota(links.begin(), links.end(), 0);
	check_routes(links, routes);
}

void check_routes(std::vector<int> const& link_to_visit, std::vector<ArcRoute> const& routes){
	for(int const l: link_to_visit){
		bool flag = false;
		for(auto const& route: routes){
			if(route.contains(l)){
				flag = true;
			}
		}
		assert(flag);
	}
}

void print_routes(Instance const& inst, const std::string& file_name, std::vector<ArcRoute>& routes){
	std::string print = "";
	for(auto const& route: routes){
		std::string list_of_links = fmt::format("{} [", route.period);
		for(auto const& link: route.full_path){
			int u = link.first;
			int v = link.second;
			if(inst.type(u, v) == ARC && inst.required(u, v)){
				list_of_links += fmt::format("({},{}), ", u, v);
			}else if(inst.type(u, v) == EDGE){
				if(inst.required(u, v)){
					list_of_links += fmt::format("({},{}), ", u, v);
				}else if(inst.required(v, u)){
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