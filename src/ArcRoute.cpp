#include <algorithm>
#include <numeric>
#include <fmt/core.h>
#include <iterator>
#include <fstream>

#include "ArcRoute.hpp"
#include "Instance.hpp"
#include "local_search.hpp"


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
		if(other.full_path[end].second == 0){
			routes.emplace_back(inst, other, start, end);
			start = end + 1;
		}
	}
	return routes;
}

ArcRoute::ArcRoute(Instance const& inst, ArcRoute const& other, int start, int end) : _links{inst.nlinks}{
	cost = 0;
	period = other.period;
	residual_capacity = inst.capacity;

	for(int l = 0; l < _links.dimension(0); ++l)
		links(l) = false;

	std::copy(other.full_path.begin() + start, other.full_path.begin() + (end + 1), std::back_inserter(full_path));
	for(auto const [i, j]: full_path){
		if(inst.t_l_matrix(period, inst.id(i, j))){
			links(inst.id(i, j)) = true;
			residual_capacity -= inst.demand(i, j);
		}
		cost += inst.trav_cost(i, j);
	}
}

ArcRoute::ArcRoute(Instance const& inst, std::vector<std::pair<int, int>> const& route, int t)
		: _links{inst.nlinks}{

	period = t;
	int prev_v = 0;
	full_path.reserve(inst.nlinks);
	for(const auto & link : route){
		int u = link.first - 1;       // -1 because vidal nodes starts from 1
		int v = link.second - 1;
		auto path = build_path(inst.prev, prev_v, u);
		for(auto const& [i, j]: path)
			full_path.emplace_back(i, j);
		if(u != 0 || v != 0)
			full_path.emplace_back(u, v);
		prev_v = v;
	}
}

ArcRoute::ArcRoute(Instance const& inst, std::vector<std::pair<int, int>> const& my_route, int t, bool dummy)
		: _links{inst.nlinks}{

	period = t;
	int firstV = my_route[0].first;
	auto before_path = build_path(inst.prev, 0, firstV);
	full_path.reserve(inst.nlinks);
	for(auto const& [i, j]: before_path)
		full_path.emplace_back(i, j);
	for(const auto & link : my_route){
		int u = link.first;
		int v = link.second;
		full_path.emplace_back(u, v);
	}
	int lastU = my_route.back().second;
	auto after_path = build_path(inst.prev, lastU, 0);
	for(auto const& [i, j]: after_path)
		full_path.emplace_back(i, j);
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
	for(auto const [i, j]: postPath)
		cost += inst.trav_cost(i, j);

	full_path.insert(full_path.begin() + pos, vecLinks.begin(), vecLinks.end());
	for(auto const [i, j]: vecLinks)
		cost += inst.trav_cost(i, j);

	auto prevPath = build_path(inst.prev, bestNode, firstU);
	full_path.insert(full_path.begin() + pos, prevPath.begin(), prevPath.end());
	for(auto const [i, j]: prevPath)
		cost += inst.trav_cost(i, j);
}

void ArcRoute::remove_links(Instance const& inst, int fromLink, int toLink){

	int prev = full_path[fromLink].first;
	int post = full_path[toLink].second;
	for(int linkIndex = fromLink; linkIndex <= toLink; linkIndex++){
		int u = full_path[linkIndex].first;
		int v = full_path[linkIndex].second;
		cost -= inst.trav_cost(u, v);
	}
	full_path.erase(full_path.begin() + fromLink, full_path.begin() + toLink + 1);
	auto path = build_path(inst.prev, prev, post);
	full_path.insert(full_path.begin() + fromLink, path.begin(), path.end());
	for(auto const [i, j]: path)
		cost += inst.trav_cost(i, j);

}

bool ArcRoute::contains(int id) const{
	return links(id);
}

void ArcRoute::make_maximal(Instance const& inst)
{
	RandomGenerator rand_gen{};
	std::vector<int> rand_index(full_path.size());
	for(int id = 0; id < full_path.size(); id++)
		rand_index[id] = id;
	std::shuffle(rand_index.begin(), rand_index.end(), rand_gen.gen);

	for(auto l : rand_index){
		int id_l = inst.id(full_path[l].first, full_path[l].second);
		if(!links(id_l) && residual_capacity - inst.demand(full_path[l].first, full_path[l].second) >= 0){
			links(id_l) = true;
			residual_capacity -= inst.demand(full_path[l].first, full_path[l].second);
		}
	}

}

