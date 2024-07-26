//
// Created by Paolo on 05/03/2024.
//


#include "Preprocessing.hpp"

#include <set>
#include <random>

Preprocessing::Preprocessing()= default;

CarpInstance::CarpInstance(int v_nVehicle) : nVehicle(v_nVehicle){}

CarpInstance::CarpInstance(){
	nVehicle = -1;
};

bool IsSubset(std::vector<int> const& A, std::vector<int> const& B)
{
	// std::sort(A.begin(), A.end());
	// std::sort(B.begin(), B.end());
	return std::includes(A.begin(), A.end(), B.begin(), B.end());
}

void Preprocessing::run(const Instance& inst){


	std::vector<std::set<int>> contains(inst.nsubperiods);
	std::set<int> elementarySP;

	for(int sp1 = 0; sp1 < inst.nsubperiods; sp1++){
		bool isElementary = true;
		for(int sp2 = 0; sp2 < inst.nsubperiods; sp2++){
			if(sp1 != sp2 && IsSubset(inst.subperiods[sp1], inst.subperiods[sp2])){
				isElementary = false;
				contains[sp1].insert(sp2);
				break;
			}
		}
		if(isElementary){
			elementarySP.insert(sp1);
			carpMap[sp1] = CarpInstance(inst.nvehicles*((int) inst.subperiods[sp1].size()));
		}
	}

	for(int sp = 0; sp < inst.nsubperiods; sp++){
		for(auto el : elementarySP)
			if(IsSubset(inst.subperiods[sp], inst.subperiods[el]))
				contains[sp].insert(el);
	}


	std::vector<int> toBeBroken(inst.nsubperiods, 1);
	for(int sp = 0; sp < inst.nsubperiods; sp++){
		for(int l = 0; l < inst.nreq_links; l++){
			if(inst.frequencies(l, sp) >= contains[sp].size()){
				for(auto c_sp : contains[sp])
					carpMap[c_sp].link_to_visit.push_back(l);
			}
			if(inst.frequencies(l, sp) > contains[sp].size()){
				toBeBroken[sp] = (int) fmax(toBeBroken[sp], inst.frequencies(l, sp));
			}
		}
	}

	std::random_device rd;
	std::mt19937 gen(0);
	std::uniform_real_distribution<> dis(0, 1);	//uniform distribution between 0 and 1

	for(int sp = 0; sp < inst.nsubperiods; sp++){
		for(int l = 0; l < inst.nreq_links; l++){
			int freq = inst.frequencies(l, sp);
			if(freq > 0 && freq < contains[sp].size()){
				std::vector<std::pair<double, int>> nearestSp;
				for(const auto& carp : carpMap)
					if(contains[sp].count(carp.first))
						nearestSp.emplace_back(dis(gen), carp.first);
				std::sort(nearestSp.begin(), nearestSp.end());
				for(int f = 0; f < freq; f++)
					carpMap[nearestSp[f].second].link_to_visit.push_back(l);
			}
		}
	}

}
