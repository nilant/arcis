//
// Created by Paolo on 05/03/2024.
//


#include "Preprocessing.hpp"


bool IsSubset(std::vector<int> const& A, std::vector<int> const& B)
{
	// std::sort(A.begin(), A.end());
	// std::sort(B.begin(), B.end());
	return std::includes(A.begin(), A.end(), B.begin(), B.end());
}

Preprocessing::Preprocessing(Instance const& inst){

	contains.resize(inst.nsubperiods);
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
		}
	}
};

CarpInstance::CarpInstance(int v_nVehicle) : nVehicle(v_nVehicle){}

CarpInstance::CarpInstance(){
	nVehicle = -1;
};

void Preprocessing::run(const Instance& inst, RandomGenerator& rand_gen){

	std::vector<int> link_id(inst.nreq_links);
	for (int i = 0; i < link_id.size(); ++i) {
		link_id[i] = i;
	}

	std::shuffle(link_id.begin(), link_id.end(), rand_gen.gen);

	for(int t=0; t < inst.horizon; t++)
		carpMap[t] = CarpInstance(1);

	for(int sp = 0; sp < inst.nsubperiods; sp++){

		std::vector<int> periods;
		for(int t = 0; t < inst.horizon; t++)
			if(inst.sp_matrix(sp, t))
				periods.push_back(t);

		for(int ll = 0; ll < inst.nreq_links; ll++){
			int l = link_id[ll];
			int freq = inst.frequencies(l, sp);
			if(freq > 0){
				std::vector<int> chooseT;
				for(auto t: periods)
					if(!carpMap[t].link_to_visit.empty() && !carpMap[t].link_to_visit.count(l))
						chooseT.push_back(t);
				if (chooseT.size() > freq) {
					std::shuffle(chooseT.begin(), chooseT.end(), rand_gen.gen);
				}
				else if (chooseT.size() < freq) {
					while(chooseT.size() < freq) {
						int periods_max_index = (int) periods.size() - 1;
						int rand_t = periods.at(rand_gen.getRandomInt(periods_max_index));
						bool not_in_choosT = std::find(chooseT.begin(), chooseT.end(), rand_t) == chooseT.end();
						if(!carpMap[rand_t].link_to_visit.count(l) && not_in_choosT)
							chooseT.push_back(rand_t);
					}
				}

				for(int f = 0; f < freq; f++){
					carpMap[chooseT[f]].link_to_visit.insert(l);
				}
			}
		}
	}
}

