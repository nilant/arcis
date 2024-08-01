#pragma once

#include "ArcRoute.hpp"
#include "Instance.hpp"
#include "mdarray.hpp"
#include "local_search.hpp"
#include <limits>
#include <iostream>
#include <random>

const int INF = std::numeric_limits<int>::max();
using Matrix = mdarray<int, 2>;

inline int sat_sum(int a, int b) {
    if (a == INF || b == INF) {
        return INF;
    }
    return a + b;
}

bool inline areDistinct(int a, int b, int c) {
	return (a != b) && (b != c) && (a != c);
}

void inline floyd_warshall(Matrix const& graph, Matrix& dist, Matrix& prev) {
    auto n = graph.dimension(0);
    
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            if (i == j) {
                dist(i, j) = 0;
            } else {
                dist(i, j) = graph(i, j);
            }
            prev(i, j) = graph(i, j) != INF ? i : INF; 
        }
    }
    for (int k = 0; k < n; ++k) {
        for (int i = 0; i < n; ++i) {
            for (int j = 0; j < n; ++j) {
                if (areDistinct(i, j, k) && dist(i, j) > sat_sum(dist(i, k), dist(k, j))) {
                    dist(i, j) = sat_sum(dist(i, k), dist(k, j));
                    prev(i, j) = prev(k, j);
                }/*else if(dist(i, j) < INF && areDistinct(i, j, k) && dist(i, j) == sat_sum(dist(i, k), dist(k, j))){
	                dist(i, j) = sat_sum(dist(i, k), dist(k, j));
	                prev(i, j) = prev(k, j);
                }*/
            }
        }
    }
}

void inline rand_floyd_warshall(Matrix const& graph, Matrix& dist, Matrix& prev, RandomGenerator&rand_gen, const mdarray<int, 2>& required) {
	auto n = graph.dimension(0);

	mdarray<int, 2> agg_required(n, n);

	for (int i = 0; i < n; ++i) {
		for (int j = 0; j < n; ++j) {
			if (i == j) {
				dist(i, j) = 0;
				agg_required(i, j) = 0;
			} else {
				dist(i, j) = graph(i, j);
				agg_required(i, j) = required(i, j);
			}
			prev(i, j) = graph(i, j) != INF ? i : INF;
		}
	}
	for (int k = 0; k < n; ++k) {
		for (int i = 0; i < n; ++i) {
			for (int j = 0; j < n; ++j) {
				if (areDistinct(i, j, k) && sat_sum(dist(i, k), dist(k, j)) < INF){
					if(dist(i, j) > sat_sum(dist(i, k), dist(k, j))){
						agg_required(i, j) = sat_sum(agg_required(i, k), agg_required(k, j));
						dist(i, j) = sat_sum(dist(i, k), dist(k, j));
						prev(i, j) = prev(k, j);
					}else if(dist(i, j) == sat_sum(dist(i, k), dist(k, j))){
						double prob = (double) agg_required(i, j) / (agg_required(i, j) + sat_sum(agg_required(i, k), agg_required(k, j)));
						double rand_number = rand_gen.rand_0_1();
						if(prob < rand_number){
							agg_required(i, j) = sat_sum(agg_required(i, k), agg_required(k, j));
							dist(i, j) = sat_sum(dist(i, k), dist(k, j));
							prev(i, j) = prev(k, j);
						}
					}
				}
			}
		}
	}
}