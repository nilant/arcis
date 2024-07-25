#pragma once

#include "ArcRoute.hpp"
#include "Instance.hpp"
#include "mdarray.hpp"
#include "LocalSearch.hpp"
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

void inline rand_floyd_warshall(Matrix const& graph, Matrix& dist, Matrix& prev, RandomGenerator&rand_gen) {
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
				} else if(dist(i, j) < INF && areDistinct(i, j, k) && dist(i, j) == sat_sum(dist(i, k), dist(k, j))){
					int coin = rand_gen.coin();
					if (coin) { // <- con probabilità del 50%
						dist(i, j) = sat_sum(dist(i, k), dist(k, j));
						prev(i, j) = prev(k, j);
					}
				}
			}
		}
	}
}