#pragma once

#include "ArcRoute.hpp"
#include "Instance.hpp"
#include "mdarray.hpp"
#include <limits>


const int INF = std::numeric_limits<int>::max();
using Matrix = mdarray<int, 2>; 

inline int sat_sum(int a, int b) {
    if (a == INF || b == INF) {
        return INF;
    }
    return a + b;
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
                if (dist(i, j) > sat_sum(dist(i, k), dist(k, j))) {
                    dist(i, j) = sat_sum(dist(i, k), dist(k, j));
                    prev(i, j) = prev(k, j);
                }
            }
        }
    }
}