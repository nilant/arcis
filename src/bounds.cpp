#include "bounds.hpp"

int lower_bound(const Instance &inst) {

    int lb{0};
    for (int l = 0; l < inst.nreq_links; ++l) {
        for (int s = 0; s < inst.nsubperiods; ++s) {
            int u = inst.links[l].first;
            int v = inst.links[l].second;
            lb += inst.serv_cost(u, v) * inst.frequencies(l, s);
        }
    }

    return lb;
}