#pragma  once

#include <unordered_map>
#include <string>
#include <chrono>


using time_point = decltype(std::chrono::high_resolution_clock::now());

struct Timer {
    std::unordered_map<std::string, std::pair<time_point, time_point>> map;

    void start(std::string const& phase) {
        auto t0 = std::chrono::high_resolution_clock::now();
        map[phase] = {t0, t0};
    }

    void stop(std::string const& phase) {
        auto t1 = std::chrono::high_resolution_clock::now();
        map[phase].second = t1;
    }

    double duration(std::string const& phase) {
        auto t0 = map[phase].first;
        auto t1 = map[phase].second;
        return std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0).count() / 1000.0;
    }
};