#pragma once

#include "env.h"


class AbsMAPF {
    public:
        AbsMAPF() {};
        virtual ~AbsMAPF() {};
        virtual vector<Path> mapf(vector<int> starts, vector<int> goals) = 0;
};


class timeout_exception : public std::runtime_error {
    public:
        using std::runtime_error::runtime_error;
};


void ensure_path_length(Path& path, int length);


bool has_collision(const vector<int>& positions, const vector<int>& next_positions, bool edge_collision);


template <typename T>
bool generate_next_combination(vector<vector<T>>& data, vector<T>& combination, vector<int>& indices) {
    if (combination.empty()) {
        if (data.empty())
            return false;

        for (auto& d : data) {
            if (d.empty())
                return false;
            combination.push_back(d[0]);
        }
        indices.resize(data.size(), 0);
        return true;
    }

    for (int i = data.size() - 1; i >= 0; --i) {
        if (indices[i] < (int)data[i].size() - 1) {
            indices[i]++;
            combination[i] = data[i][indices[i]];
            return true;
        } else {
            indices[i] = 0;
            combination[i] = data[i][0];
        }
    }
    return false;
}

struct SpaceHash {
    size_t operator()(const std::vector<int>& position) const {
        size_t hash_value = 0;
        for (const auto& x : position) {
            hash_value ^= std::hash<int>{}(x) + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
        }
        return hash_value;
    }
};


struct SpaceTimeHash {
    size_t operator()(const std::pair<int, std::vector<int>>& p) const {
        size_t hash_value = std::hash<int>{}(p.first);
        size_t vector_hash = SpaceHash{}(p.second);
        hash_value ^= vector_hash + 0x9e3779b9 + (hash_value << 6) + (hash_value >> 2);
        return hash_value;
    }
}; 
