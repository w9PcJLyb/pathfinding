#pragma once

#include "core.h"


class Grid3D : public AbsGrid {

    public:
        Grid3D(int width, int height, int depth);
        Grid3D(int width, int height, int depth, vector<double> weights);
        Grid3D(vector<vector<vector<double>>> &weights);

        const int width, height, depth;
        bool passable_borders = false;

        size_t size() const;
        int get_node_id(const vector<int>& p) const;
        vector<int> get_coordinates(int node) const;
        bool is_inside(const vector<int>& p) const;
        vector<pair<int, double>> get_neighbors(int node, bool reversed=false);
        double estimate_distance(int v1, int v2) const;
        std::string node_to_string(int v) const;
        double calculate_cost(Path& path);

    private:
        static const std::array<std::array<int, 3>, 6> directions_;

        void warp_point(vector<int>& p) const;
};
