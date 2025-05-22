#pragma once

#include "core.h"


class HexGrid : public AbsGrid {

    /*
    Hexagonal Grid

    Example of 4x3 hex grid with “odd-r” layout:

       / \     / \     / \     / \
     /     \ /     \ /     \ /     \
    |x=0,y=0| 1, 0  | 2, 0  | 3, 0  |
    | id=0  |   1   |   2   |   3   |
     \     / \     / \     / \     / \
       \ /     \ /     \ /     \ /     \
        | 0, 1  | 1, 1  | 2, 1  | 3, 1  |
        |   4   |   5   |   6   |   7   |
       / \     / \     / \     / \     /
     /     \ /     \ /     \ /     \ /
    | 0, 2  | 1, 2  | 2, 2  | 3, 2  |
    |   8   |   9   |   10  |   11  |
     \     / \     / \     / \     /
       \ /     \ /     \ /     \ /

    */

    public:
        HexGrid(int width, int height, int layout);
        HexGrid(int width, int height, int layout, vector<double> weights);
        HexGrid(int layout, vector<vector<double>> &weights);
        const int width, height;

        // 0 - odd-r
        // 1 - even-r
        // 2 - odd-q
        // 3 - even-q
        const int layout;

        bool passable_left_right_border = false, passable_up_down_border = false;

        size_t size() const;
        bool is_inside(const vector<int>& p) const;
        vector<pair<int, double>> get_neighbors(int node, bool reversed=false);
        int get_node_id(const vector<int>& p) const;
        vector<int> get_coordinates(int node) const;
        double estimate_distance(int v1, int v2) const;
        std::string node_to_string(int v) const;
        double calculate_cost(Path& path);

    private:
        static const std::array<std::array<int, 2>, 24> directions_;

        int get_direction_offset(const vector<int>& p) const;
        void warp_point(vector<int>& p) const;
};
