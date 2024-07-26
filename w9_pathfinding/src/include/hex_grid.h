#pragma once

#include "core.h"


class HexGrid : public AbsGrid {

    /*
    Hexagonal Grid

    For example, 4x3 hex grid with “odd-r” layout:

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

        struct Point {
            int x, y;

            Point() : x(0), y(0) {};
            Point(int x, int y) : x(x), y(y) {};

            friend Point operator + (const Point& a, const Point& b) {
                return {a.x + b.x, a.y + b.y};
            }
            bool operator == (const Point& p) const {
                return x == p.x && y == p.y;
            }
            friend std::ostream& operator << (std::ostream& os, const Point& p) {
                os << "(" << p.x << ", " << p.y << ")";
                return os;
            }
        };

        typedef const vector<Point> points_;

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
        bool is_inside(const Point &p) const;
        vector<pair<int, double>> get_neighbors(int node) const;
        int get_node_id(const Point &p) const;
        Point get_coordinates(int node) const;
        double estimate_distance(int v1, int v2) const;
        AbsGraph* reverse() const;

    private:
        // pointy top: odd-r or even-r
        points_ pointy_even_directions_ = {{-1, 0}, {1, 0}, {-1, -1}, {0, -1}, {-1, 1}, {0, 1}};
        points_ pointy_odd_directions_ = {{-1, 0}, {1, 0}, {0, -1}, {1, -1}, {0, 1}, {1, 1}};

        // flat top: odd-q or even-q
        points_ flat_even_directions_ = {{0, -1}, {0, 1}, {-1, -1}, {-1, 0}, {1, -1}, {1, 0}};
        points_ flat_odd_directions_ = {{0, -1}, {0, 1}, {-1, 0}, {-1, 1}, {1, 0}, {1, 1}};

        points_ get_directions(const Point &p) const;
        void warp_point(Point &p) const;
};
