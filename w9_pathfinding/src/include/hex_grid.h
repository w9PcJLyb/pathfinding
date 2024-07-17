#pragma once

#include "core.h"


class HexGrid : public AbsGrid {

    /*
    Hexagonal Grid with pointy top orientation and “odd-r” layout

    Example, 4x3 hex grid:

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

        HexGrid(int width, int height);
        HexGrid(int width, int height, vector<double> weights);
        const int width, height;
        bool passable_left_right_border, passable_up_down_border;

        bool is_inside(const Point &p) const;
        vector<pair<int, double>> get_neighbors(int node) const;
        int get_node_id(const Point &p) const;
        Point get_coordinates(int node) const;
        double estimate_distance(int v1, int v2) const;
        AbsGraph* reverse() const;

    private:
        vector<Point> even_directions_ = {{-1, 0}, {1, 0}, {-1, -1}, {0, -1}, {-1, 1}, {0, 1}};
        vector<Point> odd_directions_ = {{-1, 0}, {1, 0}, {0, -1}, {1, -1}, {0, 1}, {1, 1}};

        void warp_point(Point &p) const;
};
