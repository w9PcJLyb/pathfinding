#pragma once

#include "core.h"


class Grid : public AbsGrid {

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

        Grid(int width, int height);
        Grid(int width, int height, vector<double> weights);
        Grid(vector<vector<double>> &weights);
        Grid(const Grid& grid);
        const int width, height;
        double diagonal_movement_cost_multiplier = 1;
        bool passable_left_right_border = false, passable_up_down_border = false;

        size_t size() const;
        void set_diagonal_movement(int);
        int get_diagonal_movement() const;
        void show_obstacle_map() const;
        bool is_inside(const Point &p) const;
        vector<pair<int, double>> get_neighbors(int node) const;
        int get_node_id(const Point &p) const;
        Point get_coordinates(int node) const;
        double estimate_distance(int v1, int v2) const;
        AbsGraph* reverse() const;

    private:
        // 0 - without diagonal movements
        // 1 - allow only when no obstacle
        // 2 - allow if at most one obstacle
        // 3 - always allow
        int diagonal_movement_ = 0;

        const vector<Point> directions_ = {
            // orthogonal movements: top, bottom, left, right
            {0, -1}, {0, 1}, {-1, 0}, {1, 0},
            // diagonal movements
            {-1, -1}, {1, -1}, {-1, 1}, {1, 1}
        };

        void warp_point(Point &p) const;
};
