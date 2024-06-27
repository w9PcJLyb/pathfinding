#pragma once

#include "core.h"


class Grid : public AbsGraph {

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
        Grid(int width, int height, vector<int> obstacle_map);
        const int width, height;
        double diaganal_movement_cost_multiplier;
        bool passable_left_right_border, passable_up_down_border;

        size_t size() const;
        void set_diagonal_movement(int);
        int get_diagonal_movement() const;
        void set_obstacle_map(vector<int> &map);
        vector<int> get_obstacle_map() const;
        void show_obstacle_map() const;
        bool has_obstacle(int node) const;
        void add_obstacle(int node);
        void remove_obstacle(int node);
        void clear_obstacles();
        bool is_inside(const Point &p) const;
        vector<pair<int, double>> get_neighbours(int node) const; 
        int get_node_id(const Point &p) const;
        Point get_coordinates(int node) const;
        vector<Point> get_coordinates(vector<int> &p) const;
        double estimate_distance(int v1, int v2) const;
        AbsGraph* reverse() const;

    private:
        int diagonal_movement_;
        vector<Point> directions_ = {
            // top, bottom, left, right
            {0, -1}, {0, 1}, {-1, 0}, {1, 0},
            // diagonal movements
            {-1, -1}, {1, -1}, {-1, 1}, {1, 1}
        };
        vector<int> obstacle_map_;

        void warp_point(Point &p) const;
};
