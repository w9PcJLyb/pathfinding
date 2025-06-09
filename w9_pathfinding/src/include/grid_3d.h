#pragma once

#include "core.h"


class Grid3D : public AbsGrid {

    public:

        struct Point {
            int x, y, z;

            Point() : x(0), y(0), z(0) {};
            Point(int x, int y, int z) : x(x), y(y), z(z) {};
            Point(vector<int> v) : x(v[0]), y(v[1]), z(v[2]) {};

            friend Point operator + (const Point& a, const Point& b) {
                return {a.x + b.x, a.y + b.y, a.z + b.z};
            }
            bool operator == (const Point& p) const {
                return x == p.x && y == p.y && z == p.z;
            }
            std::string to_string() const {
                return "(" + std::to_string(x) + ", " + std::to_string(y) + ", " + std::to_string(z) + ")";
            }
            friend std::ostream& operator << (std::ostream& os, const Point& p) {
                os << p.to_string();
                return os;
            }
        };

        Grid3D(int width, int height, int depth);
        Grid3D(int width, int height, int depth, vector<double> weights);
        Grid3D(vector<vector<vector<double>>> &weights);

        const int width, height, depth;
        bool passable_borders = false;

        size_t size() const;
        int get_node_id(const Point &p) const;
        Point get_coordinates(int node) const;
        bool is_inside(const Point &p) const;
        vector<pair<int, double>> get_neighbors(int node, bool reversed=false, bool include_self=false);
        double estimate_distance(int v1, int v2) const;
        std::string node_to_string(int v) const;
        double calculate_cost(Path& path);

    private:
        static const std::array<Point, 6> directions_;

        void warp_point(Point &p) const;
};
