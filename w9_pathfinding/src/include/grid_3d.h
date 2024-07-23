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
            friend std::ostream& operator << (std::ostream& os, const Point& p) {
                os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
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
        vector<pair<int, double>> get_neighbors(int node) const;
        double estimate_distance(int v1, int v2) const;
        AbsGraph* reverse() const;

    private:
        const vector<Point> directions_ = {
            {0, 0, -1}, {0, 0, 1}, {0, -1, 0}, {0, 1, 0}, {-1, 0, 0}, {1, 0, 0}
        };

        void warp_point(Point &p) const;
};
