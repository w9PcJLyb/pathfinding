#include "include/grid_3d.h"


Grid3D::Grid3D(int width, int height, int depth) : width(width), height(height), depth(depth) {
    min_weight_ = 1;
    weights_.resize(size(), min_weight_);
}

Grid3D::Grid3D(int width, int height, int depth, vector<double> weights) : width(width), height(height), depth(depth) {
    set_weights(weights);
}

Grid3D::Grid3D(vector<vector<vector<double>>> &weights) : width(weights[0][0].size()), height(weights[0].size()), depth(weights.size()) {
    vector<double> flat_weights;
    flat_weights.reserve(size());
    for (auto &cut : weights) {
        for (auto &row : cut) {
            flat_weights.insert(flat_weights.end(), row.begin(), row.end());
        }
    }
    set_weights(flat_weights);
}

size_t Grid3D::size() const {
    return width * height * depth;
}

bool Grid3D::is_inside(const Point &p) const {
    return (p.x >= 0 && p.x < width && p.y >= 0 && p.y < height && p.z >= 0 && p.z < depth);
}

int Grid3D::get_node_id(const Point &p) const {
    return p.x + p.y * width + p.z * width * height;
}

Grid3D::Point Grid3D::get_coordinates(int node) const {
    int xy = node % (width * height);
    return {xy % width, xy / width, node / (width * height)};
}

void Grid3D::warp_point(Point &p) const {
    if (!passable_borders) {
        return;
    }

    if (p.x < 0)
        p.x += width;
    else if (p.x >= width)
        p.x -= width;
    
    if (p.y < 0)
        p.y += height;
    else if (p.y >= height)
        p.y -= height;

    if (p.z < 0)
        p.z += depth;
    else if (p.z >= depth)
        p.z -= depth;
}

vector<pair<int, double>> Grid3D::get_neighbors(int node) const {
    vector<pair<int, double>> nb;

    double node_weight = weights_.at(node);
    if (node_weight == -1)
        return nb;

    Point p0 = get_coordinates(node);

    for (const Point &d : directions_) {
        Point p = p0 + d;
        if (!is_inside(p)) {
            if (passable_borders)
                warp_point(p);
            else
                continue;
        }

        int node_id = get_node_id(p);
        double weight = weights_.at(node_id);
        if (weight == -1)
            continue;

        if (reversed_)
            weight = node_weight;
        
        nb.push_back({node_id, weight});
    }

    return nb;
}

double Grid3D::estimate_distance(int v1, int v2) const {
    Point p1 = get_coordinates(v1);
    Point p2 = get_coordinates(v2);

    int dx = abs(p1.x - p2.x);
    int dy = abs(p1.y - p2.y);
    int dz = abs(p1.z - p2.z);

    if (passable_borders) {
        if (dx > width / 2)
            dx = width - dx;

        if (dy > height / 2)
            dy = height - dy;

        if (dz > depth / 2)
            dz = depth - dz;
    }

    return min_weight_ * (dx + dy + dz);
}

AbsGraph* Grid3D::reverse() const {
    Grid3D* reversed_grid(new Grid3D(width, height, depth, weights_));
    reversed_grid->passable_borders = passable_borders;
    reversed_grid->set_pause_action_cost(get_pause_action_cost());
    reversed_grid->reversed_ = !reversed_;
    return reversed_grid;
}
