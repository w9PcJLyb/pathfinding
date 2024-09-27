#include "include/hex_grid.h"


HexGrid::HexGrid(int width, int height, int layout) : width(width), height(height), layout(layout) {
    min_weight_ = 1;
    weights_.resize(size(), min_weight_);
}

HexGrid::HexGrid(int width, int height, int layout, vector<double> weights) : width(width), height(height), layout(layout) {
    set_weights(weights);
}

HexGrid::HexGrid(int layout, vector<vector<double>> &weights) : width(weights[0].size()), height(weights.size()), layout(layout) {
    vector<double> flat_weights;
    flat_weights.reserve(size());
    for (auto &row : weights) {
        flat_weights.insert(flat_weights.end(), row.begin(), row.end());
    }
    set_weights(flat_weights);
}

size_t HexGrid::size() const {
    return width * height;
}

bool HexGrid::is_inside(const Point &p) const {
    return (p.x >= 0 && p.x < width && p.y >= 0 && p.y < height);
}

int HexGrid::get_node_id(const Point &p) const {
    return p.x + p.y * width;
}

HexGrid::Point HexGrid::get_coordinates(int node) const {
    return {node % width, node / width};
}

void HexGrid::warp_point(Point &p) const {
    if (passable_left_right_border) {
        if (p.x < 0)
            p.x += width;
        else if (p.x >= width)
            p.x -= width;
    }
    if (passable_up_down_border) {
        if (p.y < 0)
            p.y += height;
        else if (p.y >= height)
            p.y -= height;
    }
}

HexGrid::points_ HexGrid::get_directions(const Point &p) const {
    if (layout <= 1) {
        return (p.y % 2 == layout) ? pointy_even_directions_ : pointy_odd_directions_;
    }
    else {
        return (p.x % 2 == layout - 2) ? flat_even_directions_ : flat_odd_directions_;
    }
}

vector<pair<int, double>> HexGrid::get_neighbors(int node, bool reversed) {
    vector<pair<int, double>> nb;
    
    double node_weight = weights_.at(node);
    if (node_weight == -1)
        return nb;

    Point p0 = get_coordinates(node);

    nb.reserve(6);

    for (const Point &d : get_directions(p0)) {
        Point p = p0 + d;
        bool inside = is_inside(p);
        if (!inside) {
            warp_point(p);
            inside = is_inside(p);
        }
        if (!inside) 
            continue;;

        int node_id = get_node_id(p);
        double weight = weights_.at(node_id);
        if (weight == -1)
            continue;

        if (reversed)
            weight = node_weight;

        nb.push_back({node_id, weight});
    }

    return nb;
}

double HexGrid::estimate_distance(int v1, int v2) const {
    Point p1 = get_coordinates(v1);
    Point p2 = get_coordinates(v2);

    int dx = abs(p1.x - p2.x);
    if (passable_left_right_border && dx > width / 2) {
        dx = width - dx;
    }

    int dy = abs(p1.y - p2.y);
    if (passable_up_down_border && dy > height / 2) {
        dy = height - dy;
    }

    return min_weight_ * std::max(dx, dy);  // not as tight as possible
}

std::string HexGrid::node_to_string(int v) const {
    return get_coordinates(v).to_string();
}

double HexGrid::calculate_cost(Path& path) {
    if (path.size() <= 1)
        return 0;

    double total_cost = 0;

    Point point = get_coordinates(path[0]);

    for (size_t i = 1; i < path.size(); i++) {
        Point next_point = get_coordinates(path[i]);

        if (point == next_point)
            total_cost += get_pause_action_cost();
        else {
            total_cost += weights_.at(path[i]);
            point = next_point;
        }
    }

    return total_cost;
}
