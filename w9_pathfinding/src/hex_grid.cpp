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

bool HexGrid::is_inside(const vector<int>& p) const {
    return (p[0] >= 0 && p[0] < width && p[1] >= 0 && p[1] < height);
}

int HexGrid::get_node_id(const vector<int>& p) const {
    return p[0] + p[1] * width;
}

vector<int> HexGrid::get_coordinates(int node) const {
    return {node % width, node / width};
}

void HexGrid::warp_point(vector<int>& p) const {
    if (passable_left_right_border) {
        if (p[0] < 0)
            p[0] += width;
        else if (p[0] >= width)
            p[0] -= width;
    }
    if (passable_up_down_border) {
        if (p[1] < 0)
            p[1] += height;
        else if (p[1] >= height)
            p[1] -= height;
    }
}

const std::array<std::array<int, 2>, 24> HexGrid::directions_ = {{
    {-1, 0}, {1, 0}, {0, -1}, {1, -1}, {0, 1}, {1, 1},   // odd-r
    {-1, 0}, {1, 0}, {-1, -1}, {0, -1}, {-1, 1}, {0, 1}, // even-r
    {0, -1}, {0, 1}, {-1, 0}, {-1, 1}, {1, 0}, {1, 1},   // odd-q
    {0, -1}, {0, 1}, {-1, -1}, {-1, 0}, {1, -1}, {1, 0}  // even-q
}};

int HexGrid::get_direction_offset(const vector<int>& p) const {
    if (layout <= 1)
        return (p[1] % 2 == layout) ? 6 : 0;
    else
        return (p[0] % 2 == layout - 2) ? 18 : 12;
}

vector<pair<int, double>> HexGrid::get_neighbors(int node, bool reversed) {
    vector<pair<int, double>> nb;
    
    double node_weight = weights_.at(node);
    if (node_weight == -1)
        return nb;

    vector<int> p0 = get_coordinates(node);

    nb.reserve(6);

    int offset = get_direction_offset(p0);
    for (int i = 0; i < 6; i++) {
        auto& dir = directions_[i + offset];
        vector<int> p = {p0[0] + dir[0], p0[1] + dir[1]};
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
    vector<int> p1 = get_coordinates(v1);
    vector<int> p2 = get_coordinates(v2);

    int dx = abs(p1[0] - p2[0]);
    if (passable_left_right_border && dx > width / 2) {
        dx = width - dx;
    }

    int dy = abs(p1[1] - p2[1]);
    if (passable_up_down_border && dy > height / 2) {
        dy = height - dy;
    }

    return min_weight_ * std::max(dx, dy);  // not as tight as possible
}

std::string HexGrid::node_to_string(int v) const {
    vector<int> p = get_coordinates(v);
    return "(" + std::to_string(p[0]) + ", " + std::to_string(p[1]) + ")";
}

double HexGrid::calculate_cost(Path& path) {
    if (path.size() <= 1)
        return 0;

    double total_cost = 0;

    vector<int> point = get_coordinates(path[0]);

    for (size_t i = 1; i < path.size(); i++) {
        vector<int> next_point = get_coordinates(path[i]);

        if (point == next_point)
            total_cost += get_pause_action_cost();
        else {
            total_cost += weights_.at(path[i]);
            point = next_point;
        }
    }

    return total_cost;
}
