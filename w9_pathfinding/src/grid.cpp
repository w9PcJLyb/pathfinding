#include "include/grid.h"


Grid::Grid(int width, int height) : width(width), height(height) {
    min_weight_ = 1;
    weights_.resize(size(), min_weight_);
}

Grid::Grid(int width, int height, vector<double> weights) : width(width), height(height) {
    set_weights(weights);
}

Grid::Grid(vector<vector<double>> &weights) : width(weights[0].size()), height(weights.size()) {
    vector<double> flat_weights;
    flat_weights.reserve(size());
    for (auto &row : weights) {
        flat_weights.insert(flat_weights.end(), row.begin(), row.end());
    }
    set_weights(flat_weights);
}

size_t Grid::size() const {
    return width * height;
}

void Grid::set_diagonal_movement(int diagonal_movement) {
    if (diagonal_movement < 0 || diagonal_movement > 3)
        std::invalid_argument("diagonal_movement must be uint and less than 4.");
    diagonal_movement_ = diagonal_movement;
}

int Grid::get_diagonal_movement() const {
    return diagonal_movement_;
}

bool Grid::is_inside(const vector<int>& p) const {
    return (p[0] >= 0 && p[0] < width && p[1] >= 0 && p[1] < height);
}

void Grid::show_obstacle_map() const {
    for (int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            cout << has_obstacle(get_node_id({x, y})) << " ";
        }
        cout << endl;
    }
}

void Grid::warp_point(vector<int>& p) const {
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

int Grid::get_node_id(const vector<int>& p) const {
    return p[0] + p[1] * width;
}

vector<int> Grid::get_coordinates(int node) const {
    return {node % width, node / width};
}

const std::array<std::array<int, 2>, 8> Grid::directions_ = {{
    // orthogonal movements: top, bottom, left, right
    {0, -1}, {0, 1}, {-1, 0}, {1, 0},
    // diagonal movements
    {-1, -1}, {1, -1}, {-1, 1}, {1, 1}
}};

vector<pair<int, double>> Grid::get_neighbors(int node, bool reversed) {
    vector<pair<int, double>> nb;
    nb.reserve(diagonal_movement_ == 0 ? 4 : 8);

    vector<int> p0 = get_coordinates(node);

    double node_weight = weights_.at(node);
    if (node_weight == -1)
        return nb;

    auto add_direction = [&] (int direction_id) {
        auto& dir = directions_[direction_id];
        vector<int> p = {p0[0] + dir[0], p0[1] + dir[1]};

        bool inside = is_inside(p);
        if (!inside) {
            warp_point(p);
            inside = is_inside(p);
        }
        if (!inside) 
            return -1;

        int node_id = get_node_id(p);
        double weight = weights_.at(node_id);
        if (weight == -1)
            return -1;

        if (reversed)
            weight = node_weight;

        if (direction_id < 4)
            nb.push_back({node_id, weight});
        else
            nb.push_back({node_id, weight * diagonal_movement_cost_multiplier});

        return node_id;
    };

    if (diagonal_movement_ == 0) {
        // without diagonal movements
        for (int i = 0; i < 4; i++)
            add_direction(i);
        return nb;
    }

    int top = add_direction(0);
    int bottom = add_direction(1);
    int left = add_direction(2);
    int right = add_direction(3);

    bool top_left = 1, top_right = 1, bottom_left = 1, bottom_right = 1;
    if (diagonal_movement_ == 1) {
        // only when no obstacle
        top_left = (top >= 0) && (left >= 0);
        top_right = (top >= 0) && (right >= 0);
        bottom_left = (bottom >= 0) && (left >= 0);
        bottom_right = (bottom >= 0) && (right >= 0);
    }
    else if (diagonal_movement_ == 2) {
        // if at most one obstacle
        top_left = (top >= 0) || (left >= 0);
        top_right = (top >= 0) || (right >= 0);
        bottom_left = (bottom >= 0) || (left >= 0);
        bottom_right = (bottom >= 0) || (right >= 0);
    }
    
    if (top_left) add_direction(4);
    if (top_right) add_direction(5);
    if (bottom_left) add_direction(6);
    if (bottom_right) add_direction(7);

    return nb;
}

double Grid::estimate_distance(int v1, int v2) const {
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

    if (!diagonal_movement_) {
        return min_weight_ * (dx + dy);  // manhattan
    }

    // octile
    if (dx > dy) {
        return min_weight_ * (dx + dy * (diagonal_movement_cost_multiplier - 1));
    }
    else {
        return min_weight_ * (dy + dx * (diagonal_movement_cost_multiplier - 1));
    }
}

std::string Grid::node_to_string(int v) const {
    vector<int> p = get_coordinates(v);
    return "(" + std::to_string(p[0]) + ", " + std::to_string(p[1]) + ")";
}

double Grid::calculate_cost(Path& path) {
    if (path.size() <= 1)
        return 0;

    double total_cost = 0;

    vector<int> point = get_coordinates(path[0]);

    for (size_t i = 1; i < path.size(); i++) {
        vector<int> next_point = get_coordinates(path[i]);

        if (point == next_point) {
            total_cost += get_pause_action_cost();
            continue;
        }

        if (point[0] != next_point[0] && point[1] != next_point[1])
            total_cost += weights_.at(path[i]) * diagonal_movement_cost_multiplier;
        else
            total_cost += weights_.at(path[i]);

        point = next_point;
    }

    return total_cost;
}
