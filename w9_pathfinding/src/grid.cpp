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

bool Grid::is_inside(const Point &p) const {
    return (p.x >= 0 && p.x < width && p.y >= 0 && p.y < height);
}

void Grid::show_obstacle_map() const {
    for (int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            cout << has_obstacle(get_node_id({x, y})) << " ";
        }
        cout << endl;
    }
}

void Grid::warp_point(Point &p) const {
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

int Grid::get_node_id(const Point &p) const {
    return p.x + p.y * width;
}

Grid::Point Grid::get_coordinates(int node) const {
    return {node % width, node / width};
}

const std::array<Grid::Point, 8> Grid::directions_ = {{
    // orthogonal movements: top, bottom, left, right
    {0, -1}, {0, 1}, {-1, 0}, {1, 0},
    // diagonal movements
    {-1, -1}, {1, -1}, {-1, 1}, {1, 1}
}};

vector<pair<int, double>> Grid::get_neighbors(int node, bool reversed, bool include_self) {
    vector<pair<int, double>> nb;  // neighbors
    nb.reserve(include_self + (diagonal_movement_ == 0 ? 4 : 8));

    double node_weight = weights_.at(node);
    if (node_weight == -1)
        return nb;

    Point p0 = get_coordinates(node);

    auto add_direction = [&] (int direction_id) {
        Point p = p0 + directions_[direction_id];

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

    // add center direction
    if (include_self) {
        double weight = get_pause_weight(node);
        if (weight != -1)
            nb.push_back({node, weight});
    }

    // add cardinal directions
    int top = add_direction(0);
    int bottom = add_direction(1);
    int left = add_direction(2);
    int right = add_direction(3);

    // add diagonal directions
    switch (diagonal_movement_) {

        case 1:  // only when no obstacle
            if ((top >= 0) && (left >= 0)) add_direction(4);
            if ((top >= 0) && (right >= 0)) add_direction(5);
            if ((bottom >= 0) && (left >= 0)) add_direction(6);
            if ((bottom >= 0) && (right >= 0)) add_direction(7);
            break;

        case 2:  // if at most one obstacle
            if ((top >= 0) || (left >= 0)) add_direction(4);
            if ((top >= 0) || (right >= 0)) add_direction(5);
            if ((bottom >= 0) || (left >= 0)) add_direction(6);
            if ((bottom >= 0) || (right >= 0)) add_direction(7);
            break;

        case 3:  // always
            add_direction(4);
            add_direction(5);
            add_direction(6);
            add_direction(7);
            break;
    }

    return nb;
}

double Grid::estimate_distance(int v1, int v2) const {
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
    return get_coordinates(v).to_string();
}

double Grid::calculate_cost(Path& path) {
    if (path.size() <= 1)
        return 0;

    double total_cost = 0;

    Point point = get_coordinates(path[0]);

    for (size_t i = 1; i < path.size(); i++) {
        Point next_point = get_coordinates(path[i]);

        if (point == next_point) {
            total_cost += get_pause_weight(path[i]);
            continue;
        }

        if (point.x != next_point.x && point.y != next_point.y)
            total_cost += weights_.at(path[i]) * diagonal_movement_cost_multiplier;
        else
            total_cost += weights_.at(path[i]);

        point = next_point;
    }

    return total_cost;
}
