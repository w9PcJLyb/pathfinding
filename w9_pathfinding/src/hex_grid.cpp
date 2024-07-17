#include "include/hex_grid.h"


HexGrid::HexGrid(int width, int height) : width(width), height(height) {
    passable_left_right_border = false;
    passable_up_down_border = false;

    weights_.resize(width * height, 1);
    min_weight_ = 1;
    reversed_ = false;
}

HexGrid::HexGrid(int width, int height, vector<double> weights) : HexGrid(width, height) {
    set_weights(weights);
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

vector<pair<int, double>> HexGrid::get_neighbors(int node) const {
    vector<pair<int, double>> nb;
    
    double node_weight = weights_.at(node);
    if (node_weight == -1)
        return nb;

    Point p0 = get_coordinates(node);

    nb.reserve(6);

    const vector<Point> &directions = (p0.y % 2 == 0) ? even_directions_ : odd_directions_;

    for (const Point &d : directions) {
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

        if (reversed_)
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

AbsGraph* HexGrid::reverse() const {
    HexGrid* reversed_grid(new HexGrid(width, height, weights_));
    reversed_grid->passable_left_right_border = passable_left_right_border;
    reversed_grid->passable_up_down_border = passable_up_down_border;
    reversed_grid->set_pause_action_cost(get_pause_action_cost());
    reversed_grid->reversed_ = !reversed_;
    return reversed_grid;
}
