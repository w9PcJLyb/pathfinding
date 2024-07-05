#include "include/grid_3d.h"


Grid3D::Grid3D(int width, int height, int depth) : width(width), height(height), depth(depth) {
    passable_borders = false;

    weights_.resize(size(), 1);
    min_weight_ = 1;
    reversed_ = false;
}

Grid3D::Grid3D(int width, int height, int depth, vector<double> weights) : width(width), height(height), depth(depth) {
    passable_borders = false;
    set_weights(weights);
    reversed_ = false;
}

size_t Grid3D::size() const {
    return width * height * depth;
}

bool Grid3D::is_inside(const Point &p) const {
    return (p.x >= 0 && p.x < width && p.y >= 0 && p.y < height && p.z >= 0 && p.z < depth);
}

void Grid3D::set_weights(vector<double> &weights) {
    if (weights.size() != size())
        throw std::invalid_argument("Wrong shape");

    if (!weights.empty()) {
        min_weight_ = -1;
        for (double w : weights) {
            if (w < 0 && w != -1) {
                throw std::invalid_argument("Weight must be positive or equal to -1");
            }
            if (w != -1) {
                if (min_weight_ == -1 || min_weight_ > w)
                    min_weight_ = w;
            }
        }
    }

    weights_ = weights;
}

vector<double> Grid3D::get_weights() const {
    return weights_;
}

bool Grid3D::has_obstacle(int node) const {
    return weights_.at(node) == -1;
}

void Grid3D::add_obstacle(int node) {
    weights_.at(node) = -1;
}

void Grid3D::remove_obstacle(int node) {
    weights_.at(node) = 1;
}

void Grid3D::clear_weights() {
    std::fill(weights_.begin(), weights_.end(), 1);
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

vector<pair<int, double>> Grid3D::get_neighbours(int node) const {
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
    reversed_grid->reversed_ = !reversed_;
    return reversed_grid;
}

vector<vector<int>> Grid3D::find_components() const {
    vector<vector<int>> components = AbsGraph::find_components();
    vector<vector<int>> components_without_walls;
    for (vector<int> x : components) {
        if (x.size() == 1 && has_obstacle(x[0]))
            continue;
        components_without_walls.push_back(x);
    }
    return components_without_walls;
}
