#include "include/grid.h"


Grid::Grid(int width, int height) : width(width), height(height) {
    diagonal_movement_ = 0;
    passable_left_right_border = false;
    passable_up_down_border = false;
    diagonal_movement_cost_multiplier = 1.0;

    weights_.resize(size(), 1);
    min_weight_ = 1;
    reversed_ = false;
}

Grid::Grid(int width, int height, vector<double> weights) : width(width), height(height) {
    diagonal_movement_ = 0;
    passable_left_right_border = false;
    passable_up_down_border = false;
    diagonal_movement_cost_multiplier = 1.0;
    set_weights(weights);
    reversed_ = false;
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

void Grid::set_weights(vector<double> &weights) {
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

vector<double> Grid::get_weights() const {
    return weights_;
}

void Grid::show_obstacle_map() const {
    for (int y = 0; y < height; y++) {
        for(int x = 0; x < width; x++) {
            cout << has_obstacle(get_node_id({x, y})) << " ";
        }
        cout << endl;
    }
}

bool Grid::has_obstacle(int node) const {
    return weights_.at(node) == -1;
}

void Grid::add_obstacle(int node) {
    weights_.at(node) = -1;
}

void Grid::remove_obstacle(int node) {
    weights_.at(node) = 1;
}

void Grid::clear_weights() {
    std::fill(weights_.begin(), weights_.end(), 1);
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

vector<Grid::Point> Grid::get_coordinates(vector<int> &nodes) const {
    vector<Point> v;
    for (int node : nodes) {
        v.push_back(get_coordinates(node));
    }
    return v;
}


vector<pair<int, double>> Grid::get_neighbours(int node) const {
    vector<pair<int, double>> nb;
    nb.reserve(diagonal_movement_ == 0 ? 4 : 8);
    
    Point p0 = get_coordinates(node);

    double node_weight = weights_.at(node);
    if (node_weight == -1)
        return nb;

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

        if (reversed_)
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

AbsGraph* Grid::reverse() const {
    Grid* reversed_grid(new Grid(width, height, weights_));
    reversed_grid->set_diagonal_movement(diagonal_movement_);
    reversed_grid->passable_left_right_border = passable_left_right_border;
    reversed_grid->passable_up_down_border = passable_up_down_border;
    reversed_grid->diagonal_movement_cost_multiplier = diagonal_movement_cost_multiplier;
    reversed_grid->reversed_ = !reversed_;
    return reversed_grid;
}

void Grid::reverse_inplace() {
    reversed_ = !reversed_;
}

vector<vector<int>> Grid::find_components() const {
    vector<vector<int>> components = AbsGraph::find_components();
    vector<vector<int>> components_without_walls;
    for (vector<int> x : components) {
        if (x.size() == 1 && has_obstacle(x[0]))
            continue;
        components_without_walls.push_back(x);
    }
    return components_without_walls;
}
