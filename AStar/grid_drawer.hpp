#ifndef ASTAR_GRID_DRAWER_HPP
#define ASTAR_GRID_DRAWER_HPP

#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include <unordered_set>
#include "point.hpp"


template <typename Point = P>
class GridMap: public sf::Drawable, public sf::Transformable{
public:
    GridMap() = default;

    bool load(sf::Vector2u tileSize, std::vector<sf::Color> colors);
    void set_width(unsigned int w_) { width = w_; };
    void set_height(unsigned int h_) { height = h_; };
    void set_start(const Point &p){ start = p; };
    void set_stop(const Point &p){ stop = p; };
    void update_path(const std::unordered_set<Point, std::hash<Point>> &new_path){ path = new_path; };
    void update_wall(const std::unordered_set<Point, std::hash<Point>> &new_walls){ walls = new_walls; };
    bool in_walls(const Point& check) { return walls.find(check) != walls.end(); };
    void clear() { path.clear(); walls.clear(); start = {-1, -1}; stop = {-1, -1}; };
    void clear_path() { path.clear(); };
    void clear_walls() { walls.clear(); };
    void set_point(const Point &p, sf::Color color_);
    bool add_wall(const Point& wall_coords);
    bool remove_wall(const Point& wall_cords);

private:
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const;

    unsigned int width = 0;
    unsigned int height = 0;
    Point start = {-1, -1};
    Point stop = {-1, -1};
    sf::VertexArray grid_vertices;
    sf::VertexArray line_col;
    sf::VertexArray line_row;
    std::unordered_set<Point, std::hash<Point>> path;
    std::unordered_set<Point, std::hash<Point>> walls;
};

template <typename Point>
bool GridMap<Point>::load(sf::Vector2u tileSize, std::vector<sf::Color> colors) {
    if(colors.size() != 5) return false;

    grid_vertices.clear();
    grid_vertices.setPrimitiveType(sf::Quads);
    grid_vertices.resize(width * height * 4);
    auto end_w = walls.end(), end_p = path.end();

    for(auto i = 0; i < width; ++i){
        for(auto j = 0; j < height; ++j){
            auto curr = i + j * width;
            sf::Color c = walls.find({i, j}) != end_w ? colors[1] : path.find({i, j}) != end_p ? colors[3] : Point{i, j} == start ? colors[2] : Point{i, j} == stop ? colors[4] : colors[0];
            sf::Vertex* q = &grid_vertices[curr * 4];

            q[0].position = sf::Vector2f(i * tileSize.x, j * tileSize.y);
            q[1].position = sf::Vector2f((i + 1) * tileSize.x - 1, j * tileSize.y);
            q[2].position = sf::Vector2f((i + 1) * tileSize.x - 1, (j + 1) * tileSize.y - 1);
            q[3].position = sf::Vector2f(i * tileSize.x, (j + 1) * tileSize.y - 1);
            q[0].color = c;
            q[1].color = c;
            q[2].color = c;
            q[3].color = c;
        }
    }

    line_row.clear();
    line_row.setPrimitiveType(sf::Lines);
    line_row.resize(height * 2);

    for(auto i = 0; i < width; ++i){
        sf::Vertex* l = &line_row[i * 2];

        l[0].position = sf::Vector2f(i * tileSize.x, 0);
        l[1].position = sf::Vector2f(i * tileSize.x, width * tileSize.x);
        l[0].color = sf::Color::Black;
        l[1].color = sf::Color::Black;
    }

    line_col.clear();
    line_col.setPrimitiveType(sf::Lines);
    line_col.resize(width * 2);

    for(auto i = 0; i < width; ++i){
        sf::Vertex* l = &line_col[i * 2];

        l[0].position = sf::Vector2f(0, i * tileSize.y);
        l[1].position = sf::Vector2f(height * tileSize.y, i * tileSize.y);
        l[0].color = sf::Color::Black;
        l[1].color = sf::Color::Black;
    }

    return true;
}

template <typename Point>
bool GridMap<Point>::add_wall(const Point &wall_coords) {
    if(walls.find(wall_coords) == walls.end()){
        walls.insert(wall_coords);

        return true;
    }

    return false;
}

template <typename Point>
void GridMap<Point>::set_point(const Point &p, sf::Color color_) {
    sf::Vertex* temp = &grid_vertices[(p.x + p.y * width) * 4];

    temp[0].color = color_;
    temp[1].color = color_;
    temp[2].color = color_;
    temp[3].color = color_;
}

template <typename Point>
bool GridMap<Point>::remove_wall(const Point &wall_cords) {
    if(walls.find(wall_cords) != walls.end()){
        walls.erase(wall_cords);

        return true;
    }

    return false;
}

template <typename Point>
void GridMap<Point>::draw(sf::RenderTarget &target, sf::RenderStates states) const {
    states.transform *= getTransform();
    target.draw(grid_vertices, states);
    target.draw(line_col, states);
    target.draw(line_row, states);
}

#endif //ASTAR_GRID_DRAWER_HPP
