#ifndef ASTAR_ASTAR_HPP
#define ASTAR_ASTAR_HPP

#pragma once
#include <vector>
#include <functional>
#include <set>
#include <algorithm>
#include <cmath>

class AStar{
        struct Coords{
            ssize_t x;
            ssize_t y;

            bool   operator==(const Coords& cords) const { return (x == cords.x && y == cords.y); };
            bool   operator!=(const Coords& cords) const { return !(*this == cords); };
            Coords operator+ (const Coords &other) const { return { x + other.x, y + other.y }; };
            Coords operator- (const Coords &other) const { return { x - other.x, y - other.y }; };
        };

        struct TileNode{
            ssize_t   G;
            ssize_t   H;
            Coords    coordinates;
            TileNode *parent;

            TileNode()                                                         : parent(nullptr), coordinates({-1, -1}), G(0), H(0){};
            explicit TileNode(Coords coord_)                                   : parent(nullptr), coordinates(coord_), G(0), H(0){};
            explicit TileNode(TileNode * parent_)                              : parent(parent_), coordinates({-1, -1}), G(0), H(0){};
            TileNode(Coords coord_, TileNode *parent_)                         : parent(parent_), coordinates(coord_), G(0), H(0){};
            TileNode(Coords coord_, TileNode *parent_, ssize_t g_, ssize_t h_) : parent(parent_), coordinates(coord_), G(g_), H(h_){};
            TileNode(const TileNode &source)                                   = default;;
            TileNode(TileNode &&source) noexcept                               : parent(source.parent), coordinates(source.coordinates), G(source.G), H(source.H){};

            inline ssize_t getScore() const { return (G + H); };
        };

        inline bool             detectCollision(Coords coordinates_)                       const { return (coordinates_.x < 0 || coordinates_.x >= worldSize.x || coordinates_.y < 0 || coordinates_.y >= worldSize.y || std::find(walls.begin(), walls.end(), coordinates_) != walls.end()); };
        inline static void      releaseNodes(std::set<TileNode*>& nodes_)                        { for(auto it = nodes_.begin(); it != nodes_.end();){ delete *it; it = nodes_.erase(it); } };
        inline static TileNode* findNodeOnList(std::set<TileNode*>& nodes_, Coords coordinates_) { for(auto node : nodes_){ if(node->coordinates == coordinates_){ return node; } } return nullptr; };

    public:
        class HeuristicFunctionType{
            private:
                inline static Coords getDelta(Coords source_, Coords target_)  { return{ std::abs(source_.x - target_.x), std::abs(source_.y - target_.y) }; };

            public:
                inline static size_t manhattan(Coords source_, Coords target_) { auto delta = getDelta(source_, target_); return static_cast<size_t>(10 * (delta.x + delta.y)); };
                inline static size_t euclidean(Coords source_, Coords target_) { auto delta = getDelta(source_, target_); return static_cast<size_t>(10 * std::sqrt((delta.x * delta.x) + (delta.y * delta.y))); };
                inline static size_t octagonal(Coords source_, Coords target_) { auto delta = getDelta(source_, target_); return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y); };
        };

        AStar() : direction({ {0, 1}, {1, 0}, {0, -1}, {-1, 0}, {-1, -1}, {1, 1}, {-1, 1}, {1, -1} }), worldSize({10, 10}), directions(4){ setDiagonalMovement(false); setHeuristic(&HeuristicFunctionType::manhattan); };

        inline void                setWorldSize(Coords worldSize_)                                { worldSize = worldSize_; };
        inline void                setDiagonalMovement(bool enable_)                              { directions = (enable_ ? 8 : 4); };
        inline void                setHeuristic(std::function<size_t(Coords, Coords)> heuristic_) { heuristic = std::bind(heuristic_, std::placeholders::_1, std::placeholders::_2); };
        inline void                addCollision(Coords coordinates_)                              { if(coordinates_.x < worldSize.x && coordinates_.y < worldSize.y){ walls.push_back(coordinates_); } };
        inline void                clearCollisions()                                              { walls.clear(); };
        inline void                removeCollision(Coords coordinates_)                           { auto it = std::find(walls.begin(), walls.end(), coordinates_); if(it != walls.end()) walls.erase(it); };
        inline std::vector<Coords> findPath(Coords source_, Coords target_) const;
        inline void                printWorld() const;

    private:
        std::function<size_t(Coords, Coords)> heuristic;
        std::vector<Coords>                   direction;
        std::vector<Coords>                   walls;
        Coords                                worldSize;
        size_t                                directions;
};

inline std::vector<AStar::Coords> AStar::findPath(Coords source_, Coords target_) const{
    TileNode *current = nullptr;
    std::set<TileNode*> openSet;
    std::set<TileNode*> closedSet;
    openSet.insert(new TileNode(source_));

    while(!openSet.empty()){
        current = *openSet.begin();

        for(auto node : openSet) if(node->getScore() <= current->getScore()) current = node;

        if(current->coordinates == target_) break;

        closedSet.insert(current);
        openSet.erase(std::find(openSet.begin(), openSet.end(), current));

        for(auto i = 0; i < directions; ++i){
            Coords newCoordinates(current->coordinates + direction[i]);

            if(detectCollision(newCoordinates) || findNodeOnList(closedSet, newCoordinates)) continue;

            size_t totalCost = current->G + ((i < 4) ? 10 : 14);
            TileNode *successor = findNodeOnList(openSet, newCoordinates);

            if(successor == nullptr) {
                successor = new TileNode(newCoordinates, current);
                successor->G = totalCost;
                successor->H = heuristic(successor->coordinates, target_);
                openSet.insert(successor);
            }
            else if(totalCost < successor->G){
                successor->parent = current;
                successor->G = totalCost;
            }
        }
    }

    std::vector<AStar::Coords> path;

    while(current != nullptr) {
        path.push_back(current->coordinates);
        current = current->parent;
    }

    releaseNodes(openSet);
    releaseNodes(closedSet);

    return path;
}

void AStar::printWorld() const{
    auto k = 0;
    auto wSize = walls.size();
    for(auto i = 0; i < worldSize.x; i++){
        for(auto j = 0; j < worldSize.y; j++){
            if(k < wSize){
                if(walls.at(k).x == i && walls.at(k).y == j){
                    k++;
                    std::cout << "W ";
                }
                else std::cout << "O ";
            }
            else std::cout << "O ";
        }
        std::cout << std::endl;
    }
}

#endif //ASTAR_ASTAR_HPP
