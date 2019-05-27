#include <iostream>
#include "AStar.hpp"

int main() {
    AStar g;
    g.setWorldSize({15, 15});
    g.setHeuristic(AStar::HeuristicFunctionType::euclidean);
    g.setDiagonalMovement(true);
    g.addCollision({1, 0});
    g.addCollision({2, 0});
    g.addCollision({2, 10});
    g.addCollision({3, 0});
    g.addCollision({3, 1});
    g.addCollision({3, 2});
    g.addCollision({3, 3});
    g.addCollision({3, 4});
    g.addCollision({3, 5});
    g.addCollision({3, 10});
    g.addCollision({4, 5});
    g.addCollision({4, 10});
    g.addCollision({5, 5});
    g.addCollision({5, 6});
    g.addCollision({5, 7});
    g.addCollision({5, 8});
    g.addCollision({5, 9});
    g.addCollision({5, 10});
    std::cout << std::endl;
    g.printWorld();
    std::cout << std::endl;

    auto path = g.findPath({0, 0}, {14, 12});

    std::cout << "Generated Path:" << std::endl;
    for(auto it = path.rbegin(); it != path.rend(); ++it) std::cout << "(" << (*it).x << ", " << (*it).y << ")" << std::endl;

    return 0;
}