#include <iostream>
#include "AStar.hpp"

int main() {
    AStar<bool> g;
    g.setWorldSize({20, 20});
    g.setHeuristic(AStar<bool>::HeuristicFunctionType::euclidean);
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
    auto path = g.findPath({0, 0}, {19, 4});
    std::cout << "Generated Path:" << std::endl;
    for(auto & it : path) std::cout << "(" << it.x << ", " << it.y << ")" << std::endl;

    std::cout << std::endl << std::endl << std::endl;

    AStar<char> c;
    c.setWorldSize({15, 15});
    c.setDiagonalMovement(false);
    c.setHeuristic(AStar<char>::HeuristicFunctionType::euclidean);
    std::vector<std::vector<char>> walls_ = { {'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w'},
                                              {'w', 'o', 'o', 'o', 'w', 'o', 'o', 'o', 'o', 'o', 'o', 'o', 'o', 'o', 'w'},
                                              {'w', 'o', 'o', 'o', 'w', 'o', 'o', 'o', 'o', 'o', 'w', 'w', 'w', 'o', 'w'},
                                              {'w', 'o', 'o', 'o', 'w', 'o', 'o', 'o', 'o', 'w', 'w', 'o', 'o', 'o', 'w'},
                                              {'w', 'o', 'o', 'o', 'w', 'o', 'o', 'o', 'o', 'w', 'w', 'o', 'o', 'o', 'w'},
                                              {'w', 'o', 'o', 'o', 'w', 'o', 'o', 'o', 'o', 'w', 'w', 'o', 'w', 'w', 'w'},
                                              {'w', 'o', 'o', 'o', 'w', 'o', 'w', 'w', 'o', 'w', 'o', 'o', 'w', 'w', 'w'},
                                              {'w', 'o', 'o', 'o', 'w', 'o', 'w', 'o', 'o', 'w', 'o', 'w', 'w', 'w', 'w'},
                                              {'w', 'o', 'o', 'o', 'w', 'o', 'w', 'o', 'o', 'w', 'o', 'w', 'w', 'w', 'w'},
                                              {'w', 'o', 'o', 'o', 'w', 'o', 'w', 'o', 'o', 'w', 'o', 'o', 'o', 'o', 'w'},
                                              {'w', 'o', 'o', 'o', 'w', 'w', 'w', 'o', 'o', 'w', 'o', 'w', 'o', 'o', 'w'},
                                              {'w', 'o', 'o', 'o', 'o', 'o', 'o', 'o', 'o', 'w', 'o', 'w', 'o', 'o', 'w'},
                                              {'w', 'o', 'o', 'o', 'o', 'o', 'o', 'o', 'o', 'w', 'o', 'w', 'o', 'o', 'w'},
                                              {'w', 'o', 'o', 'o', 'o', 'o', 'o', 'o', 'o', 'w', 'o', 'w', 'o', 'o', 'w'},
                                              {'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w', 'w'}
                                            };
    c.addCollisionsFromVectorMatrix(walls_, [](const char &e)->bool{ return (e == 'w'); });
    c.printWorld();
    std::cout << std::endl;
    auto path2 = c.findPath({1, 1}, {13, 12});
    std::cout << "Generated Path:" << std::endl;
    for(auto & i : path2) std::cout << "(" << i.x << ", " << i.y << ")" << std::endl;

    std::cout << std::endl << std::endl << std::endl;
    c.printWorldWithPath(path2);

    return 0;
}