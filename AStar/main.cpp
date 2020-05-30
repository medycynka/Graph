#include <iostream>
#include "astar.hpp"
#include "grid_drawer.hpp"
#include "maze.hpp"


constexpr const int TILE_WIDTH{20};
constexpr const int TILE_HEIGHT{20};
constexpr const int NR_TILE_X{41};
constexpr const int NR_TILE_Y{41};
constexpr const int WIDTH{TILE_WIDTH * NR_TILE_X};
constexpr const int HEIGHT{TILE_HEIGHT * NR_TILE_Y};
constexpr const std::size_t num_cols = {(NR_TILE_X - 1) / 2};
constexpr const std::size_t num_rows = {(NR_TILE_Y - 1) / 2};


int main() {
    constexpr auto maze = make_maze<num_cols, num_rows>();
    constexpr auto rendered_maze = render_maze(maze);
    std::unordered_set<P, std::hash<P>> walls;

    for (std::size_t row = 0; row < num_rows*2 + 1; ++row){
        for (std::size_t col = 0; col < num_cols*2 + 1; ++col){
            const auto square = [&](){
                switch (rendered_maze(col, row)) {
                    case WallType::Empty:      return " ";
                    case WallType::UpperLeft:  walls.insert({ssize_t(row), ssize_t(col)}); return "┌";
                    case WallType::Vertical:   walls.insert({ssize_t(row), ssize_t(col)}); return "│";
                    case WallType::Horizontal: walls.insert({ssize_t(row), ssize_t(col)}); return "─";
                    case WallType::UpperRight: walls.insert({ssize_t(row), ssize_t(col)}); return "┐";
                    case WallType::LowerLeft:  walls.insert({ssize_t(row), ssize_t(col)}); return "└";
                    case WallType::LowerRight: walls.insert({ssize_t(row), ssize_t(col)}); return "┘";
                    case WallType::RightTee:   walls.insert({ssize_t(row), ssize_t(col)}); return "├";
                    case WallType::LeftTee:    walls.insert({ssize_t(row), ssize_t(col)}); return "┤";
                    case WallType::UpTee:      walls.insert({ssize_t(row), ssize_t(col)}); return "┴";
                    case WallType::DownTee:    walls.insert({ssize_t(row), ssize_t(col)}); return "┬";
                    case WallType::FourWay:    walls.insert({ssize_t(row), ssize_t(col)}); return "┼";
                    case WallType::Up:         walls.insert({ssize_t(row), ssize_t(col)}); return "╵";
                    case WallType::Down:       walls.insert({ssize_t(row), ssize_t(col)}); return "╷";
                    case WallType::Left:       walls.insert({ssize_t(row), ssize_t(col)}); return "╴";
                    case WallType::Right:      walls.insert({ssize_t(row), ssize_t(col)}); return "╶";
                    case WallType::Visited:    return "·";
                    case WallType::Used:       return "*";
                    default:                   return "";
                }
            }();

            std::cout << square;
        }
        std::cout << '\n';
    }

    std::cout << "\n\n\n";

    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Maze solver");
    window.setVerticalSyncEnabled(true);
    window.setFramerateLimit(60);

    std::vector<sf::Color> c_ = {sf::Color(219, 219, 219), sf::Color(32, 32, 32), sf::Color::Green, sf::Color(252, 252, 144), sf::Color::Red};

    P s_ = {-1, -1};
    P e_ = {-1, -1};

    GridMap gm;
    gm.set_width(NR_TILE_X);
    gm.set_height(NR_TILE_Y);
    gm.update_wall(walls);
    gm.set_start(s_);
    gm.set_stop(e_);

    AStar algo({NR_TILE_X, NR_TILE_Y});
    algo.addWalls(walls);

    if(!gm.load(sf::Vector2u(TILE_WIDTH, TILE_HEIGHT), c_)){
        std::cerr << "Couldn't load map, check your colors" << std::endl;

        return -1;
    }

    while(window.isOpen()){
        sf::Event e;

        while(window.pollEvent(e)) {
            switch(e.type){
                case sf::Event::Closed:
                    window.close();
                    break;
                case sf::Event::KeyPressed:
                    if(sf::Keyboard::isKeyPressed(sf::Keyboard::Escape) || sf::Keyboard::isKeyPressed(sf::Keyboard::Q)){
                        window.close();
                    } else if(sf::Keyboard::isKeyPressed(sf::Keyboard::C)){
                        gm.clear();
                        s_.x = -1;
                        s_.y = -1;
                        e_.x = -1;
                        e_.y = -1;

                        if(!gm.load(sf::Vector2u(TILE_WIDTH, TILE_HEIGHT), c_)){
                            std::cerr << "Couldn't load map, check your colors" << std::endl;

                            window.close();
                        }
                    } else if(sf::Keyboard::isKeyPressed(sf::Keyboard::Space)){
                        if(s_ != P{-1, -1} && e_ != P{-1, -1}) {
                            auto p = algo.get_set_from_path(algo.findPath({s_.x, s_.y}, {e_.x, e_.y}));
                            p.erase(p.find(s_));
                            p.erase(p.find(e_));

                            gm.update_path(p);

                            if (!gm.load(sf::Vector2u(TILE_WIDTH, TILE_HEIGHT), c_)) {
                                std::cerr << "Couldn't load map, check your colors" << std::endl;

                                window.close();
                            }

                            gm.set_point(s_, c_[2]);
                            gm.set_point(e_, c_[4]);
                        }
                    }

                    break;
                case sf::Event::MouseButtonPressed:
                    if(e.mouseButton.button == sf::Mouse::Left){
                        sf::Vector2i mouse = sf::Mouse::getPosition(window);
                        P pom{int(mouse.x / TILE_WIDTH), int(mouse.y / TILE_HEIGHT)};
                        std::cout << pom.x << ", " << pom.y << std::endl;

                        if(sf::Keyboard::isKeyPressed(sf::Keyboard::LControl)) {
                            if (gm.in_walls(pom)) {
                                gm.set_point(pom, c_[0]);
                                gm.remove_wall(pom);
                                algo.removeCollision({pom.x, pom.y});
                            } else{
                                gm.add_wall(pom);
                                gm.set_point(pom, c_[1]);
                                algo.addCollision({pom.x, pom.y});
                            }
                        } else {
                            if (s_ == P{-1, -1} && !gm.in_walls(pom)) {
                                gm.set_start(s_);
                                gm.set_point(pom, c_[2]);
                                s_.x = pom.x;
                                s_.y = pom.y;
                            } else if (s_ == pom) {
                                gm.set_start({-1, -1});
                                gm.set_point(pom, c_[0]);
                                s_.x = -1;
                                s_.y = -1;
                            }
                        }
                    } else if(e.mouseButton.button == sf::Mouse::Right){
                        sf::Vector2i mouse = sf::Mouse::getPosition(window);
                        P pom{int(mouse.x / TILE_WIDTH), int(mouse.y / TILE_HEIGHT)};
                        std::cout << pom.x << ", " << pom.y << std::endl;

                        if(e_ == P{-1, -1} && !gm.in_walls(pom)){
                            gm.set_stop(pom);
                            gm.set_point(pom, c_[4]);
                            e_.x = pom.x;
                            e_.y = pom.y;
                        } else if(e_ == pom){
                            gm.set_stop({-1, -1});
                            gm.set_point(pom, c_[0]);
                            e_.x = -1;
                            e_.y = -1;
                        }
                    }

                    break;
                default:
                    break;
            }
        }

        window.clear();
        window.draw(gm);
        window.display();
    }

    return 0;
}