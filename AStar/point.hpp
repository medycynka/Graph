#ifndef ASTAR_POINT_HPP
#define ASTAR_POINT_HPP

#pragma once

#include <functional>


struct P{
    ssize_t x;
    ssize_t y;
};

bool operator==(const P &lhs, const P &rhs){
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator!=(const P &lhs, const P &rhs){
    return !(lhs == rhs);
}

template <> struct std::hash<P>{
    std::size_t operator()(P const& p_) const noexcept{
        std::size_t h1 = std::hash<ssize_t>{}(p_.x);
        std::size_t h2 = std::hash<ssize_t>{}(p_.y);
        return h1 ^ (h2 << 1);
    };
};

#endif //ASTAR_POINT_HPP
