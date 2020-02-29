#ifndef GRAPH_ASTAR_HPP
#define GRAPH_ASTAR_HPP

#pragma once

#include <cstdint>
#include <utility>
#include <vector>
#include <functional>
#include <vector>
#include <queue>
#include <tuple>
#include <algorithm>

#include "graph.hpp"

template<typename T, typename priority_t>
struct PriorityQueue{
    typedef std::pair<priority_t, T> PQElement;
    std::priority_queue<PQElement, std::vector<PQElement>, std::greater<>> elements;

    [[nodiscard]] inline bool empty()      const { return elements.empty(); }
    inline void put(T item, priority_t priority) { elements.emplace(priority, item); }
    T get()                                      { auto best_item = elements.top().second; elements.pop(); return best_item; }
};

template<typename V, typename E>
std::pair<double, std::vector<std::size_t>> astar(Graph<V, E> &graph, std::size_t start_idx, std::size_t end_idx, std::function<double(const Graph<V, E>&, std::size_t actual_vertex_id, std::size_t end_vertex_id)> heuristics, std::function<double(const E&)> getEdgeLength){
    auto size_ = graph.nrOfVertices();
    std::unordered_map<size_t, double> distances;   // vertex - distance
    std::unordered_map<size_t, size_t> previous;
    std::vector<size_t> path;
    PriorityQueue<std::size_t, double> frontier;
    auto tempMatrix = graph.getNeighMatrix();

    frontier.put(start_idx, 0);
    distances[start_idx] = 0;

    while(!frontier.empty()){
        auto current = frontier.get();

        if(current == end_idx){
            while(previous.find(current) != previous.end()){
                path.push_back(current);
                current = previous[current];
            }

            break;
        }

        if(distances[current] == INF) break;

        for(auto i = 0; i < size_; i++){
            auto alt = distances[current] + getEdgeLength(tempMatrix.at(current).at(i).value_or(INF));

            if(distances.find(i) == distances.end() || alt < distances[i]){
                distances[i] = alt;
                previous[i] = current;
                frontier.put(i, alt+heuristics(graph, current, end_idx));
            }
        }
    }

    if(distances[end_idx] != INF) {
        path.push_back(start_idx);
        std::reverse(path.begin(), path.end());

        return std::make_pair(distances[end_idx], path);
    }
    else return std::make_pair(0, path);
}

#endif //GRAPH_ASTAR_HPP
