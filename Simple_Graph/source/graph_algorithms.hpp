#ifndef GRAPH_GRAPH_ALGORITHMS_HPP
#define GRAPH_GRAPH_ALGORITHMS_HPP

#pragma once

#include <set>
#include "graph.hpp"


namespace ads::algo::ga {
    template<typename V, typename E>
    inline size_t getMinVertex(const ads::ds::graph::Graph<V, E>& g, std::vector<bool>& vis, std::vector<double>& keys) {
        double minKey = INF;
        int_fast32_t vertex = -1;

        for (auto i = 0; i < g.nrOfVertices(); i++) {
            if (!vis.at(i) && minKey > keys.at(i)) {
                minKey = keys.at(i);
                vertex = i;
            }
        }

        return vertex;
    }

    template<typename V, typename E>
    inline void dijkstraSimple(const ads::ds::graph::Graph<V, E>& g, size_t source) {
        auto size_ = g.nrOfVertices();
        std::vector<bool> visited(size_, false);
        std::vector<double> distance(size_, INF);
        size_t temp1;
        double temp2;
        auto nm = g.getNeighMatrix();

        distance.at(source) = 0;

        for (auto i = 0; i < size_ - 1; i++) {
            temp1 = getMinVertex(g, visited, distance);
            visited.at(temp1) = true;

            for (auto j = 0; j < size_; j++) {
                if (nm.at(temp1).at(j).value_or(INF) > 0) {
                    if (!visited.at(j) && nm.at(temp1).at(j).value_or(INF) != INF) {
                        temp2 = nm.at(temp1).at(j).value() + distance.at(temp1);

                        if (temp2 < distance.at(j)) distance.at(j) = temp2;
                    }
                }
            }
        }

        printPathDijkstra(g, source, distance);
    }

    template<typename V, typename E>
    inline void printPathDijkstra(const ads::ds::graph::Graph<V, E>& g, size_t source, std::vector<double>& keys) {
        auto v = g.getVertices();
        for (auto i = 0; i < g.nrOfVertices(); i++) {
            if (keys.at(i) != INF) std::cout << "From vertex " << v.at(source) << " to " << v.at(i) << " distance = " << keys.at(i) << std::endl;
            else std::cout << "From vertex " << v.at(source) << " couldn't reach vertex " << v.at(i) << " (no connection)" << std::endl;
        }
    }

    template<typename V, typename E>
    inline void computeFloydWarshall(const ads::ds::graph::Graph<V, E>& g) {
        auto temp = g.getNeighMatrix();
        auto size_ = g.nrOfVertices();

        for (auto k = 0; k < size_; k++) {
            temp.at(k).at(k) = 0;

            for (auto i = 0; i < size_; i++) {
                for (auto j = 0; j < size_; j++) {
                    if (temp.at(i).at(j).value_or(INF) > temp.at(i).at(k).value_or(INF) + temp.at(k).at(j).value_or(INF)) {
                        temp.at(i).at(j) = temp.at(i).at(k).value_or(INF) + temp.at(k).at(j).value_or(INF);
                    }
                }
            }
        }

        printPathFloydWarshall(temp);
    }

    template<typename V, typename E>
    inline void printPathFloydWarshall(const ads::ds::graph::Graph<V, E>& g, std::vector<std::vector<std::optional<E>>>& input) {
        std::cout << "Shortest paths:" << std::endl;
        auto size_ = g.nrOfVertices();
        auto v = g.getVertices();

        for (auto i = 0; i < size_; i++) {
            for (auto j = 0; j < size_; j++) {
                std::cout << "From " << v.at(i) << " to " << v.at(j) << " = ";

                if (input.at(i).at(j).value_or(INF) == INF) std::cout << "NO PATH";
                else std::cout << input.at(i).at(j).value_or(INF);

                std::cout << std::endl;
            }
        }
    }

    inline bool checkEdge(size_t u, size_t v, std::vector<bool>& vis) {
        if (u == v || (!vis.at(u) && !vis.at(v))) return false;
        else return !(vis.at(u) && vis.at(v));
    }

    template<typename V, typename E>
    inline void primsMST(const ads::ds::graph::Graph<V, E>& g) {
        std::vector<bool> visited(g.nrOfVertices(), false);
        size_t counter = 0;
        double min_cost = 0;
        auto size_ = g.nrOfVertices();
        auto nm = g.getNeighMatrix();

        visited.at(0) = true;

        while (counter < size_ - 1) {
            auto min = INF;
            auto a = -1;
            auto b = -1;

            for (auto i = 0; i < size_; i++) {
                for (auto j = 0; j < size_; j++) {
                    if (nm.at(i).at(j).value_or(INF) < min) {
                        if (checkEdge(i, j, visited)) {
                            min = nm.at(i).at(j).value_or(INF);
                            a = i;
                            b = j;
                        }
                    }
                }
            }

            if (a != -1 && b != -1) {
                std::cout << "Edge: " << counter++ << " [" << a << ", " << b << "] cost: " << min << std::endl;
                min_cost += min;
                visited.at(a) = true;
                visited.at(b) = true;
            }
        }

        std::cout << "Minimum cost of MST = " << min_cost << std::endl;
    }

    template<typename V, typename E>
    inline bool isValidForColors(const ads::ds::graph::Graph<V, E>& g, size_t vertex_id, std::vector<size_t>& col, size_t col_checker) {
        auto nm = g.getNeighMatrix();

        for (auto i = 0; i < g.nrOfVertices(); i++) if (nm.at(vertex_id).at(i) && col_checker == col.at(i)) return false;

        return true;
    }

    template<typename V, typename E>
    inline bool tryGraphColoring(const ads::ds::graph::Graph<V, E>& g, size_t nrOfColorsToTry, size_t vertex_id, std::vector<size_t>& col) {
        auto size_ = g.nrOfVertices();

        if (vertex_id == size_) return true;

        for (auto i = 1; i <= size_; i++) {
            if (isValidForColors(g, vertex_id, col, i)) {
                col.at(vertex_id) = i;

                if (tryGraphColoring(g, nrOfColorsToTry, vertex_id + 1, col)) return true;

                col.at(vertex_id) = 0;
            }
        }

        return false;
    }

    template<typename V, typename E>
    inline bool checkColoringResult(const ads::ds::graph::Graph<V, E>& g, size_t nrOfColorsToTry) {
        std::vector<size_t> colors(g.nrOfVertices(), 0);

        if (!tryGraphColoring(g, nrOfColorsToTry, 0, colors)) {
            std::cout << "Couldn't find solution for " << nrOfColorsToTry << " colors" << std::endl;

            return false;
        }
        else {
            printColors(g, colors);

            return true;
        }
    }

    template<typename V, typename E>
    inline std::pair<size_t, bool>  colorUntilDone(const ads::ds::graph::Graph<V, E>& g) {
        auto size_ = g.nrOfVertices();
        std::vector<size_t> colors(size_, 0);
        auto chromaticNumber = 1;

        while (!tryGraphColoring(g, chromaticNumber, 0, colors)) chromaticNumber++;

        printColors(g, colors);

        return std::make_pair(chromaticNumber, (chromaticNumber < size_));
    }

    template<typename V, typename E>
    inline void printColors(const ads::ds::graph::Graph<V, E>& g, std::vector<size_t>& col) {
        std::cout << "Solution for coloring vertices:" << std::endl;
        auto v = g.getVertices();

        for (auto i = 0; i < g.nrOfVertices(); i++) std::cout << v.at(i) << " color: " << col.at(i) << std::endl;
    }

    template<typename V, typename E>
    inline bool checkVerticesDFS_undirected(const ads::ds::graph::Graph<V, E>& g, size_t curr, std::set<size_t>& wSet, std::set<size_t>& gSet, std::set<size_t>& bSet) {
        wSet.erase(wSet.find(curr));
        gSet.insert(curr);
        auto nm = g.getNeighMatrix();

        for (auto i = 0; i < g.nrOfVertices(); i++) {
            if (nm.at(curr).at(i)) {
                if (bSet.find(i) != bSet.end()) continue;

                if (gSet.find(i) != gSet.end()) return true;

                if (checkVerticesDFS_undirected(g, i, wSet, gSet, bSet)) return true;
            }
        }

        gSet.erase(gSet.find(curr));
        bSet.insert(curr);

        return false;
    }

    template<typename V, typename E>
    inline bool hasCycle_undirected(const ads::ds::graph::Graph<V, E>& g) {
        std::set<size_t> whiteSet, greySey, blackSet;
        auto size_ = g.nrOfVertices();

        for (auto i = 0; i < size_; i++) whiteSet.insert(i);

        while (!whiteSet.empty()) {
            for (auto i = 0; i < size_; i++) {
                if (whiteSet.find(i) != whiteSet.end()) {
                    if (checkVerticesDFS_undirected(g, i, whiteSet, greySey, blackSet)) return true;
                }
            }
        }

        return false;
    }

    template<typename V, typename E>
    inline bool checkVerticesDFS_directed(const ads::ds::graph::Graph<V, E>& g, size_t curr, std::set<size_t>& vis, size_t parent) {
        vis.insert(curr);
        auto nm = g.getNeighMatrix();

        for (auto i = 0; i < g.nrOfVertices(); i++) {
            if (nm.at(curr).at(i)) {
                if (i == parent) continue;

                if (vis.find(i) != vis.end()) return true;

                if (checkVerticesDFS_directed(g, i, vis, curr)) return true;
            }
        }

        return false;
    }

    template<typename V, typename E>
    inline bool hasCycle_directed(const ads::ds::graph::Graph<V, E>& g) {
        std::set<size_t> checker;

        for (auto i = 0; i < g.nrOfVertices(); i++) {
            if (checker.find(i) != checker.end()) continue;

            if (checkVerticesDFS_directed(g, i, checker, -1)) return true;
        }

        return false;
    }

    template<typename V, typename E>
    inline bool isSafeVertex(const ads::ds::graph::Graph<V, E>& g, size_t vertex_id, std::vector<int64_t>& path, size_t pos) {
        if (!g.getNeighMatrix().at(path.at(pos - 1)).at(vertex_id)) return false;

        for (auto i = 0; i < pos; i++) if (path.at(i) == vertex_id) return false;

        return true;
    }

    template<typename V, typename E>
    inline bool tryFindingHamCycle(const ads::ds::graph::Graph<V, E>& g, std::vector<int64_t>& path, size_t pos) {
        if (pos == g.nrOfVertices()) return (g.getNeighMatrix().at(path.at(pos - 1)).at(path.at(0)) ? true : false);

        for (auto i = 1; i < g.nrOfVertices(); i++) {
            if (isSafeVertex(g, i, path, pos)) {
                path.at(pos) = i;

                if (tryFindingHamCycle(g, path, pos + 1)) return true;

                path.at(pos) = -1;
            }
        }

        return false;
    }

    template<typename V, typename E>
    inline bool hasHamiltonCycle(const ads::ds::graph::Graph<V, E>& g, size_t startingVertex) {
        std::vector<int64_t> path(g.nrOfVertices(), -1);
        path.at(0) = 0;

        if (!tryFindingHamCycle(g, path, startingVertex)) {
            std::cout << "This graph doesn't have Hamilton cycles" << std::endl;

            return false;
        }
        else {
            printHamiltonCycle(path, g.nrOfVertices());

            return true;
        }
    }

    inline void printHamiltonCycle(std::vector<int64_t>& path, const size_t& nr_of_vertices) {
        std::cout << "Hamilton cycle:" << std::endl;

        for (auto i = 0; i < nr_of_vertices; i++) std::cout << path.at(i) << ", ";
        std::cout << path.at(0) << std::endl;
    }

    template<typename V, typename E>
    inline int64_t getInDegree(const ads::ds::graph::Graph<V, E>& g, size_t vertex_id) {
        if (vertex_id >= g.nrOfVertices() || vertex_id < 0) return -1;
        else {
            int64_t degree = 0;
            auto nm = g.getNeighMatrix();

            for (auto i = 0; i < g.nrOfVertices(); i++) if (nm.at(i).at(vertex_id)) degree++;

            return degree;
        }
    }

    template<typename V, typename E>
    inline int64_t getOutDegree(const ads::ds::graph::Graph<V, E>& g, size_t vertex_id) {
        if (vertex_id >= g.nrOfVertices() || vertex_id < 0) return -1;
        else {
            int64_t degree = 0;
            auto nm = g.getNeighMatrix();

            for (auto i = 0; i < g.nrOfVertices(); i++) if (nm.at(vertex_id).at(i)) degree++;

            return degree;
        }
    }

    template<typename V, typename E>
    inline typename ads::ds::graph::Graph<V, E>::vertices_t getOutConnectedVerticesTo(const ads::ds::graph::Graph<V, E>& g, size_t vertex_id) {
        ads::ds::graph::Graph<V, E>::vertices_t result;
        auto nm = g.getNeighMatrix();
        auto v = g.getVertices();

        for (auto i = 0; i < g.nrOfVertices(); i++) {
            if (nm.at(vertex_id).at(i)) result.push_back(v.at(i));
        }

        return result;
    }

    template<typename V, typename E>
    inline typename ads::ds::graph::Graph<V, E>::vertices_t getInConnectedVerticesTo(const ads::ds::graph::Graph<V, E>& g, size_t vertex_id) {
        ads::ds::graph::Graph<V, E>::vertices_t result;
        auto nm = g.getNeighMatrix();
        auto v = g.getVertices();

        for (auto i = 0; i < g.nrOfVertices(); i++) {
            if (nm.at(i).at(vertex_id)) result.push_back(v.at(i));
        }

        return result;
    }

}

#endif