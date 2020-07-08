#ifndef GRAPH_GRAPH_HPP
#define GRAPH_GRAPH_HPP

#pragma onceedge_iterator

#include <iostream>
#include <functional>
#include <limits>
#include <iomanip>
#include <exception>
#include <initializer_list>
#include "vertice_iterator.hpp"
#include "edge_iterator.hpp"
#include "dfs_iterator.hpp"
#include "bfs_iterator.hpp"

namespace ads::ds::graph {

    const double INF = std::numeric_limits<double>::max();

    template <typename V, typename E>
    class Graph {
    public:
        typedef std::vector<V>                                 vertices_t;
        typedef std::vector<std::vector<std::optional<E>>>     matrix_t;
        typedef ads::ds::graph::iterators::VerticesIterator<V> vertex_iterator;
        typedef ads::ds::graph::iterators::EdgesIterator<E>    edge_iterator;
        typedef ads::ds::graph::iterators::dfsIterator<V, E>   dfs_iterator;
        typedef ads::ds::graph::iterators::bfsIterator<V, E>   bfs_iterator;

        Graph()                            = default;
        Graph(const Graph&)                = default;
        Graph& operator=(const Graph&)     = default;
        Graph(Graph&&)            noexcept = default;
        Graph& operator=(Graph&&) noexcept = default;
        explicit Graph(vertices_t& in)                       : vertices{ in } {};
        Graph(vertices_t& in_v, matrix_t& in_m, size_t in_e) : vertices{ in_v }, neigh_matrix{ in_m }, no_of_edges{ in_e } {};
        Graph(std::initializer_list<V> init) { for (auto& e : init) { insertVertex(e); } };
        Graph(std::pair<std::initializer_list<V>, std::initializer_list<std::pair<std::pair<size_t, size_t>, E>>>& init);

        bool         operator==(const Graph& other)                     { return (no_of_edges == other.no_of_edges && vertices == other.vertices && neigh_matrix == other.neigh_matrix); };
        bool         operator!=(const Graph& other)                     { return !(*this == other); };
        Graph<V, E>& operator+=(const V& new_vertex)                    { insertVertex(new_vertex); return *this; };
        Graph<V, E>& operator-=(const V& new_vertex)                    { removeVertex(new_vertex); return *this; };
        Graph<V, E>& operator+=(const std::pair<std::pair<V, V>, E>& e) { insertEdge(e.first.first, e.first.second, e.second); return *this; };
        Graph<V, E>& operator-=(const std::pair<V, V>& p)               { removeEdge(p.first, p.second); return *this; };

        vertex_iterator                   insertVertex(const V& vertex_data);
        std::pair<edge_iterator, bool>    insertEdge(size_t vertex1_id, size_t vertex2_id, const E& label = E(), bool replace = true);
        vertex_iterator                   removeVertex(size_t vertex_id);
        edge_iterator                     removeEdge(size_t vertex1_id, size_t vertex2_id);
        void                              printNeighborhoodMatrix() const;
        void                              dfs(size_t start, std::function<void(const V&)> visitator_f);
        vertices_t                        dfs(size_t start);
        void                              bfs(size_t start, std::function<void(const V&)> visitator_f);
        vertices_t                        bfs(size_t start);
        [[nodiscard]] bool                edgeExist(size_t vertex1_id, size_t vertex2_id)           const { return neigh_matrix[vertex1_id][vertex2_id]; };
        [[nodiscard]] size_t              nrOfVertices()                                            const { return vertices.size(); };
        [[nodiscard]] size_t              nrOfEdges()                                               const { return no_of_edges; };
        vertex_iterator                   vertex(std::size_t vertex_id)                                   { return (vertex_id < nrOfVertices() ? vertex_iterator(vertices, vertex_id) : endVertices()); };
        edge_iterator                     edge(std::size_t vertex1_id, std::size_t vertex2_id)            { return (neigh_matrix.at(vertex1_id).at(vertex2_id) ? edge_iterator(neigh_matrix, vertex1_id, vertex2_id) : endEdges()); };
        matrix_t&                         getNeighMatrix()                                                { return neigh_matrix; };
        vertices_t&                       getVertices()                                                   { return vertices; };
        [[nodiscard]] std::vector<size_t> getNeighbours(size_t id)                                  const { std::vector<size_t> ret; for (auto i = 0; i < nrOfVertices(); i++) { if (neigh_matrix.at(id).at(i)) { ret.push_back(i); } } return ret; }
        V&                                vertexData(size_t id) { if (id < 0 || id > nrOfVertices())      { throw std::out_of_range("Wrong id"); } else { return vertices.at(id); } };
        const V&                          vertexData(size_t id)                                     const { if (id < 0 || id > nrOfVertices()) { throw std::out_of_range("Wrong id"); } else { return vertices.at(id); } };
        const E&                          edgeLabel(std::size_t vertex1_id, std::size_t vertex2_id) const { if (neigh_matrix.at(vertex1_id).at(vertex2_id)) { return neigh_matrix.at(vertex1_id).at(vertex2_id).value(); } else { throw std::bad_optional_access(); } };
        E&                                edgeLabel(std::size_t vertex1_id, std::size_t vertex2_id)       { if (neigh_matrix.at(vertex1_id).at(vertex2_id)) { return neigh_matrix.at(vertex1_id).at(vertex2_id).value(); } else { throw std::bad_optional_access(); } };

        vertex_iterator begin()                          { return beginVertices(); };
        vertex_iterator end()                            { return endVertices(); };
        vertex_iterator beginVertices()                  { return vertex_iterator(vertices, 0); };
        vertex_iterator endVertices()                    { return vertex_iterator(vertices, vertices.size()); };
        edge_iterator   beginEdges()                     { return edge_iterator(neigh_matrix); };
        edge_iterator   endEdges()                       { return edge_iterator(neigh_matrix, neigh_matrix.size(), 0); };
        dfs_iterator    beginDFS(size_t starting_vertex) { return (starting_vertex < vertices.size() ? dfs_iterator(vertices, neigh_matrix, starting_vertex) : endDFS()); };
        dfs_iterator    endDFS()                         { return dfs_iterator(vertices, neigh_matrix, nrOfVertices()); };
        bfs_iterator    beginBFS(size_t starting_vertex) { return (starting_vertex < vertices.size() ? bfs_iterator(vertices, neigh_matrix, starting_vertex) : endBFS()); };
        bfs_iterator    endBFS()                         { return bfs_iterator(vertices, neigh_matrix, nrOfVertices()); };

        friend std::ostream& operator<<(std::ostream& ofs, Graph<V, E>& g) {
            ofs << "Number of vertices: " << g.nrOfVertices() << "\n" << "Vertices: ";

            for (auto it = g.beginVertices(); it != g.endVertices(); ++it) {
                ofs << *it << ", ";
            }

            ofs << "\n" << "Number of edges: " << g.nrOfEdges() << "\n" << "Edges: ";

            for (auto it = g.beginEdges(); it != g.endEdges(); ++it) {
                ofs << *it << ", ";
            }

            ofs << "\n";

            return ofs;
        }

    private:
        vertices_t vertices;
        matrix_t neigh_matrix;
        size_t no_of_edges = 0;
    };

    template<typename V, typename E>
    Graph<V, E>::Graph(std::pair<std::initializer_list<V>, std::initializer_list<std::pair<std::pair<size_t, size_t>, E>>>& init) {
        for (auto& e : init.first) {
            insertVertex(e);
        }

        for (auto& p : init.second) {
            insertEdge(p.first.first, p.first.second, p.second);
        }
    }

    template <typename V, typename E>
    inline typename Graph<V, E>::vertex_iterator Graph<V, E>::insertVertex(const V& vertex_data) {
        vertices.push_back(vertex_data);
        neigh_matrix.resize(neigh_matrix.size() + 1);
        neigh_matrix.front().resize(neigh_matrix.front().size() + 1);

        for (auto i = 0; i < neigh_matrix.size(); i++) {
            neigh_matrix.at(i).resize(neigh_matrix.front().size());
        }

        for (auto it = beginVertices(); it != endVertices(); ++it) {
            if (*it == vertex_data) return it;
        }

        return --endVertices();
    }

    template <typename V, typename E>
    inline std::pair<Graph<V, E>::edge_iterator, bool> Graph<V, E>::insertEdge(size_t vertex1_id, size_t vertex2_id, const E& label, bool replace) {
        auto& currentEdge = neigh_matrix.at(vertex1_id).at(vertex2_id);

        if (currentEdge && replace) {
            currentEdge = label;

            return std::make_pair(edge(vertex1_id, vertex2_id), true);
        }
        else if (!currentEdge) {
            currentEdge = label;
            no_of_edges++;

            return std::make_pair(edge(vertex1_id, vertex2_id), true);
        }

        return std::make_pair(endEdges(), false);
    }

    template <typename V, typename E>
    inline typename Graph<V, E>::vertex_iterator Graph<V, E>::removeVertex(size_t vertex_id) {
        if (vertex_id >= vertices.size()) return endVertices();

        auto edges_to_erase = 0;

        vertices.erase(vertices.begin() + vertex_id);

        for (auto i = 0; i < neigh_matrix.at(vertex_id).size(); i++) {
            if (i != vertex_id) {
                if (neigh_matrix.at(vertex_id).at(i)) edges_to_erase++;
            }
        }

        for (auto i = 0; i < neigh_matrix.size(); i++) {
            if (neigh_matrix.at(i).at(vertex_id)) edges_to_erase++;

            neigh_matrix.at(i).erase(neigh_matrix.at(i).begin() + vertex_id);
        }

        neigh_matrix.erase(neigh_matrix.begin() + vertex_id);
        no_of_edges -= edges_to_erase;

        return vertex(vertex_id);
    }

    template <typename V, typename E>
    inline typename Graph<V, E>::edge_iterator Graph<V, E>::removeEdge(size_t vertex1_id, size_t vertex2_id) {
        auto& currentEdge = neigh_matrix.at(vertex1_id).at(vertex2_id);

        if (!currentEdge) return endEdges();

        currentEdge.reset();
        no_of_edges--;

        return edge_iterator(neigh_matrix, vertex1_id, vertex2_id);
    }

    template <typename V, typename E>
    inline void Graph<V, E>::printNeighborhoodMatrix() const {
        if (typeid(E) == typeid(double)) {
            for (auto i = 0; i < neigh_matrix.size(); ++i) {
                for (auto j = 0; j < neigh_matrix.at(i).size(); ++j) {
                    std::cout << std::setw(3) << neigh_matrix.at(i).at(j).value_or(0.0) << "   ";
                }

                std::cout << std::endl;
            }
        }
        else {
            for (auto i = 0; i < neigh_matrix.size(); ++i) {
                for (auto j = 0; j < neigh_matrix.at(i).size(); ++j) {
                    std::cout << neigh_matrix.at(i).at(j).value_or(0) << " ";
                }

                std::cout << std::endl;
            }
        }
    }

    template <typename V, typename E>
    inline void Graph<V, E>::dfs(size_t start, std::function<void(const V&)> visitator_f) {
        std::vector<bool> visited(vertices.size(), false);
        std::stack<size_t> s;
        size_t current = start;
        size_t count = 0;

        std::cout << "DFS algorithm:" << std::endl;

        while (count < nrOfVertices()) {
            if (!visited.at(current)) {
                visitator_f(vertices.at(current));
                visited.at(current) = true;
                count++;

                for (auto i = neigh_matrix.at(current).size() - 1; i != -1; i--) if (neigh_matrix.at(current).at(i)) s.push(i);
            }

            if (s.empty()) break;
            else {
                current = s.top();
                s.pop();
            }
        }

        std::cout << std::endl;
    }

    template <typename V, typename E>
    inline typename Graph<V, E>::vertices_t Graph<V, E>::dfs(size_t start) {
        std::vector<bool> visited(vertices.size(), false);
        std::stack<size_t> s;
        size_t current = start;
        size_t count = 0;
        vertices_t ret;

        while (count < nrOfVertices()) {
            if (!visited.at(current)) {
                ret.push_back(vertices.at(current));
                visited.at(current) = true;
                count++;

                for (auto i = neigh_matrix.at(current).size() - 1; i != -1; i--) if (neigh_matrix.at(current).at(i)) s.push(i);
            }

            if (s.empty()) break;
            else {
                current = s.top();
                s.pop();
            }
        }

        return ret;
    }

    template<typename V, typename E>
    inline void Graph<V, E>::bfs(size_t start, std::function<void(const V&)> visitator_f) {
        std::vector<bool> visited(vertices.size(), false);
        std::queue<size_t> s;
        size_t current = start;
        size_t count = 0;

        std::cout << "BFS algorithm:" << std::endl;

        while (count < nrOfVertices()) {
            if (!visited.at(current)) {
                visitator_f(vertices.at(current));
                visited.at(current) = true;
                count++;

                for (auto i = 0; i < neigh_matrix.at(current).size(); i++) if (neigh_matrix.at(current).at(i)) s.push(i);
            }

            if (s.empty()) break;
            else {
                current = s.front();
                s.pop();
            }
        }

        std::cout << std::endl;
    }

    template<typename V, typename E>
    inline typename Graph<V, E>::vertices_t Graph<V, E>::bfs(size_t start) {
        std::vector<bool> visited(vertices.size(), false);
        std::queue<size_t> s;
        size_t current = start;
        size_t count = 0;
        vertices_t ret;

        std::cout << "BFS algorithm:" << std::endl;

        while (count < nrOfVertices()) {
            if (!visited.at(current)) {
                ret.push_back(vertices.at(current));
                visited.at(current) = true;
                count++;

                for (auto i = 0; i < neigh_matrix.at(current).size(); i++) if (neigh_matrix.at(current).at(i)) s.push(i);
            }

            if (s.empty()) break;
            else {
                current = s.front();
                s.pop();
            }
        }

        return ret;
    }

}

#endif
