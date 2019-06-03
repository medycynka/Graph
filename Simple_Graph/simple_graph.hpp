#ifndef SIMPLE_GRAPH_SIMPLE_GRAPH_HPP
#define SIMPLE_GRAPH_SIMPLE_GRAPH_HPP

#pragma once
#include <iostream>
#include <vector>
#include <optional>
#include <utility>
#include <functional>
#include <algorithm>
#include <stack>
#include <queue>
#include <set>
#include <limits>
#include <unordered_map>
#include <iomanip>
#include <iterator>

#include "maxCliqueAlgorithm.hpp"

const double INF = std::numeric_limits<double>::max();

template <typename V, typename E>
class Graph{
    private:
        class VerticesIterator{
            protected:
                size_t curr_It;
                Graph<V, E> &ref;

                VerticesIterator(Graph<V, E> &graph, std::size_t current_vertex_id) : ref(graph), curr_It(current_vertex_id){};

            public:
                friend class Graph;
                typedef VerticesIterator self_type;
                typedef V value_type;
                typedef V& reference;
                typedef V* pointer;
                typedef std::forward_iterator_tag iterator_category;
                typedef int difference_type;

                VerticesIterator(const VerticesIterator &s)                         : ref(s.ref), curr_It(s.curr_It){};
                VerticesIterator(VerticesIterator &&s) noexcept                     : ref(s.ref), curr_It(s.curr_It){};

                bool                   operator==(const VerticesIterator &vi) const { return (curr_It == vi.curr_It && ref == vi.ref); };
                bool                   operator!=(const VerticesIterator &vi) const { return !(*this == vi); };
                VerticesIterator&      operator++();
                VerticesIterator const operator++(int);
                VerticesIterator&      operator--();
                VerticesIterator const operator--(int);
                VerticesIterator&      operator+=(size_t count);
                VerticesIterator&      operator-=(size_t count);
                V &                    operator* ()     const { return ref.vertices.at(curr_It); };
                V *                    operator->()     const { return *(ref.vertices.at(curr_It)); };
                explicit               operator  bool() const { return curr_It != ref.nrOfVertices(); };
                size_t                 id()                   { return curr_It; };
                const size_t           id()             const { return curr_It; };
        };

        class EdgesIterator{
            protected:
                size_t curr_row;
                size_t curr_col;
                Graph<V, E> &ref;

                explicit EdgesIterator(Graph<V, E> &graph);
                EdgesIterator(Graph<V, E> &graph, std::size_t nm_row, std::size_t nm_col) : ref(graph), curr_row(nm_row), curr_col(nm_col){};

            public:
                friend class Graph;
                typedef EdgesIterator self_type;
                typedef V value_type;
                typedef V& reference;
                typedef V* pointer;
                typedef std::forward_iterator_tag iterator_category;
                typedef int difference_type;

                EdgesIterator(const EdgesIterator &s)                                     : ref(s.ref), curr_row(s.curr_row), curr_col(s.curr_col){};
                EdgesIterator(EdgesIterator &&s) noexcept                                 : ref(s.ref), curr_row(s.curr_row), curr_col(s.curr_col){};

                bool                            operator==(const EdgesIterator &ei) const { return (curr_row == ei.curr_row && curr_col == ei.curr_col && ref == ei.ref); };
                bool                            operator!=(const EdgesIterator &ei) const { return !(*this == ei); };
                EdgesIterator&                  operator++();
                EdgesIterator const             operator++(int);
                E&                              operator* ()     const { return ref.neigh_matrix.at(curr_row).at(curr_col).value(); };
                E*                              operator->()     const { return *(ref.neigh_matrix.at(curr_row).at(curr_col).value()); };
                explicit                        operator  bool() const { return curr_row != ref.nrOfVertices(); };
                std::pair<size_t, size_t>       id()                   { return std::make_pair(curr_row, curr_col); };
                const std::pair<size_t, size_t> id()             const { return std::make_pair(curr_row, curr_col); };
        };

        class dfsIterator{
            protected:
                size_t current;
                Graph<V, E> &ref;
                std::vector<bool> visited;
                std::stack<size_t> s;
                size_t count;

                dfsIterator(Graph<V, E> &graph, std::size_t starting_vertex) : ref(graph), current(starting_vertex), count(1){ visited.resize(ref.nrOfVertices(), false); };

            public:
                friend class Graph;
                typedef dfsIterator self_type;
                typedef V value_type;
                typedef V& reference;
                typedef V* pointer;
                typedef std::forward_iterator_tag iterator_category;
                typedef int difference_type;

                dfsIterator(const dfsIterator &other)                         : ref(other.ref), current(other.current), visited(other.visited), s(other.s), count(other.count){};
                dfsIterator(dfsIterator &&other) noexcept                     : ref(other.ref), current(other.current), visited(other.visited), s(other.s), count(other.count){};

                bool              operator==(const dfsIterator &vi) const { return (current == vi.current && ref == vi.ref); };
                bool              operator!=(const dfsIterator &vi) const { return !(*this == vi); };
                dfsIterator&      operator++();
                dfsIterator const operator++(int);
                V &               operator* ()     const { return ref.vertices.at(current); };
                V *               operator->()     const { return *(ref.vertices.at(current)); };
                explicit          operator  bool() const { return count != ref.nrOfVertices(); };
                size_t            id()                   { return current; };
                const size_t      id()             const { return current; };
        };

        class bfsIterator{
            protected:
                size_t current;
                Graph<V, E> &ref;
                std::vector<bool> visited;
                std::queue<size_t> s;
                size_t count;

                bfsIterator(Graph<V, E> &graph, std::size_t starting_vertex) : ref(graph), current(starting_vertex), count(1){ visited.resize(ref.nrOfVertices(), false); };

            public:
                friend class Graph;
                typedef bfsIterator self_type;
                typedef V value_type;
                typedef V& reference;
                typedef V* pointer;
                typedef std::forward_iterator_tag iterator_category;
                typedef int difference_type;

                bfsIterator(const bfsIterator &other)                         : ref(other.ref), current(other.current), visited(other.visited), s(other.s), count(other.count){};
                bfsIterator(bfsIterator &&other) noexcept                     : ref(other.ref), current(other.current), visited(other.visited), s(other.s), count(other.count){};

                bool              operator==(const bfsIterator &vi) const { return (current == vi.current && ref == vi.ref); };
                bool              operator!=(const bfsIterator &vi) const { return !(*this == vi); };
                bfsIterator&      operator++();
                bfsIterator const operator++(int);
                V &               operator* ()     const { return ref.vertices.at(current); };
                V *               operator->()     const { return *(ref.vertices.at(current)); };
                explicit          operator  bool() const { return count != ref.nrOfVertices(); };
                size_t            id()                   { return current; };
                const size_t      id()             const { return current; };
        };

    public:
        Graph()                            = default;
        Graph(const Graph&)                = default;
        Graph& operator=(const Graph&)     = default;
        Graph(Graph&&)            noexcept = default;
        Graph& operator=(Graph&&) noexcept = default;

        explicit Graph(std::vector<V> &in)                                                         : vertices(in){};
        Graph(std::vector<V> &in_v, std::vector<std::vector<std::optional<E>>> &in_m)              : vertices(in_v), neigh_matrix(in_m){};
        Graph(std::vector<V> &in_v, std::vector<std::vector<std::optional<E>>> &in_m, size_t in_e) : vertices(in_v), neigh_matrix(in_m), no_of_edges(in_e){};

        bool operator==(const Graph &other) { return (no_of_edges == other.no_of_edges && vertices == other.vertices && neigh_matrix == other.neigh_matrix); };
        bool operator!=(const Graph &other) { return !(*this == other); };

        inline VerticesIterator                           insertVertex(const V& vertex_data);
        inline std::pair<EdgesIterator, bool>             insertEdge(size_t vertex1_id, size_t vertex2_id, const E &label = E(), bool replace = true);
        inline bool                                       removeVertex(size_t vertex_id);
        inline bool                                       removeEdge(size_t vertex1_id, size_t vertex2_id);
        inline void                                       printNeighborhoodMatrix() const;
        inline void                                       dfs(size_t start, std::function<void(const V&)> visitator_f);
        inline void                                       bfs(size_t start, std::function<void(const V&)> visitator_f);

        /** Graph's algorithms */
        inline size_t                                     getMinVertex(std::vector<bool> &vis, std::vector<double> &keys) const;
        inline void                                       dijkstraShortestPath(size_t source) const;
        inline void                                       printPathDijkstra(size_t source, std::vector<double> &keys) const;
        inline std::pair<double, std::vector<size_t>>     dijkstra(size_t start, size_t end, std::function<double(const E&)> visitator_f = [](const double &e) -> double { return e; }) const;
        inline std::pair<double, std::vector<size_t>>     AStar(size_t start, size_t end, std::function<double(const E&)> visitator_f, std::function<double(const Graph<V, E>&, const size_t&, const size_t&)> heuristic) const;
        inline void                                       computeFloydWarshall() const;
        inline void                                       printPathFloydWarshall(std::vector<std::vector<std::optional<E>>> &input) const;
        inline bool                                       checkEdge(size_t u, size_t v, std::vector<bool> &vis) const;
        inline void                                       primsMST() const;
        /** Only for directed graphs */
        inline bool                                       checkVerticesDFS_undirected(size_t curr, std::set<size_t> &wSet, std::set<size_t> &gSet, std::set<size_t> &bSet) const;
        inline bool                                       hasCycle_undirected() const;
        inline ssize_t                                    getInDegree(size_t vertex_id) const;
        inline ssize_t                                    getOutDegree(size_t vertex_id) const;
        inline std::vector<V>                             getOutConnectedVerticesTo(size_t vertex_id);
        inline std::vector<V>                             getInConnectedVerticesTo(size_t vertex_id);
        /** Only for undirected graphs */
        inline bool                                       isValidForColors(size_t vertex_id, std::vector<size_t> &col, size_t col_checker) const;
        inline bool                                       tryGraphColoring(size_t nrOfColorsToTry, size_t vertex_id, std::vector<size_t> &col) const;
        inline bool                                       checkColoringResult(size_t nrOfColorsToTry) const;
        inline std::pair<size_t, bool>                    colorUntilDone() const;
        inline void                                       printColors(std::vector<size_t> &col) const;
        inline bool                                       checkVerticesDFS_directed(size_t curr, std::set<size_t> &vis, size_t parent) const;
        inline bool                                       hasCycle_directed() const;
        inline bool                                       isSafeVertex(size_t vertex_id, std::vector<ssize_t> &path, size_t pos) const;
        inline bool                                       tryFindingHamCycle(std::vector<ssize_t> &path, size_t pos) const;
        inline bool                                       hasHamiltonCycle(size_t startingVertex) const;
        inline void                                       printHamiltonCycle(std::vector<ssize_t> &path) const;
        inline void                                       findMaxClique();
        inline ssize_t                                    getDegree(size_t vertex_id)                     const { return getOutDegree(vertex_id); };
        inline std::vector<V>                             getConnectedVerticesTo(size_t vertex_id)              { return getOutConnectedVerticesTo(vertex_id); };

        inline bool                                       edgeExist(size_t vertex1_id, size_t vertex2_id) const { return neigh_matrix[vertex1_id][vertex2_id]; };
        inline size_t                                     nrOfVertices()                                  const { return vertices.size(); };
        inline size_t                                     nrOfEdges()                                     const { return no_of_edges; };
        inline VerticesIterator                           vertex(std::size_t vertex_id)                         { return VerticesIterator(*this, vertex_id); };
        inline EdgesIterator                              edge(std::size_t vertex1_id, std::size_t vertex2_id)  { return EdgesIterator(*this, vertex1_id, vertex2_id); };
        inline std::vector<std::vector<std::optional<E>>> getNeighMatrix() const                                { return neigh_matrix; };
        inline V&                                         vertexData(size_t id)                                 { return vertices.at(id); };
        inline const V&                                   vertexData(size_t id)                           const { return vertices.at(id); };

        inline VerticesIterator begin()                          { return beginVertices(); };
        inline VerticesIterator end()                            { return endVertices(); };
        inline VerticesIterator beginVertices()                  { return VerticesIterator(*this, 0); };
        inline VerticesIterator endVertices()                    { return VerticesIterator(*this, vertices.size()); };
        inline EdgesIterator    beginEdges()                     { return EdgesIterator(*this); };
        inline EdgesIterator    endEdges()                       { return EdgesIterator(*this, neigh_matrix.size(), 0); };
        inline dfsIterator      beginDFS(size_t starting_vertex) { return (starting_vertex < vertices.size() ? dfsIterator(*this, starting_vertex) : endDFS()); };
        inline dfsIterator      endDFS()                         { return dfsIterator(*this, nrOfVertices()); };
        inline bfsIterator      beginBFS(size_t starting_vertex) { return (starting_vertex < vertices.size() ? bfsIterator(*this, starting_vertex) : endBFS()); };
        inline bfsIterator      endBFS()                         { return bfsIterator(*this, nrOfVertices()); };

    private:
        std::vector<V> vertices;
        std::vector<std::vector<std::optional<E>>> neigh_matrix;
        size_t no_of_edges = 0;
};

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator &Graph<V, E>::VerticesIterator::operator++(){
    curr_It++;

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator const Graph<V, E>::VerticesIterator::operator++(int){
    auto pom = *this;
    ++curr_It;

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator &Graph<V, E>::VerticesIterator::operator--(){
    curr_It--;

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator const Graph<V, E>::VerticesIterator::operator--(int){
    auto pom = *this;
    --curr_It;

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator &Graph<V, E>::VerticesIterator::operator+=(size_t count){
    curr_It += count;

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator &Graph<V, E>::VerticesIterator::operator-=(size_t count){
    curr_It -= count;

    return *this;
}

template<typename V, typename E>
Graph<V, E>::EdgesIterator::EdgesIterator(Graph<V, E> &graph) : ref(graph), curr_row(0), curr_col(0){
    bool checker = (ref.neigh_matrix.at(curr_row).at(curr_col) ? true : false);

    while(!checker){
        curr_col++;

        if(curr_col == ref.neigh_matrix.size()){
            curr_row++;
            curr_col = 0;
        }

        if(curr_row == ref.neigh_matrix.size()) checker = true;
        else if(ref.neigh_matrix.at(curr_row).at(curr_col)) checker = true;
    }
}

template<typename V, typename E>
typename Graph<V, E>::EdgesIterator &Graph<V, E>::EdgesIterator::operator++(){
    bool checker = false;

    while(!checker){
        curr_col++;

        if(curr_col == ref.nrOfVertices()){
            curr_row++;
            curr_col = 0;
        }

        if(curr_row == ref.nrOfVertices()) checker = true;
        else if(ref.neigh_matrix.at(curr_row).at(curr_col)) checker = true;
    }

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::EdgesIterator const Graph<V, E>::EdgesIterator::operator++(int){
    auto pom = *this;
    ++(*this);

    return pom;
}

template<typename V, typename E>
typename Graph<V, E>::dfsIterator &Graph<V, E>::dfsIterator::operator++(){
    visited.at(current) = true;

    if(count < ref.nrOfVertices()){
        for(auto i = ref.neigh_matrix.at(current).size() - 1; i != -1; i--) if(ref.neigh_matrix.at(current).at(i)) s.push(i);

        if(!s.empty()){
            while(true){
                if(!s.empty() && !visited.at(s.top())){
                    count++;
                    current = s.top();
                    break;
                }

                if(!s.empty()){
                    s.pop();
                }
                else{
                    count = ref.nrOfVertices();
                    current = ref.nrOfVertices();
                    break;
                }
            }
        }
        else{
            count = ref.nrOfVertices();
            current = ref.nrOfVertices();
        }
    }
    else{
        count = ref.nrOfVertices();
        current = ref.nrOfVertices();
    }

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::dfsIterator const Graph<V, E>::dfsIterator::operator++(int){
    auto pom = *this;
    ++(*this);

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::bfsIterator &Graph<V, E>::bfsIterator::operator++(){
    visited.at(current) = true;

    if(count < ref.nrOfVertices()){
        for(auto i = 0; i < ref.neigh_matrix.at(current).size(); i++) if(ref.neigh_matrix.at(current).at(i)) s.push(i);

        if(!s.empty()){
            while(true){
                if(!s.empty() && !visited.at(s.front())){
                    count++;
                    current = s.front();
                    break;
                }

                if(!s.empty()){
                    s.pop();
                }
                else{
                    count = ref.nrOfVertices();
                    current = ref.nrOfVertices();
                    break;
                }
            }
        }
        else{
            count = ref.nrOfVertices();
            current = ref.nrOfVertices();
        }
    }
    else{
        count = ref.nrOfVertices();
        current = ref.nrOfVertices();
    }

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::bfsIterator const Graph<V, E>::bfsIterator::operator++(int){
    auto pom = *this;
    ++(*this);

    return *this;
}

template <typename V, typename E>
inline typename Graph<V, E>::VerticesIterator Graph<V, E>::insertVertex(const V& vertex_data){
    vertices.push_back(vertex_data);
    neigh_matrix.resize(neigh_matrix.size() + 1);
    neigh_matrix.front().resize(neigh_matrix.front().size() + 1);

    for(auto i = 0; i < neigh_matrix.size(); i++){
        neigh_matrix.at(i).resize(neigh_matrix.front().size());
    }

    for(auto it = beginVertices(); it != endVertices(); ++it){
        if(*it == vertex_data) return it;
    }

    return --endVertices();
}

template <typename V, typename E>
inline std::pair<typename Graph<V, E>::EdgesIterator, bool> Graph<V, E>::insertEdge(size_t vertex1_id, size_t vertex2_id, const E &label, bool replace){
    auto& currentEdge = neigh_matrix.at(vertex1_id).at(vertex2_id);

    if(currentEdge && replace){
        currentEdge = label;

        return std::make_pair(edge(vertex1_id, vertex2_id), true);
    }
    else if(!currentEdge){
        currentEdge = label;
        no_of_edges++;

        return std::make_pair(edge(vertex1_id, vertex2_id), true);
    }

    return std::make_pair(endEdges(), false);
}

template <typename V, typename E>
inline bool Graph<V, E>::removeVertex(size_t vertex_id){
    if(vertex_id >= vertices.size()) return false;

    size_t edges_to_erase = 0;

    vertices.erase(vertices.begin() + vertex_id);

    for(auto i = 0; i < neigh_matrix.at(vertex_id).size(); i++){
        if(i != vertex_id){
            if(neigh_matrix.at(vertex_id).at(i)) edges_to_erase++;
        }
    }

    for(auto i = 0; i < neigh_matrix.size(); i++){
        if(neigh_matrix.at(i).at(vertex_id)) edges_to_erase++;

        neigh_matrix.at(i).erase(neigh_matrix.at(i).begin() + vertex_id);
    }

    neigh_matrix.erase(neigh_matrix.begin() + vertex_id);
    no_of_edges -= edges_to_erase;

    return true;
}

template <typename V, typename E>
inline bool Graph<V, E>::removeEdge(size_t vertex1_id, size_t vertex2_id){
    auto& currentEdge = neigh_matrix.at(vertex1_id).at(vertex2_id);

    if(!currentEdge) return false;

    currentEdge.reset();
    no_of_edges--;

    return true;
}

template <typename V, typename E>
inline void Graph<V, E>::printNeighborhoodMatrix() const{
    if(typeid(E) == typeid(double)){
        for(auto i = 0; i < neigh_matrix.size(); ++i){
            for(auto j = 0; j < neigh_matrix.at(i).size(); ++j){
                std::cout << std::setw(3) << neigh_matrix.at(i).at(j).value_or(0.0) << "   ";
            }

            std::cout << std::endl;
        }
    }
    else{
        for(auto i = 0; i < neigh_matrix.size(); ++i){
            for(auto j = 0; j < neigh_matrix.at(i).size(); ++j){
                std::cout << neigh_matrix.at(i).at(j).value_or(0) << " ";
            }

            std::cout << std::endl;
        }
    }
}

template <typename V, typename E>
inline void Graph<V, E>::dfs(size_t start, std::function<void(const V&)> visitator_f){
    std::vector<bool> visited(vertices.size(), false);
    std::stack<size_t> s;
    size_t current = start;
    size_t count = 0;

    std::cout << "DFS algorithm:" << std::endl;

    while(count < nrOfVertices()){
        if(!visited.at(current)){
            visitator_f(vertices.at(current));
            visited.at(current) = true;
            count++;

            for(auto i = neigh_matrix.at(current).size() - 1; i != -1; i--) if(neigh_matrix.at(current).at(i)) s.push(i);
        }

        if(s.empty()) break;
        else{
            current = s.top();
            s.pop();
        }
    }

    std::cout << std::endl;
}

template<typename V, typename E>
inline void Graph<V, E>::bfs(size_t start, std::function<void(const V &)> visitator_f){
    std::vector<bool> visited(vertices.size(), false);
    std::queue<size_t> s;
    size_t current = start;
    size_t count = 0;

    std::cout << "BFS algorithm:" << std::endl;

    while(count < nrOfVertices()){
        if(!visited.at(current)){
            visitator_f(vertices.at(current));
            visited.at(current) = true;
            count++;

            for(auto i = 0; i < neigh_matrix.at(current).size(); i++) if(neigh_matrix.at(current).at(i)) s.push(i);
        }

        if(s.empty()) break;
        else{
            current = s.front();
            s.pop();
        }
    }

    std::cout << std::endl;
}

template<typename V, typename E>
inline size_t Graph<V, E>::getMinVertex(std::vector<bool> &vis, std::vector<double> &keys) const{
    double minKey = INF;
    int_fast32_t vertex = -1;

    for(auto i = 0; i < nrOfVertices(); i++){
        if(!vis.at(i) && minKey > keys.at(i)){
            minKey = keys.at(i);
            vertex = i;
        }
    }

    return vertex;
}

template<typename V, typename E>
inline void Graph<V, E>::dijkstraShortestPath(size_t source) const{
    auto size_ = nrOfVertices();
    std::vector<bool> visited(size_, false);
    std::vector<double> distance(size_, INF);
    size_t temp1;
    double temp2;

    distance.at(source) = 0;

    for(auto i = 0; i < size_-1; i++){
        temp1 = getMinVertex(visited, distance);
        visited.at(temp1) = true;

        for(auto j = 0; j < size_; j++){
            if(neigh_matrix.at(temp1).at(j).value_or(INF) > 0){
                if(!visited.at(j) && neigh_matrix.at(temp1).at(j).value_or(INF) != INF){
                    temp2 = neigh_matrix.at(temp1).at(j).value() + distance.at(temp1);

                    if(temp2 < distance.at(j)) distance.at(j) = temp2;
                }
            }
        }
    }

    printPathDijkstra(source, distance);
}

template<typename V, typename E>
inline void Graph<V, E>::printPathDijkstra(size_t source, std::vector<double> &keys) const{
    for(auto i = 0; i < nrOfVertices(); i++) {
        if(keys.at(i) != INF) std::cout << "From vertex " << vertices.at(source) << " to " << vertices.at(i) << " distance = " << keys.at(i) << std::endl;
        else std::cout << "From vertex " << vertices.at(source) << " couldn't reach vertex " << vertices.at(i) << " (no connection)" << std::endl;
    }
}

template<typename V, typename E>
inline std::pair<double, std::vector<size_t>> Graph<V, E>::dijkstra(size_t start, size_t end, std::function<double(const E&)> visitator_f) const{
    auto size_ = nrOfVertices();
    std::unordered_map<size_t, double> distances;   // vertex - distance
    std::unordered_map<size_t, size_t> previous;
    std::vector<size_t> nodes;
    std::vector<size_t> path;
    auto comparator = [&](size_t lhs, size_t rhs){ return (distances[lhs] > distances[rhs]); };

    for(auto i = 0; i < size_; i++){
        if(i == start) distances[i] = 0;
        else distances[i] = INF;

        nodes.push_back(i);
        std::push_heap(nodes.begin(), nodes.end(), comparator);
    }

    while(!nodes.empty()){
        std::pop_heap(nodes.begin(), nodes.end(), comparator);
        auto smallest = nodes.back();
        nodes.pop_back();

        if(smallest == end){
            while(previous.find(smallest) != previous.end()){
                path.push_back(smallest);
                smallest = previous[smallest];
            }

            break;
        }

        if(distances[smallest] == INF) break;

        for(auto i = 0; i < size_; i++){
            auto alt = distances[smallest] + visitator_f(neigh_matrix.at(smallest).at(i).value_or(INF));

            if(alt < distances[i]){
                distances[i] = alt;
                previous[i] = smallest;
                std::make_heap(nodes.begin(), nodes.end(), comparator);
            }
        }
    }

    if(distances[end] != INF) {
        path.push_back(start);
        std::reverse(path.begin(), path.end());

        return std::make_pair(distances[end], path);
    }
    else return std::make_pair(-1, path);
}

template<typename V, typename E>
inline void Graph<V, E>::computeFloydWarshall() const{
    std::vector<std::vector<std::optional<E>>> temp = neigh_matrix;
    auto size_ = nrOfVertices();

    for(auto k = 0; k < size_; k++){
        temp.at(k).at(k) = 0;

        for(auto i = 0; i < size_; i++){
            for(auto j = 0; j < size_; j++){
                if(temp.at(i).at(j).value_or(INF) > temp.at(i).at(k).value_or(INF) + temp.at(k).at(j).value_or(INF)){
                    temp.at(i).at(j) = temp.at(i).at(k).value_or(INF) + temp.at(k).at(j).value_or(INF);
                }
            }
        }
    }

    printPathFloydWarshall(temp);
}

template<typename V, typename E>
inline void Graph<V, E>::printPathFloydWarshall(std::vector<std::vector<std::optional<E>>> &input) const{
    std::cout << "Shortest paths:" << std::endl;
    auto size_ = nrOfVertices();

    for(auto i = 0; i < size_; i++){
        for(auto j = 0; j < size_; j++){
            std::cout << "From " << vertices.at(i) << " to " << vertices.at(j) << " = ";

            if(input.at(i).at(j).value_or(INF) == INF) std::cout << "NO PATH";
            else std::cout << input.at(i).at(j).value_or(INF);

            std::cout << std::endl;
        }
    }
}

template<typename V, typename E>
inline bool Graph<V, E>::checkEdge(size_t u, size_t v, std::vector<bool> &vis) const{
    if( u == v || (!vis.at(u) && !vis.at(v)) ) return false;
    else return !(vis.at(u) && vis.at(v));
}

template<typename V, typename E>
inline void Graph<V, E>::primsMST() const{
    std::vector<bool> visited(nrOfVertices(), false);
    size_t counter = 0;
    double min_cost = 0;
    auto size_ = nrOfVertices();

    visited.at(0) = true;

    while(counter < size_-1){
        auto min = INF;
        auto a = -1;
        auto b = -1;

        for(auto i = 0; i < size_; i++){
            for(auto j = 0; j < size_; j++){
                if(neigh_matrix.at(i).at(j).value_or(INF) < min){
                    if(checkEdge(i, j, visited)){
                        min = neigh_matrix.at(i).at(j).value_or(INF);
                        a = i;
                        b = j;
                    }
                }
            }
        }

        if(a != -1 && b != -1){
            std::cout << "Edge: " << counter++ << " [" << a << ", " << b << "] cost: " << min << std::endl;
            min_cost += min;
            visited.at(a) = true;
            visited.at(b) = true;
        }
    }

    std::cout << "Minimum cost of MST = " << min_cost << std::endl;
}

template<typename V, typename E>
inline bool Graph<V, E>::isValidForColors(size_t vertex_id, std::vector<size_t> &col, size_t col_checker) const{
    for(auto i = 0; i < nrOfVertices(); i++) if(neigh_matrix.at(vertex_id).at(i) && col_checker == col.at(i)) return false;

    return true;
}

template<typename V, typename E>
inline bool Graph<V, E>::tryGraphColoring(size_t nrOfColorsToTry, size_t vertex_id, std::vector<size_t> &col) const{
    auto size_ = nrOfVertices();

    if(vertex_id == size_) return true;

    for(auto i = 1; i <= size_; i++){
        if(isValidForColors(vertex_id, col, i)){
            col.at(vertex_id) = i;

            if(tryGraphColoring(nrOfColorsToTry, vertex_id+1, col)) return true;

            col.at(vertex_id) = 0;
        }
    }

    return false;
}

template<typename V, typename E>
inline bool Graph<V, E>::checkColoringResult(size_t nrOfColorsToTry) const{
    std::vector<size_t> colors(nrOfVertices(), 0);

    if(!tryGraphColoring(nrOfColorsToTry, 0, colors)){
        std::cout << "Couldn't find solution for " << nrOfColorsToTry << " colors" << std::endl;

        return false;
    }
    else{
        printColors(colors);

        return true;
    }
}

template<typename V, typename E>
inline std::pair<size_t, bool>  Graph<V, E>::colorUntilDone() const{
    auto size_ = nrOfVertices();
    std::vector<size_t> colors(size_, 0);
    auto chromaticNumber = 1;

    while(!tryGraphColoring(chromaticNumber, 0, colors)) chromaticNumber++;

    printColors(colors);

    return std::make_pair(chromaticNumber, (chromaticNumber < size_));
}

template<typename V, typename E>
inline void Graph<V, E>::printColors(std::vector<size_t> &col) const{
    std::cout << "Solution for coloring vertices:" << std::endl;

    for(auto i = 0; i < nrOfVertices(); i++) std::cout << vertices.at(i) << " color: " << col.at(i) << std::endl;
}

template<typename V, typename E>
inline bool Graph<V, E>::checkVerticesDFS_undirected(size_t curr, std::set<size_t> &wSet, std::set<size_t> &gSet, std::set<size_t> &bSet) const{
    wSet.erase(wSet.find(curr));
    gSet.insert(curr);

    for(auto i = 0; i < nrOfVertices(); i++){
        if(neigh_matrix.at(curr).at(i)){
            if(bSet.find(i) != bSet.end()) continue;

            if(gSet.find(i) != gSet.end()) return true;

            if(checkVerticesDFS_undirected(i, wSet, gSet, bSet)) return true;
        }
    }

    gSet.erase(gSet.find(curr));
    bSet.insert(curr);

    return false;
}

template<typename V, typename E>
inline bool Graph<V, E>::hasCycle_undirected() const{
    std::set<size_t> whiteSet, greySey, blackSet;
    auto size_ = nrOfVertices();

    for(auto i = 0; i < size_; i++) whiteSet.insert(i);

    while(!whiteSet.empty()){
        for(auto i = 0; i < size_; i++){
            if(whiteSet.find(i) != whiteSet.end()){
                if(checkVerticesDFS_undirected(i, whiteSet, greySey, blackSet)) return true;
            }
        }
    }

    return false;
}

template<typename V, typename E>
inline bool Graph<V, E>::checkVerticesDFS_directed(size_t curr, std::set<size_t> &vis, size_t parent) const{
    vis.insert(curr);

    for(auto i = 0; i < nrOfVertices(); i++){
        if(neigh_matrix.at(curr).at(i)){
            if(i == parent) continue;

            if(vis.find(i) != vis.end()) return true;

            if(checkVerticesDFS_directed(i, vis, curr)) return true;
        }
    }

    return false;
}

template<typename V, typename E>
inline bool Graph<V, E>::hasCycle_directed() const{
    std::set<size_t> checker;

    for(auto i = 0; i < nrOfVertices(); i++){
        if(checker.find(i) != checker.end()) continue;

        if(checkVerticesDFS_directed(i, checker, -1)) return true;
    }

    return false;
}

template<typename V, typename E>
inline bool Graph<V, E>::isSafeVertex(size_t vertex_id, std::vector<ssize_t> &path, size_t pos) const{
    if(!neigh_matrix.at(path.at(pos-1)).at(vertex_id)) return false;

    for(auto i = 0; i < pos; i++) if(path.at(i) == vertex_id) return false;

    return true;
}

template<typename V, typename E>
inline bool Graph<V, E>::tryFindingHamCycle(std::vector<ssize_t> &path, size_t pos) const{
    if(pos == nrOfVertices()) return (neigh_matrix.at(path.at(pos-1)).at(path.at(0)) ? true : false);

    for(auto i = 1; i < nrOfVertices(); i++){
        if(isSafeVertex(i, path, pos)){
            path.at(pos) = i;

            if(tryFindingHamCycle(path, pos+1)) return true;

            path.at(pos) = -1;
        }
    }

    return false;
}

template<typename V, typename E>
inline bool Graph<V, E>::hasHamiltonCycle(size_t startingVertex) const{
    std::vector<ssize_t> path(nrOfVertices(), -1);
    path.at(0) = 0;

    if(!tryFindingHamCycle(path, startingVertex)){
        std::cout << "This graph doesn't have Hamilton cycles" << std::endl;

        return false;
    }
    else{
        printHamiltonCycle(path);

        return true;
    }
}

template<typename V, typename E>
inline void Graph<V, E>::printHamiltonCycle(std::vector<ssize_t> &path) const{
    std::cout << "Hamilton cycle:" << std::endl;

    for(auto i = 0; i < nrOfVertices(); i++) std::cout << path.at(i) << ", ";
    std::cout << path.at(0) << std::endl;
}

template<typename V, typename E>
inline ssize_t Graph<V, E>::getInDegree(size_t vertex_id) const{
    if(vertex_id >= nrOfVertices() || vertex_id < 0) return -1;
    else{
        ssize_t degree = 0;

        for(auto i = 0; i < nrOfVertices(); i++) if(neigh_matrix.at(i).at(vertex_id)) degree++;

        return degree;
    }
}

template<typename V, typename E>
inline ssize_t Graph<V, E>::getOutDegree(size_t vertex_id) const{
    if(vertex_id >= nrOfVertices() || vertex_id < 0) return -1;
    else{
        ssize_t degree = 0;

        for(auto i = 0; i < nrOfVertices(); i++) if(neigh_matrix.at(vertex_id).at(i)) degree++;

        return degree;
    }
}

template<typename V, typename E>
inline std::vector<V> Graph<V, E>::getOutConnectedVerticesTo(size_t vertex_id){
    std::vector<V> result;

    for(auto i = 0; i < nrOfVertices(); i++){
        if(neigh_matrix.at(vertex_id).at(i)) result.push_back(vertices.at(i));
    }

    return result;
}

template<typename V, typename E>
inline std::vector<V> Graph<V, E>::getInConnectedVerticesTo(size_t vertex_id){
    std::vector<V> result;

    for(auto i = 0; i < nrOfVertices(); i++){
        if(neigh_matrix.at(i).at(vertex_id)) result.push_back(vertices.at(i));
    }

    return result;
}

template<typename V, typename E>
inline void Graph<V, E>::findMaxClique(){
    FindMaxClique<E> mc(neigh_matrix, nrOfVertices(), 0.025);
    std::vector<ssize_t> maxClique;
    mc.maxCliqueAlgorithm(maxClique);

    std::cout << "Maximum clique: ";
    for(auto i : maxClique) std::cout << i << " ";
    std::cout << std::endl;
    std::cout << "Size = " << maxClique.size() << std::endl;
    std::cout << "Number of steps = " << mc.steps() << std::endl;
}

template<typename V, typename E>
inline std::pair<double, std::vector<size_t>> Graph<V, E>::AStar(size_t start, size_t end, std::function<double(const E&)> visitator_f, std::function<double(const Graph<V, E>&, const size_t&, const size_t&)> heuristic) const{
    auto size_ = nrOfVertices();
    std::unordered_map<size_t, double> distances;   // vertex - distance
    std::unordered_map<size_t, size_t> previous;
    std::vector<size_t> nodes;
    std::vector<size_t> path;
    auto comparator = [&](size_t lhs, size_t rhs){ return (distances[lhs] > distances[rhs]); };
    //auto calculateG = [&](size_t current_, size_t end_)->double{ return std::sqrt(std::abs(neigh_matrix.at(current_).at(end_).value_or(INF) - neigh_matrix.at(end_).at(current_).value_or(INF))); };
    auto tilesVisited = 1;

    for(auto i = 0; i < size_; i++){
        if(i == start) distances[i] = 0;
        else distances[i] = INF;

        nodes.push_back(i);
        std::push_heap(nodes.begin(), nodes.end(), comparator);
    }

    while(!nodes.empty()){
        std::pop_heap(nodes.begin(), nodes.end(), comparator);
        auto smallest = nodes.back();
        nodes.pop_back();

        if(smallest == end){
            while(previous.find(smallest) != previous.end()){
                path.push_back(smallest);
                smallest = previous[smallest];
            }

            break;
        }

        if(distances[smallest] == INF) break;

        for(auto i = 0; i < size_; i++){
            auto heu = heuristic(*this, smallest, i);
            auto alt = distances[smallest] + visitator_f(neigh_matrix.at(smallest).at(i).value_or(INF));

            if(alt + heu < distances[i]){
                distances[i] = alt;
                previous[i] = smallest;
                std::make_heap(nodes.begin(), nodes.end(), comparator);
            }
        }

        tilesVisited++;
    }

    std::cout << "Tiles visited during algorithm: " << tilesVisited << std::endl;
    if(distances[end] != INF) {
        path.push_back(start);
        std::reverse(path.begin(), path.end());

        return std::make_pair(distances[end], path);
    }
    else return std::make_pair(-1, path);
}

#endif //SIMPLE_GRAPH_SIMPLE_GRAPH_HPP