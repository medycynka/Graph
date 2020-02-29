#ifndef GRAPH_GRAPH_HPP
#define GRAPH_GRAPH_HPP

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
#include <exception>


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
        friend class Graph<V, E>;
        typedef VerticesIterator          self_type;
        typedef V                         value_type;
        typedef V&                        reference;
        typedef V*                        pointer;
        typedef std::forward_iterator_tag iterator_category;
        typedef int                       difference_type;

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
        [[nodiscard]] size_t   id()             const { return curr_It; };
    };

    class EdgesIterator{
    protected:
        size_t curr_row;
        size_t curr_col;
        Graph<V, E> &ref;

        explicit EdgesIterator(Graph<V, E> &graph);
        EdgesIterator(Graph<V, E> &graph, std::size_t nm_row, std::size_t nm_col);

    public:
        friend class Graph;
        typedef EdgesIterator             self_type;
        typedef E                         value_type;
        typedef E&                        reference;
        typedef E*                        pointer;
        typedef std::forward_iterator_tag iterator_category;
        typedef int                       difference_type;

        EdgesIterator(const EdgesIterator &s)                                     : ref(s.ref), curr_row(s.curr_row), curr_col(s.curr_col){};
        EdgesIterator(EdgesIterator &&s) noexcept                                 : ref(s.ref), curr_row(s.curr_row), curr_col(s.curr_col){};

        bool                            operator==(const EdgesIterator &ei) const { return (curr_row == ei.curr_row && curr_col == ei.curr_col && ref == ei.ref); };
        bool                            operator!=(const EdgesIterator &ei) const { return !(*this == ei); };
        EdgesIterator&                  operator++();
        EdgesIterator const             operator++(int);
        E&                              operator* ()     const { return ref.neigh_matrix.at(curr_row).at(curr_col).value(); };
        E*                              operator->()     const { return *(ref.neigh_matrix.at(curr_row).at(curr_col).value()); };
        explicit                        operator  bool() const { return curr_row != ref.nrOfVertices(); };
        std::pair<size_t, size_t>               id()           { return std::make_pair(curr_row, curr_col); };
        [[nodiscard]] std::pair<size_t, size_t> id()     const { return std::make_pair(curr_row, curr_col); };
        [[nodiscard]] size_t                    v1id()   const { return curr_row; };
        [[nodiscard]] size_t                    v2id()   const { return curr_col; };
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
        typedef dfsIterator               self_type;
        typedef V                         value_type;
        typedef V&                        reference;
        typedef V*                        pointer;
        typedef std::forward_iterator_tag iterator_category;
        typedef int                       difference_type;

        dfsIterator(const dfsIterator &other)                     : ref(other.ref), current(other.current), visited(other.visited), s(other.s), count(other.count){};
        dfsIterator(dfsIterator &&other) noexcept                 : ref(other.ref), current(other.current), visited(other.visited), s(other.s), count(other.count){};

        bool              operator==(const dfsIterator &vi) const { return (current == vi.current && ref == vi.ref); };
        bool              operator!=(const dfsIterator &vi) const { return !(*this == vi); };
        dfsIterator&      operator++();
        dfsIterator const operator++(int);
        V &               operator* ()     const { return ref.vertices.at(current); };
        V *               operator->()     const { return *(ref.vertices.at(current)); };
        explicit          operator  bool() const { return count != ref.nrOfVertices(); };
        size_t               id()                { return current; };
        [[nodiscard]] size_t id()          const { return current; };
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
        typedef bfsIterator               self_type;
        typedef V                         value_type;
        typedef V&                        reference;
        typedef V*                        pointer;
        typedef std::forward_iterator_tag iterator_category;
        typedef int                       difference_type;

        bfsIterator(const bfsIterator &other)                     : ref(other.ref), current(other.current), visited(other.visited), s(other.s), count(other.count){};
        bfsIterator(bfsIterator &&other) noexcept                 : ref(other.ref), current(other.current), visited(other.visited), s(other.s), count(other.count){};

        bool              operator==(const bfsIterator &vi) const { return (current == vi.current && ref == vi.ref); };
        bool              operator!=(const bfsIterator &vi) const { return !(*this == vi); };
        bfsIterator&      operator++();
        bfsIterator const operator++(int);
        V &               operator* ()     const { return ref.vertices.at(current); };
        V *               operator->()     const { return *(ref.vertices.at(current)); };
        explicit          operator  bool() const { return count != ref.nrOfVertices(); };
        size_t               id()                { return current; };
        [[nodiscard]] size_t id()          const { return current; };
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
    Graph<V, E>&operator+=(const V &new_vertex){ insertVertex(new_vertex); return *this; };
    Graph<V, E>&operator-=(const V &new_vertex){ removeVertex(new_vertex); return *this; };
    Graph<V, E>&operator+=(const std::pair<std::pair<V, V>, E> &e){ insertEdge(e.first.first, e.first.second, e.second); return *this; };
    Graph<V, E>&operator-=(const std::pair<V, V> &p){ removeEdge(p.first, p.second); return *this; };

    inline VerticesIterator                           insertVertex(const V& vertex_data);
    inline std::pair<EdgesIterator, bool>             insertEdge(size_t vertex1_id, size_t vertex2_id, const E &label = E(), bool replace = true);
    inline VerticesIterator                           removeVertex(size_t vertex_id);
    inline EdgesIterator                              removeEdge(size_t vertex1_id, size_t vertex2_id);
    inline void                                       printNeighborhoodMatrix() const;
    inline void                                       dfs(size_t start, std::function<void(const V&)> visitator_f);
    inline std::vector<V>                             dfs(size_t start);
    inline void                                       bfs(size_t start, std::function<void(const V&)> visitator_f);
    inline std::vector<V>                             bfs(size_t start);
    [[nodiscard]] inline bool                         edgeExist(size_t vertex1_id, size_t vertex2_id)           const { return neigh_matrix[vertex1_id][vertex2_id]; };
    [[nodiscard]] inline size_t                       nrOfVertices()                                            const { return vertices.size(); };
    [[nodiscard]] inline size_t                       nrOfEdges()                                               const { return no_of_edges; };
    inline VerticesIterator                           vertex(std::size_t vertex_id)                                   { return (vertex_id < nrOfVertices() ? VerticesIterator(*this, vertex_id) : endVertices()); };
    inline EdgesIterator                              edge(std::size_t vertex1_id, std::size_t vertex2_id)            { return (neigh_matrix.at(vertex1_id).at(vertex2_id) ? EdgesIterator(*this, vertex1_id, vertex2_id) : endEdges()); };
    inline std::vector<std::vector<std::optional<E>>> getNeighMatrix() const                                          { return neigh_matrix; };
    inline std::vector<V>                             getVertices() const                                             { return vertices; };
    [[nodiscard]] inline std::vector<size_t>          getNeighbours(size_t id)                                  const { std::vector<size_t> ret; for(auto i = 0; i < nrOfVertices(); i++){ if(neigh_matrix.at(id).at(i)){ ret.push_back(i); } } return ret; }
    inline V&                                         vertexData(size_t id)                                           { if(id < 0 || id > nrOfVertices()){ throw std::out_of_range("Wrong id"); }else{ return vertices.at(id); } };
    inline const V&                                   vertexData(size_t id)                                     const { if(id < 0 || id > nrOfVertices()){ throw std::out_of_range("Wrong id"); }else{ return vertices.at(id); } };
    inline const E&                                   edgeLabel(std::size_t vertex1_id, std::size_t vertex2_id) const { if(neigh_matrix.at(vertex1_id).at(vertex2_id)){ return neigh_matrix.at(vertex1_id).at(vertex2_id).value(); }else{ throw std::bad_optional_access(); } };
    inline E&                                         edgeLabel(std::size_t vertex1_id, std::size_t vertex2_id)       { if(neigh_matrix.at(vertex1_id).at(vertex2_id)){ return neigh_matrix.at(vertex1_id).at(vertex2_id).value(); }else{ throw std::bad_optional_access(); } };

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

        checker = (curr_row == ref.neigh_matrix.size() ? true : (ref.neigh_matrix.at(curr_row).at(curr_col) ? true : false));
    }
}

template<typename V, typename E>
Graph<V, E>::EdgesIterator::EdgesIterator(Graph<V, E> &graph, std::size_t nm_row, std::size_t nm_col) : ref(graph), curr_row(nm_row), curr_col(nm_col){
    if(curr_row != graph.nrOfVertices() && curr_col != graph.nrOfVertices()) {
        bool checker = (ref.neigh_matrix.at(curr_row).at(curr_col) ? true : false);

        while(!checker){
            curr_col++;

            if(curr_col == ref.neigh_matrix.size()){
                curr_row++;
                curr_col = 0;
            }

            checker = (curr_row == ref.neigh_matrix.size() ? true : (ref.neigh_matrix.at(curr_row).at(curr_col) ? true : false));
        }
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

        checker = (curr_row == ref.neigh_matrix.size() ? true : (ref.neigh_matrix.at(curr_row).at(curr_col) ? true : false));
//        if(curr_row == ref.neigh_matrix.size()) checker = true;
//        else if(ref.neigh_matrix.at(curr_row).at(curr_col)) checker = true;
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
inline typename Graph<V, E>::VerticesIterator Graph<V, E>::removeVertex(size_t vertex_id){
    if(vertex_id >= vertices.size()) return endVertices();

    auto edges_to_erase = 0;

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

    return vertex(vertex_id);
}

template <typename V, typename E>
inline typename Graph<V, E>::EdgesIterator Graph<V, E>::removeEdge(size_t vertex1_id, size_t vertex2_id){
    auto& currentEdge = neigh_matrix.at(vertex1_id).at(vertex2_id);

    if(!currentEdge) return endEdges();

    currentEdge.reset();
    no_of_edges--;

    return EdgesIterator(*this, vertex1_id, vertex2_id);
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

template <typename V, typename E>
inline std::vector<V> Graph<V, E>::dfs(size_t start){
    std::vector<bool> visited(vertices.size(), false);
    std::stack<size_t> s;
    size_t current = start;
    size_t count = 0;
    std::vector<V> ret;

    while(count < nrOfVertices()){
        if(!visited.at(current)){
            ret.push_back(vertices.at(current));
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

    return ret;
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
inline std::vector<V> Graph<V, E>::bfs(size_t start){
    std::vector<bool> visited(vertices.size(), false);
    std::queue<size_t> s;
    size_t current = start;
    size_t count = 0;
    std::vector<V> ret;

    std::cout << "BFS algorithm:" << std::endl;

    while(count < nrOfVertices()){
        if(!visited.at(current)){
            ret.push_back(vertices.at(current));
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

    return ret;
}

#endif //GRAPH_GRAPH_HPP
