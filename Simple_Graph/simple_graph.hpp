#ifndef SIMPLE_GRAPH_SIMPLE_GRAPH_HPP
#define SIMPLE_GRAPH_SIMPLE_GRAPH_HPP

#pragma once
#include <iostream>
#include <vector>
#include <optional>
#include <utility>

template <typename V, typename E>
class Graph{
private:
    class VerticesIterator{
        protected:
            size_t curr_It;
            Graph<V, E> &reference;

            VerticesIterator(Graph<V, E> &graph, std::size_t current_vertex_id) : reference(graph), curr_It(current_vertex_id){};

        public:
            friend class Graph;

			VerticesIterator(const VerticesIterator &s)                         : reference(s.reference), curr_It(s.curr_It){};
            VerticesIterator(VerticesIterator &&s) noexcept                     : reference(s.reference), curr_It(s.curr_It){};

            bool                   operator==(const VerticesIterator &vi) const { return (curr_It == vi.curr_It && reference == vi.reference); };
            bool                   operator!=(const VerticesIterator &vi) const { return !(*this == vi); };
            VerticesIterator&      operator++();
            VerticesIterator const operator++(int);
            VerticesIterator&      operator--();
            VerticesIterator const operator--(int);
            VerticesIterator&      operator+=(size_t count);
            VerticesIterator&      operator-=(size_t count);
            V &                    operator* () const { return reference.vertices.at(curr_It); };
            V *                    operator->() const { return *(reference.vertices.at(curr_It)); };
    };

    class EdgesIterator{
        protected:
            size_t curr_row;
            size_t curr_col;
            Graph<V, E> &reference;

            EdgesIterator(Graph<V, E> &graph) : reference(graph), curr_row(0), curr_col(0){bool checker = (reference.neigh_matrix.at(curr_row).at(curr_col) ? true : false);

	while(!checker){
        curr_col++;

        if(curr_col == reference.neigh_matrix.size()){
            curr_row++;
            curr_col = 0;
        }
        if(curr_row == reference.neigh_matrix.size()) checker = true;
        else if(reference.neigh_matrix.at(curr_row).at(curr_col)) checker = true;}};

            EdgesIterator(Graph<V, E> &graph, std::size_t nm_row, std::size_t nm_col) : reference(graph), curr_row(nm_row), curr_col(nm_col){};

        public:
            friend class Graph;

			EdgesIterator(const EdgesIterator &s)                                     : reference(s.reference), curr_row(s.curr_row), curr_col(s.curr_col){};
			EdgesIterator(EdgesIterator &&s) noexcept                                 : reference(s.reference), curr_row(s.curr_row), curr_col(s.curr_col){};

			std::pair<size_t, size_t> get_row_col(){ return std::make_pair(curr_row, curr_col); };

            bool                operator==(const EdgesIterator &ei) const { return (curr_row == ei.curr_row && curr_col == ei.curr_col && reference == ei.reference); };
            bool                operator!=(const EdgesIterator &ei) const { return !(*this == ei); };
            EdgesIterator&      operator++();
            EdgesIterator const operator++(int);
            E&                  operator* () const { return reference.neigh_matrix.at(curr_row).at(curr_col).value(); };
            E*                  operator->() const { return *(reference.neigh_matrix.at(curr_row).at(curr_col).value()); };
    };

public:
    friend class VerticesIterator;
    friend class EdgesIterator;

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

    inline VerticesIterator               insertVertex (const V& vertex_data);
    inline std::pair<EdgesIterator, bool> insertEdge (size_t vertex1_id, size_t vertex2_id, const E &label = E(), bool replace = true);
    inline bool                           removeVertex(size_t vertex_id);
    inline bool                           removeEdge(size_t vertex1_id, size_t vertex2_id);
    inline void                           printNeighborhoodMatrix() const;
	inline void                           dfs();
    inline bool                           edgeExist(size_t vertex1_id, size_t vertex2_id) const { return neigh_matrix[vertex1_id][vertex2_id]; };
    inline size_t                         nrOfVertices()                                  const { return vertices.size(); };
    inline size_t                         nrOfEdges()                                     const { return no_of_edges; };
    inline VerticesIterator               vertex(std::size_t vertex_id)                         { return VerticesIterator(*this, vertex_id); };
    inline EdgesIterator                  edge(std::size_t vertex1_id, std::size_t vertex2_id)  { return EdgesIterator(*this, vertex1_id, vertex2_id); };

    inline VerticesIterator begin()         { return beginVertices(); };
    inline VerticesIterator end()           { return endVertices(); };
    inline VerticesIterator beginVertices() { return VerticesIterator(*this, 0); };
    inline VerticesIterator endVertices()   { return VerticesIterator(*this, vertices.size()); };
    inline EdgesIterator    beginEdges()    { return EdgesIterator(*this); };
    inline EdgesIterator    endEdges()      { return EdgesIterator(*this, neigh_matrix.size(), 0); };

private:
    std::vector<V> vertices;
    std::vector<std::vector<std::optional<E>>> neigh_matrix;
    size_t no_of_edges = 0;
};

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator &Graph<V, E>::VerticesIterator::operator++() {
    curr_It++;

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator const Graph<V, E>::VerticesIterator::operator++(int) {
    auto pom = *this;
    ++curr_It;

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator &Graph<V, E>::VerticesIterator::operator--() {
    curr_It--;

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator const Graph<V, E>::VerticesIterator::operator--(int) {
    auto pom = *this;
    --curr_It;

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator &Graph<V, E>::VerticesIterator::operator+=(size_t count) {
    curr_It += count;

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::VerticesIterator &Graph<V, E>::VerticesIterator::operator-=(size_t count) {
    curr_It -= count;

    return *this;
}

template<typename V, typename E>
typename Graph<V, E>::EdgesIterator &Graph<V, E>::EdgesIterator::operator++(){
	bool checker = false;

	while(!checker){
        curr_col++;

        if(curr_col == reference.nrOfVertices()){
            curr_row++;
            curr_col = 0;
        }
        if(curr_row == reference.nrOfVertices()) checker = true;
		else if(reference.neigh_matrix.at(curr_row).at(curr_col)) checker = true;
	}
	
	return *this;
}

template<typename V, typename E>
typename Graph<V, E>::EdgesIterator const Graph<V, E>::EdgesIterator::operator++(int){
    auto pom = *this;
    ++(*this);

    return pom;
}

template <typename V, typename E>
inline typename Graph<V, E>::VerticesIterator  Graph<V, E>::insertVertex(const V& vertex_data){
    vertices.push_back(vertex_data);
    neigh_matrix.resize(neigh_matrix.size() + 1);
    neigh_matrix.front().resize(neigh_matrix.front().size() + 1);

    for(auto i = 0; i < neigh_matrix.size(); i++){
        neigh_matrix[i].resize(neigh_matrix.front().size());
    }

    for(auto it = beginVertices(); it != endVertices(); ++it){
        if(*it == vertex_data) return it;
    }

    return endVertices();
}

template <typename V, typename E>
inline std::pair<typename Graph<V, E>::EdgesIterator, bool> Graph<V, E>::insertEdge(size_t vertex1_id, size_t vertex2_id, const E &label, bool replace){
    auto& currentEdge = neigh_matrix[vertex1_id][vertex2_id];

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

    for(auto i = 0; i < neigh_matrix[vertex_id].size(); i++){
        if(i != vertex_id){
            if(neigh_matrix[vertex_id][i]) edges_to_erase++;
        }
    }

    for(auto i = 0; i < neigh_matrix.size(); i++){
        if(neigh_matrix[i][vertex_id]) edges_to_erase++;

        neigh_matrix[i].erase(neigh_matrix[i].begin() + vertex_id);
    }

    neigh_matrix.erase(neigh_matrix.begin() + vertex_id);
    no_of_edges -= edges_to_erase;

    return true;
}

template <typename V, typename E>
inline bool Graph<V, E>::removeEdge(size_t vertex1_id, size_t vertex2_id){
    auto& currentEdge = neigh_matrix[vertex1_id][vertex2_id];

    if(!currentEdge) return false;

    currentEdge.reset();
    no_of_edges--;

    return true;
}

template <typename V, typename E>
inline void Graph<V, E>::printNeighborhoodMatrix() const{
    for(int i = 0; i < neigh_matrix.size(); ++i){
        for(int j = 0; j < neigh_matrix[i].size(); ++j){
            if(neigh_matrix[i][j]) std::cout << neigh_matrix[i][j].value() << " ";
            else std::cout << "0 ";
        }
        std::cout << std::endl;
    }
}

template <typename V, typename E>
inline void Graph<V, E>::dfs(){
	std::vector<bool> visited(vertices.size());

	
}

#endif //SIMPLE_GRAPH_SIMPLE_GRAPH_HPP
