#include <iostream>
#include <functional>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include "simple_graph.hpp"

using namespace std;

int main()
{
Graph<std::pair<float, float>, double> g;

constexpr std::size_t grid_size = 16u;
const auto sqrt_2 = std::sqrt(2.);

auto zero_heuristics = [](const Graph<std::pair<float, float>, double> &graph, std::size_t current_vertex_id, std::size_t end_vertex_id) -> double {
return 0.;
};

auto manhattan_heuristics = [](const Graph<std::pair<float, float>, double> &graph, std::size_t current_vertex_id, std::size_t end_vertex_id) -> double {
const auto &v1_data = graph.vertexData(current_vertex_id);
const auto &v2_data = graph.vertexData(end_vertex_id);
return std::abs(v1_data.first-v2_data.first) + std::abs(v1_data.second-v2_data.second);
};

auto euclidean_heuristics = [](const Graph<std::pair<float, float>, double> &graph, std::size_t current_vertex_id, std::size_t end_vertex_id) -> double {
const auto &v1_data = graph.vertexData(current_vertex_id);
const auto &v2_data = graph.vertexData(end_vertex_id);
return std::sqrt(std::pow(v2_data.first-v1_data.first, 2u)+std::pow(v2_data.second-v1_data.second, 2u));
};

for(std::size_t i = 0u; i < grid_size; ++i)
{
for(std::size_t j = 0u; j < grid_size; ++j)
{
g.insertVertex(std::make_pair(i, j));
}
}
for(std::size_t i = 0u; i < grid_size; ++i)
{
for(std::size_t j = 0u; j < grid_size - 1u; ++j)
{
if(j < grid_size - 1u)
{
g.insertEdge(i * grid_size + j, i * grid_size + j + 1u, 1.);
g.insertEdge(i * grid_size + j + 1u, i * grid_size + j, 1.);
}
if(i < grid_size - 1u)
{
g.insertEdge(i * grid_size + j, (i + 1u) * grid_size + j, 1.);
g.insertEdge((i + 1u) * grid_size + j, i * grid_size + j, 1.);
}
if(i < grid_size - 1u && j < grid_size - 1u)
{
g.insertEdge(i * grid_size + j, (i + 1u) * grid_size + j + 1u, sqrt_2);
g.insertEdge((i + 1u) * grid_size + j + 1u, i * grid_size + j, sqrt_2);
g.insertEdge(i * grid_size + j + 1u, (i + 1u) * grid_size + j, sqrt_2);
g.insertEdge((i + 1u) * grid_size + j, i * grid_size + j + 1u, sqrt_2);
}
}
}

for(std::size_t j = 1u; j < grid_size - 1u; ++j)
{
g.removeVertex(std::find(g.beginVertices(), g.endVertices(), std::make_pair(static_cast<float>(j), grid_size/2.f)).id());
}

auto start_data = std::make_pair(grid_size/2.f, 1.f);
auto end_data = std::make_pair(grid_size/2.f+1.f, grid_size-1.f);
auto start_it = std::find(g.beginVertices(), g.endVertices(), start_data);
auto end_it = std::find(g.beginVertices(), g.endVertices(), end_data);
if(start_it != g.endVertices() && end_it != g.endVertices())
{
auto [shortest_path_distance, shortest_path] = g.dijkstra(start_it.id(), end_it.id(), [](const double &e) -> double { return e; });
std::cout << "Dijkstra results:" << std::endl;
std::cout << "\tDistance: " << shortest_path_distance << std::endl;
std::cout << "\tPath (ids): ";
for(auto &v_id : shortest_path)
{
std::cout << v_id << ", ";
}
std::cout << std::endl;
std::cout << "\tPath (data): ";
for(auto &v_id : shortest_path)
{
std::cout << "[" << g.vertexData(v_id).first << ", " << g.vertexData(v_id).second << "]" << ", ";
}
std::cout << std::endl;

std::tie(shortest_path_distance, shortest_path) = g.AStar(start_it.id(), end_it.id(), [](const double &e) -> double { return e; }, zero_heuristics);
std::cout << "AStar (zero) results:" << std::endl;
std::cout << "\tDistance: " << shortest_path_distance << std::endl;
std::cout << "\tPath (ids): ";
for(auto &v_id : shortest_path)
{
std::cout << v_id << ", ";
}
std::cout << std::endl;
std::cout << "\tPath (data): ";
for(auto &v_id : shortest_path)
{
std::cout << "[" << g.vertexData(v_id).first << ", " << g.vertexData(v_id).second << "]" << ", ";
}
std::cout << std::endl;

std::tie(shortest_path_distance, shortest_path) = g.AStar(start_it.id(), end_it.id(), [](const double &e) -> double { return e; }, manhattan_heuristics);
std::cout << "AStar (manhattan) results:" << std::endl;
std::cout << "\tDistance: " << shortest_path_distance << std::endl;
std::cout << "\tPath (ids): ";
for(auto &v_id : shortest_path)
{
std::cout << v_id << ", ";
}
std::cout << std::endl;
std::cout << "\tPath (data): ";
for(auto &v_id : shortest_path)
{
std::cout << "[" << g.vertexData(v_id).first << ", " << g.vertexData(v_id).second << "]" << ", ";
}
std::cout << std::endl;

std::tie(shortest_path_distance, shortest_path) = g.AStar(start_it.id(), end_it.id(), [](const double &e) -> double { return e; }, euclidean_heuristics);
std::cout << "AStar (euclidean) results:" << std::endl;
std::cout << "\tDistance: " << shortest_path_distance << std::endl;
std::cout << "\tPath (ids): ";
for(auto &v_id : shortest_path)
{
std::cout << v_id << ", ";
}
std::cout << std::endl;
std::cout << "\tPath (data): ";
for(auto &v_id : shortest_path){
std::cout << "[" << g.vertexData(v_id).first << ", " << g.vertexData(v_id).second << "]" << ", ";
}
std::cout << std::endl;
}
    std::cout << std::endl << std::endl;

    /*std::cout << std::endl << std::endl;
    g.computeFloydWarshall();
    std::cout << std::endl << std::endl;
    g.primsMST();
    std::cout << std::endl << (g.hasCycle_undirected() ? "Graph g has cycle" : "Graph g doesn't have cycles") << std::endl;
    std::cout << "Vertex " << *g.vertex(0) << " has " << g.getInDegree(0) << " \"in-vertex\" and " << g.getOutDegree(0) << " \"out-vertices\"" << std::endl << std::endl;

    Graph<size_t, size_t> g2;
    g2.insertVertex(0);
    g2.insertVertex(1);
    g2.insertVertex(2);
    g2.insertVertex(3);
    g2.insertEdge(0, 1, 1);
    g2.insertEdge(0, 2, 1);
    g2.insertEdge(0, 3, 1);
    g2.insertEdge(1, 0, 1);
    g2.insertEdge(1, 2, 1);
    g2.insertEdge(2, 0, 1);
    g2.insertEdge(2, 1, 1);
    g2.insertEdge(2, 3, 1);
    g2.insertEdge(3, 2, 1);
    g2.insertEdge(3, 0, 1);
    g2.printNeighborhoodMatrix();
    g2.checkColoringResult(1);
    g2.checkColoringResult(2);
    g2.checkColoringResult(3);
    g2.colorUntilDone();
    std::cout << std::endl << (g2.hasCycle_directed() ? "Graph g2 has cycle" : "Graph g2 doesn't have cycles") << std::endl;
    g2.hasHamiltonCycle(1);
    std::cout << "Vertex " << *g2.vertex(0) << " has degree = " << g2.getDegree(0) << std::endl << std::endl;
    g2.findMaxClique();
    std::cout << std::endl;

    Graph<size_t, size_t> g3;
    g3.insertVertex(0);
    g3.insertVertex(1);
    g3.insertVertex(2);
    g3.insertEdge(0, 1, 1);
    g3.insertEdge(0, 2, 1);
    g3.insertEdge(2, 1, 1);
    g3.printNeighborhoodMatrix();
    std::cout << std::endl << (g3.hasCycle_undirected() ? "Graph g3 has cycle" : "Graph g3 doesn't have cycles") << std::endl << std::endl;
    Graph<size_t, size_t> g4;
    g4.insertVertex(0);
    g4.insertVertex(1);
    g4.insertVertex(2);
    g4.insertVertex(3);
    g4.insertEdge(0, 1, 1);
    g4.insertEdge(0, 2, 1);
    g4.insertEdge(0, 3, 1);
    g4.insertEdge(1, 0, 1);
    g4.insertEdge(2, 0, 1);
    g4.insertEdge(2, 3, 1);
    g4.insertEdge(3, 2, 1);
    g4.insertEdge(3, 0, 1);
    g4.printNeighborhoodMatrix();
    g4.hasHamiltonCycle(1);
    std::cout << std::endl;
    
    Graph<size_t, size_t> g5;
    g5.insertVertex(0);
    g5.insertVertex(1);
    g5.insertVertex(2);
    g5.insertVertex(3);
    g5.insertVertex(4);
    g5.insertVertex(5);
    g5.insertVertex(6);
    g5.insertVertex(7);
    g5.insertEdge(0, 1, 1);
    g5.insertEdge(0, 2, 1);
    g5.insertEdge(0, 5, 1);
    g5.insertEdge(0, 6, 1);
    g5.insertEdge(1, 0, 1);
    g5.insertEdge(1, 2, 1);
    g5.insertEdge(1, 3, 1);
    g5.insertEdge(1, 5, 1);
    g5.insertEdge(1, 6, 1);
    g5.insertEdge(2, 0, 1);
    g5.insertEdge(2, 1, 1);
    g5.insertEdge(2, 4, 1);
    g5.insertEdge(2, 5, 1);
    g5.insertEdge(2, 6, 1);
    g5.insertEdge(2, 7, 1);
    g5.insertEdge(3, 1, 1);
    g5.insertEdge(3, 4, 1);
    g5.insertEdge(3, 6, 1);
    g5.insertEdge(3, 7, 1);
    g5.insertEdge(4, 2, 1);
    g5.insertEdge(4, 3, 1);
    g5.insertEdge(4, 6, 1);
    g5.insertEdge(5, 0, 1);
    g5.insertEdge(5, 1, 1);
    g5.insertEdge(5, 2, 1);
    g5.insertEdge(5, 6, 1);
    g5.insertEdge(5, 7, 1);
    g5.insertEdge(6, 0, 1);
    g5.insertEdge(6, 1, 1);
    g5.insertEdge(6, 2, 1);
    g5.insertEdge(6, 3, 1);
    g5.insertEdge(6, 4, 1);
    g5.insertEdge(6, 5, 1);
    g5.insertEdge(6, 7, 1);
    g5.insertEdge(7, 2, 1);
    g5.insertEdge(7, 3, 1);
    g5.insertEdge(7, 5, 1);
    g5.insertEdge(7, 6, 1);
    g5.printNeighborhoodMatrix();
    std::cout << std::endl;
    g5.findMaxClique();
    std::cout << std::endl;*/
}
