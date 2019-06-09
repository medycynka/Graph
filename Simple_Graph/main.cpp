#include <iostream>
#include <functional>
#include <cstdint>
#include <cmath>
#include "graph.hpp"
#include "dijkstra.hpp"
#include "astar.hpp"
#include <iterator>

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
        auto d = std::make_pair(std::abs(v1_data.first - v2_data.first), std::abs(v1_data.second - v2_data.second));

        return static_cast<double>(d.first + d.second);
    };

    auto euclidean_heuristics = [](const Graph<std::pair<float, float>, double> &graph, std::size_t current_vertex_id, std::size_t end_vertex_id) -> double {
        const auto &v1_data = graph.vertexData(current_vertex_id);
        const auto &v2_data = graph.vertexData(end_vertex_id);
        auto d = std::make_pair(std::abs(v1_data.first - v2_data.first), std::abs(v1_data.second - v2_data.second));

        //return std::sqrt(std::pow(v2_data.first-v1_data.first, 2u)+std::pow(v2_data.second-v1_data.second, 2u));
        return static_cast<double>(std::sqrt((d.first * d.first) + (d.second * d.second)));
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


//    for(auto v = g.beginVertices(); v != g.endVertices(); ++v) std::cout<<"("<<(*v).first<<","<<(*v).second<<"),";
//    std::cout<<std::endl;
//    for(auto e = g.beginEdges(); e != g.endEdges(); ++e)std::cout<<*e<<", ";
//    std::cout<<std::endl<<std::endl;


    auto start_data = std::make_pair(grid_size/2.f, 1.f);
    auto end_data = std::make_pair(grid_size/2.f+1.f, grid_size-1.f);
    auto start_it = std::find(g.beginVertices(), g.endVertices(), start_data);
    auto end_it = std::find(g.beginVertices(), g.endVertices(), end_data);
    if(start_it != g.endVertices() && end_it != g.endVertices())
    {
        //auto start = std::chrono::system_clock::now();
        auto [shortest_path_distance, shortest_path] = dijkstra<std::pair<float, float>, double>(g, start_it.id(), end_it.id(), [](const double &e) -> double { return e; });
        //auto end = std::chrono::system_clock::now();
        //std::chrono::duration<double> elapsed_seconds = end - start;
        //std::cout << "dijkstra v1 took " << elapsed_seconds.count() << "s" << std::endl;
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

        std::tie(shortest_path_distance, shortest_path) = astar<std::pair<float, float>, double>(g, start_it.id(), end_it.id(), zero_heuristics, [](const double &e) -> double { return e; });
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

        std::tie(shortest_path_distance, shortest_path) = astar<std::pair<float, float>, double>(g, start_it.id(), end_it.id(), manhattan_heuristics, [](const double &e) -> double { return e; });
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

        std::tie(shortest_path_distance, shortest_path) = astar<std::pair<float, float>, double>(g, start_it.id(), end_it.id(), euclidean_heuristics, [](const double &e) -> double { return e; });
        std::cout << "AStar (euclidean) results:" << std::endl;
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
    }

    return 0;
}