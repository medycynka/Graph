#include <iostream>
#include <functional>
#include "simple_graph.hpp"

using namespace std;

int main(){
	Graph<std::string, double> g;

	for(std::size_t i = 0u; i < 6u; ++i){
		g.insertVertex("data " + std::to_string(i));
	}

	for(std::size_t i = 0u; i < g.nrOfVertices(); ++i){
		for(std::size_t j = 0u; j < g.nrOfVertices(); ++j){
			if((i + j) & 1u || i & 1u){
				g.insertEdge(i, j, ((i != j) ? (i+j)/2. : 1.));
			}
		}
	}

	g.insertEdge(0, 2, 4.);
	std::cout << (g.removeVertex(1) ? "Udalo sie" : "Nie udalo sie") << std::endl;
	std::cout << (g.removeEdge(2, 2) ? "Udalo sie" : "Nie udalo sie") << std::endl;
	std::cout << (g.removeEdge(2, 3) ? "Udalo sie" : "Nie udalo sie") << std::endl;
	std::cout << (g.removeEdge(4, 3) ? "Udalo sie" : "Nie udalo sie") << std::endl;
	std::cout << "Nr of vertices: " << g.nrOfVertices() << std::endl;
	std::cout << "Nr of edges: " << g.nrOfEdges() << std::endl;
	std::cout << std::endl;
	g.printNeighborhoodMatrix();
	std::cout << std::endl;
	std::cout << "Vertices data:" << std::endl;
	for(auto v_it = g.beginVertices(); v_it != g.endVertices(); ++v_it)
	{
	std::cout << *v_it << ", ";
	}
	std::cout << std::endl << std::endl;
	for(auto &v : g)
	{
	std::cout << v << ", ";
	}
	std::cout << std::endl << std::endl;
	std::cout << "Edges data:" << std::endl;
	for(auto e_it = g.beginEdges(); e_it != g.endEdges(); ++e_it)
	{
	std::cout << *e_it << ", ";
	}

	std::cout << std::endl << std::endl;
	std::cout << "DFS vertices data (begin from 1):" << std::endl;
	for(auto dfs_it = g.beginDFS(1); dfs_it != g.endDFS(); ++dfs_it)
	{
	std::cout << *dfs_it << ", ";
	}
	std::cout << std::endl << std::endl;
	std::cout << "DFS vertices data (begin from 3):" << std::endl;
	for(auto dfs_it = g.beginDFS(3); dfs_it != g.endDFS(); ++dfs_it)
	{
	std::cout << *dfs_it << ", ";
	}
	std::cout << std::endl << std::endl;

	std::cout << std::endl << std::endl;
	std::cout << "BFS vertices data (begin from 1):" << std::endl;
	for(auto dfs_it = g.beginBFS(1); dfs_it != g.endBFS(); ++dfs_it)
	{
	std::cout << *dfs_it << ", ";
	}
	std::cout << std::endl << std::endl;
	std::cout << "BFS vertices data (begin from 3):" << std::endl;
	for(auto dfs_it = g.beginBFS(3); dfs_it != g.endBFS(); ++dfs_it)
	{
	std::cout << *dfs_it << ", ";
	}
	std::cout << std::endl << std::endl;
	g.dijkstraShortestPath(2);
	g.dijkstraShortestPath(1);
	g.dijkstraShortestPath(3);
	std::cout << std::endl << std::endl;
	/*auto [shortest_path_distance, shortest_path] = g.dijkstra(2u, 4u, [](const double &e) -> double { return e; });
	std::cout << "Distance from 2 to 4: " << shortest_path_distance << std::endl;
	std::cout << "Path from 2 to 4:" << std::endl;
	for(auto &v_id : shortest_path)
	{
	std::cout << v_id << ", ";
	}
	std::cout << std::endl;

	std::tie(shortest_path_distance, shortest_path) = g.dijkstra(1u, 0u, [](const double &e) -> double { return e; });
	std::cout << "Distance from 1 to 0: " << shortest_path_distance << std::endl;
	std::cout << "Path from 1 to 0:" << std::endl;
	for(auto &v_id : shortest_path)
	{
	std::cout << v_id << ", ";
	}
	std::cout << std::endl;

	std::tie(shortest_path_distance, shortest_path) = g.dijkstra(3u, 0u, [](const double &e) -> double { return e; });
	std::cout << "Distance from 3 to 0: " << shortest_path_distance << std::endl;
	std::cout << "Path from 3 to 0:" << std::endl;
	for(auto &v_id : shortest_path)
	{
	std::cout << v_id << ", ";
	}
	std::cout << std::endl;

	std::tie(shortest_path_distance, shortest_path) = g.dijkstra(3u, 1u, [](const double &e) -> double { return e; });
	std::cout << "Distance from 3 to 1: " << shortest_path_distance << std::endl;
	std::cout << "Path from 3 to 1:" << std::endl;
	for(auto &v_id : shortest_path)
	{
	std::cout << v_id << ", ";
	}
	std::cout << std::endl;

    std::cout << std::endl << std::endl;
    g.computeFloydWarshall();
    std::cout << std::endl << std::endl;
    g.primsMST();
    std::cout << std::endl << (g.hasCycle_undirected() ? "Graph g has cycle" : "Graph g doesn't have cycles") << std::endl;
    std::cout << "Vertex " << *g.vertex(0) << " has " << g.getInDegree(0) << " \"in-vertex\" and " << g.getOutDegree(0) << " \"out-vertices\"" << std::endl << std::endl;

    Graph<int, int> g2;
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
    std::cout << std::endl << (g2.hasCycle_directed() ? "Graph g2 has cycle" : "Graph g2 doesn't have cycles") << std::endl;
    g2.hasHamiltonCycle(1);
    std::cout << "Vertex " << *g2.vertex(0) << " has degree = " << g2.getDegree(0) << std::endl << std::endl;
    g2.findMaxClique();
    std::cout << std::endl;

    Graph<int, int> g3;
    g3.insertVertex(0);
    g3.insertVertex(1);
    g3.insertVertex(2);
    g3.insertEdge(0, 1, 1);
    g3.insertEdge(0, 2, 1);
    g3.insertEdge(2, 1, 1);
    g3.printNeighborhoodMatrix();
    std::cout << std::endl << (g3.hasCycle_undirected() ? "Graph g3 has cycle" : "Graph g3 doesn't have cycles") << std::endl << std::endl;

    Graph<int, int> g4;
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

    Graph<int, int> g5;
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
    g5.findMaxClique();*/
}
