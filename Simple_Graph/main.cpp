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
    for(auto v_it = g.beginVertices(); v_it != g.endVertices(); ++v_it){
        std::cout << *v_it << ", ";
    }

    std::cout << std::endl << std::endl;

    std::cout << "Edges data:" << std::endl;
    for(auto e_it = g.beginEdges(); e_it != g.endEdges(); ++e_it){
        std::cout << *e_it << ", ";
    }

    std::cout << std::endl << std::endl;

    std::function<void(std::string)> func = [](std::string s){ std::cout << s << ", "; };
    g.dfs(1, func);
    std::cout << "DFS iterator:" << std::endl;
    for(auto it = g.beginDFS(1); it != g.endDFS(); it++) std::cout << *it << ", ";

    std::cout << std::endl;
    g.dfs(3, func);
    std::cout << "DFS iterator:" << std::endl;
    for(auto it = g.beginDFS(3); it != g.endDFS(); ++it) std::cout << *it << ", ";

    std::cout << std::endl << std::endl;

    g.bfs(1, func);
    std::cout << "BFS iterator:" << std::endl;
    for(auto it = g.beginBFS(1); it != g.endBFS(); it++) std::cout << *it << ", ";

    std::cout << std::endl;
    g.bfs(3, func);
    std::cout << "BFS iterator:" << std::endl;
    for(auto it = g.beginBFS(3); it != g.endBFS(); it++) std::cout << *it << ", ";

    std::cout << std::endl << std::endl;

    /*g.dijkstraShortestPath(1);
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
    g4.hasHamiltonCycle(1);*/
}