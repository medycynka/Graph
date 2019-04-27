#include <iostream>
#include <vector>
#include "simple_graph.hpp"

using namespace std;

int main(){
    Graph<std::string, int> g;
    g.insertVertex("V1");
    g.insertVertex("V2");
    g.insertVertex("V3");
    g.insertVertex("V4");
    g.insertEdge(0, 0, 1);
    g.insertEdge(0, 1, 2);
    g.insertEdge(1, 2, 3);
    g.insertEdge(2, 2, 4);
    g.insertEdge(3, 2, 5);
    g.insertEdge(3, 0, 6);
    g.insertEdge(0, 3, 7);
    g.insertEdge(1, 3, 8);
    std::cout << "Nr of vertices: " << g.nrOfVertices() << std::endl;
    std::cout << "Nr of edges: " << g.nrOfEdges() << std::endl;
    std::cout << (g.removeEdge(0, 1) ? "Udalo sie" : "Nie udalo sie") << std::endl;
    std::cout << (g.removeEdge(1, 0) ? "Udalo sie" : "Nie udalo sie") << std::endl;
    std::cout << (g.removeVertex(2) ? "Udalo sie" : "Nie udalo sie") << std::endl;
    std::cout << (g.removeVertex(5) ? "Udalo sie" : "Nie udalo sie") << std::endl;
    std::cout << "Nr of vertices: " << g.nrOfVertices() << std::endl;
    std::cout << "Nr of edges: " << g.nrOfEdges() << std::endl;
    g.printNeighborhoodMatrix();

    std::cout << std::endl;
    for(auto &e : g) std::cout << e << ", ";

    std::cout << std::endl;
    for(auto it = g.beginVertices(); it != g.endVertices(); it++) std::cout << *it << ", ";

    std::cout << std::endl;
    for(auto it = g.beginVertices(); it != g.endVertices(); ++it) std::cout << *it << ", ";

    std::cout << std::endl;
    for(auto it = g.beginEdges(); it != g.endEdges(); it++) std::cout << *it << ", ";

    std::cout << std::endl;

    return 0;
}