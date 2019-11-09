from graph import Graph
from dijkstra_algorithm import dijkstra
from a_star_algorithm import a_star
from timeit import default_timer as timer
import networkx as nx
import numpy as np

if __name__ == '__main__':
    g = Graph[int, int]()
    g.insert_vertex(0)
    g.insert_vertex(1)
    g.insert_vertex(2)
    g.insert_vertex(3)
    g += 4
    g.insert_edge(0, 1, 0.5)
    g.insert_edge(0, 3, 0.5)
    g.insert_edge(1, 0, 0.5)
    g.insert_edge(1, 2, 1)
    g.insert_edge(1, 3, 1)
    g.insert_edge(1, 4, 1)
    g.insert_edge(2, 1, 1)
    g.insert_edge(2, 4, 1.5)
    g.insert_edge(3, 0, 0.5)
    g.insert_edge(3, 1, 1)
    g.insert_edge(3, 4, 2)
    g.insert_edge(4, 1, 1)
    g.insert_edge(4, 2, 1.5)
    g += (4, 3, 2)
    print(g)
    print(repr(g))
    print(g.dfs(0, lambda x: x))
    print(g.bfs(0, lambda x: x), "\n")
    g.prim_mst()
    cd, pd = dijkstra(g, 3, 1)
    ca, pa = a_star(g, 3, 1)
    print("\nDijkstra algorithm from node 3 to node 1:\nCost = {0}, path: {1}".format(cd, pd))
    print("A* algorithm from node 3 to node 1:\nCost = {0}, path: {1}\n".format(ca, pa))
    g.color_until_done()
    g.has_hamilton_cycle(True)
    g.find_eulerian_path()
    g.find_max_clique(print_and_return=True)
    g.fin_all_cliques_of_given_size(size_=2, print_and_return=True)
