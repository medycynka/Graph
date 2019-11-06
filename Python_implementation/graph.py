from typing import TypeVar, Generic
import numpy as np
from collections import deque
from dijkstra_algorithm import dijkstra
from a_star_algorithm import a_star
from max_clique_algorithm import FindMaxClique


V = TypeVar('V')
E = TypeVar('E')
INF = np.inf


class Graph(Generic[V, E]):
    def __init__(self):
        self.__vertices = []
        mat = np.zeros((1, 1))
        self.__matrix = np.full_like(mat, INF)
        self.__num_of_edges = 0

    def __str__(self) -> str:
        ret_str = ""
        for i in range(len(self.__vertices)):
            for j in range(len(self.__vertices)):
                if self.__matrix[i][j] != INF:
                    if j != len(self.__vertices)-1:
                        ret_str += "{0}, ".format(self.__matrix[i][j])
                    else:
                        ret_str += "{0}\n".format(self.__matrix[i][j])
                else:
                    if j != len(self.__vertices)-1:
                        ret_str += "0.0, "
                    else:
                        ret_str += "0.0\n"
        return ret_str

    def __repr__(self) -> str:
        ret_str = "Graph representation (vertex: neighbours):\n"
        for i in range(len(self.__vertices)):
            ret_str += "{0} -> [".format(self.__vertices[i])
            for j in range(len(self.__vertices)):
                if self.__matrix[i][j] != INF:
                    if j != len(self.__vertices)-1:
                        ret_str += "{0}, ".format(self.__vertices[j])
                    else:
                        ret_str += "{0}".format(self.__vertices[j])
            ret_str += "]\n"
        return ret_str

    def get_number_of_vertices(self) -> int:
        return len(self.__vertices)

    def get_number_of_edges(self) -> int:
        return self.__num_of_edges

    def get_vertices(self):
        return self.__vertices

    def get_matrix(self):
        return self.__matrix

    def insert_vertex(self, vertex_data: V):
        self.__vertices.append(vertex_data)
        if len(self.__vertices) > 1:
            self.__matrix = np.insert(self.__matrix, len(self.__matrix), values=INF, axis=0)
            self.__matrix = np.insert(self.__matrix, len(self.__matrix)-1, values=INF, axis=1)

    def insert_edge(self, vertex_id1: int, vertex_id2: int, edge_data: E, replace: bool = False):
        self.__matrix[vertex_id1][vertex_id2] = edge_data
        if not replace:
            self.__num_of_edges += 1

    def remove_vertex(self, vertex_id: int) -> V:
        ret_ = self.__vertices[vertex_id]
        del self.__vertices[vertex_id]
        counter = 0
        for i in range(len(self.__vertices)):
            if self.__matrix[vertex_id][i] != 0 or self.__matrix[i][vertex_id] != 0:
                counter += 1
        if self.__matrix[vertex_id][vertex_id] != 0:
            counter -= 1
        if counter > 0:
            self.__num_of_edges -= counter
        self.__matrix = np.delete(self.__matrix, vertex_id, axis=0)
        self.__matrix = np.delete(self.__matrix, vertex_id, axis=1)
        return ret_

    def remove_edge(self, pos_x, pos_y) -> E:
        ret_ = self.__matrix[pos_x][pos_y]
        self.__matrix[pos_x][pos_y] = INF
        return ret_

    def get_neighbours(self, vertex_id: int):
        neighbours_ = []
        for i in range(len(self.__vertices)):
            if self.__matrix[vertex_id][i] != INF or self.__matrix[i][vertex_id] != INF:
                neighbours_.append(self.__vertices[i])
        return neighbours_

    def dfs(self, start: int, visitator_f) -> str:
        ret_str = "DFS Algorithm:\n"
        visited = [False for x in range(len(self.__vertices))]
        s = []
        current = start
        count = 0

        while count < len(self.__vertices):
            if not visited[current]:
                ret_str += "{0}, ".format(visitator_f(self.__vertices[current]))
                visited[current] = True
                count += 1

                for i in range(len(self.__vertices)-1, -1, -1):
                    if self.__matrix[current][i] != INF:
                        s.append(i)
            if len(s) == 0:
                break
            else:
                current = s.pop()
        return ret_str

    def bfs(self, start: int, visitator_f) -> str:
        ret_str = "BFS Algorithm:\n"
        visited = [False for x in range(len(self.__vertices))]
        s = deque([])
        current = start
        count = 0

        while count < len(self.__vertices):
            if not visited[current]:
                ret_str += "{0}, ".format(visitator_f(self.__vertices[current]))
                visited[current] = True
                count += 1

                for i in range(len(self.__vertices) - 1, -1, -1):
                    if self.__matrix[current][i] != INF:
                        s.append(i)
            if len(s) == 0:
                break
            else:
                current = s.popleft()
        return ret_str

    """ GRAPH Algorithms """
    def __check_edge(self, u: int, v: int, vis) -> bool:
        if u == v or (not vis[u] and not vis[v]):
            return False
        else:
            return not (vis[u] and vis[v])

    def prim_mst(self):
        print("Prim's algorithm for MST:")
        counter = 0
        min_cost = 0.0
        size_ = len(self.__vertices)
        visited = [False for x in range(size_)]
        visited[0] = True

        while counter < size_ - 1:
            min_ = INF
            a = -1
            b = -1
            for i in range(size_):
                for j in range(size_):
                    if self.__matrix[i][j] < min_:
                        if self.__check_edge(i, j, visited):
                            min_ = self.__matrix[i][j]
                            a = i
                            b = j

            if a != -1 and b != -1:
                counter += 1
                print("Edge: {0} [{1}, {2}] cost: {3}".format(counter, self.__vertices[a], self.__vertices[b], min_))
                min_cost += min_
                visited[a] = True
                visited[b] = True

        print("Minimum cost of MST = {0}\n".format(min_cost))

    # Only for undirected graphs
    def __is_valid_color(self, vertex_id: int, col, col_checker: int) -> bool:
        for i in range(len(self.__vertices)):
            if self.__matrix[vertex_id][i] != INF and col_checker == col[i]:
                return False
        return True

    def __try_graph_coloring(self, nr_of_colors_to_try: int, vertex_id: int, col) -> bool:
        size_ = len(self.__vertices)
        if vertex_id == size_:
            return True

        for i in range(1, size_+1):
            if self.__is_valid_color(vertex_id, col, i):
                col[vertex_id] = i
                if self.__try_graph_coloring(nr_of_colors_to_try, vertex_id+1, col):
                    return True
                col[vertex_id] = 0

        return False

    def __print_colors(self, col):
        print("Solution for coloring vertices:")
        for i in range(len(self.__vertices)):
            print("{0} color: {1}".format(self.__vertices[i], col[i]))

    def check_coloring_result(self, nr_of_colors_to_try: int) -> bool:
        colors = [0 for x in range(len(self.__vertices))]

        if not self.__try_graph_coloring(nr_of_colors_to_try, 0, colors):
            print("Couldn't find solution for {0} colors".format(nr_of_colors_to_try))
            return False
        else:
            self.__print_colors(colors)
            return True

    def color_until_done(self):
        size_ = len(self.__vertices)
        colors = [0 for x in range(len(self.__vertices))]

        while not self.__try_graph_coloring(1, 0, colors): pass

        self.__print_colors(colors)
        chromatic_number = max(colors)

        return chromatic_number, chromatic_number < size_

    def find_max_clique(self, print_and_return: bool = False):
        size_ = self.get_number_of_vertices()
        temp_mat = np.zeros((size_, size_), dtype=bool)
        for i in range(size_):
            for j in range(size_):
                if self.__matrix[i][j] != INF:
                    temp_mat[i][j] = True
        mc = FindMaxClique(temp_mat, size_, 0.025)
        max_clique = []
        mc.max_clique_algorithm(max_clique)
        if print_and_return:
            print("Maximum clique:\n"
                  "{0}\n"
                  "Size = {1}\n"
                  "Number of steps = {2}\n".format(max_clique, len(max_clique), mc.step_cout))
            return max_clique
        else:
            return max_clique

    def __check_vertices_dfs(self, curr: int, vis, parent: int) -> bool:
        vis.add(curr)
        for i in range(self.get_number_of_vertices()):
            if self.__matrix[curr][i] != INF:
                if i in vis:
                    return True
                if self.__check_vertices_dfs(i, vis, curr):
                    return True

    def has_cycles_undirected(self) -> bool:
        checker = set()
        for i in range(self.get_number_of_vertices()):
            if self.__check_vertices_dfs(i, checker, -1):
                return True
        return False

    def __is_safe_vertex(self, v: int, pos: int, path) -> bool:
        if self.__matrix[path[pos-1]][v] == INF:
            return False
        for i in path:
            if i == v:
                return False
        return True

    def __try_finding_hamilton_cycle(self, path, pos: int) -> bool:
        if pos == len(self.__vertices):
            if self.__matrix[path[pos-1]][path[0]] != INF:
                return True
            else:
                return False

        for i in range(1, len(self.__vertices)):
            if self.__is_safe_vertex(i, pos, path):
                path[pos] = i
                if self.__try_finding_hamilton_cycle(path, pos+1):
                    return True
                path[pos] = -1
        return False

    def has_hamilton_cycle(self, print_and_return: bool = False) -> bool:
        path = [-1 for x in range(self.get_number_of_vertices())]
        path[0] = 0

        if self.__try_finding_hamilton_cycle(path, 1):
            if print_and_return:
                print("Hamilton cycle: {0}".format(path + [path[0]]))
            return True
        else:
            if print_and_return:
                print("This graph doesn't have Hamilton cycle")
            return False

    def find_eulerian_path(self):
        n = self.get_number_of_vertices()
        numofadj = []
        temp = np.zeros_like(self.__matrix)
        for i in range(n):
            for j in range(n):
                if self.__matrix[i][j] != INF:
                    temp[i][j] = 1
        for i in range(n):
            numofadj.append(sum(temp[i]))
        starrtpoint = 0
        numofodd = 0
        for i in range(n-1, -1, -1):
            if numofadj[i] % 2 == 1:
                numofodd += 1
                starrtpoint = i

        if numofodd > 2:
            print("There is no Eulerian path in this graph")
            return []

        stack = []
        path = []
        curr = starrtpoint
        while len(stack) != 0 or sum(temp[curr]) != 0:
            if sum(temp[curr]) == 0:
                path.append(curr+1)
                curr = stack.pop(-1)
            else:
                for i in range(n):
                    if temp[curr][i] == 1:
                        stack.append(curr)
                        temp[curr][i] = 0
                        temp[i][curr] = 0
                        curr = i
                        break

        print("Eulerian path: {0}".format(path + [curr+1]))
        return path + [curr+1]


if __name__ == '__main__':
    g = Graph[int, int]()
    g.insert_vertex(0)
    g.insert_vertex(1)
    g.insert_vertex(2)
    g.insert_vertex(3)
    g.insert_vertex(4)
    g.insert_edge(0, 1, 1)
    g.insert_edge(0, 3, 1)
    g.insert_edge(1, 0, 1)
    g.insert_edge(1, 2, 1)
    g.insert_edge(1, 3, 1)
    g.insert_edge(1, 4, 1)
    g.insert_edge(2, 1, 1)
    g.insert_edge(2, 4, 1)
    g.insert_edge(3, 0, 1)
    g.insert_edge(3, 1, 1)
    g.insert_edge(3, 4, 1)
    g.insert_edge(4, 1, 1)
    g.insert_edge(4, 2, 1)
    g.insert_edge(4, 3, 1)
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
