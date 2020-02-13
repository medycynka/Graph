from typing import TypeVar, Generic, Tuple
import numpy as np
from collections import deque


V = TypeVar('V')
E = TypeVar('E')
INF = np.inf


class Graph(Generic[V, E]):
    def __init__(self, mat_: np.matrix = None):
        self.__vertices = []
        if mat_ is None:
            mat = np.zeros((1, 1))
            self.__matrix = np.full_like(mat, INF)
        else:
            self.__matrix = mat_
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

    def __eq__(self, other: 'Graph[V, E]') -> bool:
        return self.__num_of_edges == other.__num_of_edges and self.__vertices == other.__vertices and \
               self.__matrix == other.__matrix

    def __ne__(self, other: 'Graph[V, E]') -> bool:
        return not self == other

    def __add__(self, other: V or 'tuple(V, V, E)') -> 'Graph[V, E]':
        if isinstance(other, tuple):
            (v1, v2, e_) = other
            self.insert_edge(v1, v2, e_)
        else:
            self.insert_vertex(other)
        return self

    def __radd__(self, other: V or 'tuple(V, V, E)') -> 'Graph[V, E]':
        return self + other

    def __iadd__(self, other: V or 'tuple(V, V, E)') -> 'Graph[V, E]':
        return self + other

    def __sub__(self, other: V or 'tuple(V, V)') -> 'Graph[V, E]':
        if isinstance(other, tuple):
            (v1, v2) = other
            self.remove_edge(v1, v2)
        else:
            self.remove_vertex(other)
        return self

    def __rsub__(self, other: V or 'tuple(V, V)') -> 'Graph[V, E]':
        return self - other

    def __isub__(self, other: V or 'tuple(V, V)') -> 'Graph[V, E]':
        return self - other

    def __getitem__(self, item: V or 'tuple(V, V)'):
        if isinstance(item, tuple):
            (v1, v2) = item
            return self.__matrix[v1][v2]
        else:
            return self.__vertices[item]

    def __setitem__(self, key: V or 'tuple(V, V)', value: V or E):
        if isinstance(key, tuple):
            (v1, v2) = key
            if self.__matrix[v1][v2] == INF:
                self.__num_of_edges += 1
            self.__matrix[v1][v2] = value
        else:
            if value not in self.__vertices:
                self.__vertices[key] = value

    def __delitem__(self, key: V or 'tuple(V, V)'):
        if isinstance(key, tuple):
            (v1, v2) = key
            self.remove_edge(v1, v2)
        else:
            self.remove_vertex(key)

    # iterator krawedzi
    def __iter__(self):
        self.cur_row = 0
        self.cur_col = 0
        checker = self.__matrix[self.cur_row][self.cur_col] != INF
        while not checker:
            self.cur_col += 1
            if self.cur_col == self.get_number_of_vertices():
                self.cur_row += 1
                self.cur_col = 0
            if self.cur_row == self.get_number_of_vertices():
                checker = True
            elif self.__matrix[self.cur_row][self.cur_col] != INF:
                checker = True
        return self

    def __next__(self) -> E:
        if self.cur_row == self.get_number_of_vertices():
            raise StopIteration
        else:
            ret = self.__matrix[self.cur_row][self.cur_col]
            checker = False
            while not checker:
                self.cur_col += 1
                if self.cur_col == self.get_number_of_vertices():
                    self.cur_row += 1
                    self.cur_col = 0
                if self.cur_row == self.get_number_of_vertices():
                    checker = True
                elif self.__matrix[self.cur_row][self.cur_col] != INF:
                    checker = True
            return ret

    def __round__(self, n=None):
        np.round(self.__matrix, n)

    def __floor__(self):
        np.floor(self.__matrix)

    def __ceil__(self):
        np.ceil(self.__matrix)

    def __bool__(self) -> bool:
        return len(self.__vertices) != 0 and self.__num_of_edges != 0

    def __contains__(self, item: V or 'tuple(V, V)') -> bool:
        if isinstance(item, tuple):
            (v1, v2) = item
            return self.__matrix[v1][v2] != INF
        else:
            return item in self.__vertices

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
            if self.__matrix[vertex_id][i] != INF or self.__matrix[i][vertex_id] != INF:
                counter += 1
        counter *= 2
        if self.__matrix[vertex_id][vertex_id] != INF:
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

    def dfs(self, start: int, visitator_f) -> Tuple[str, list]:
        ret_str = "DFS Algorithm:\n"
        ret_list = []
        visited = [False for x in range(len(self.__vertices))]
        s = []
        current = start
        count = 0

        while count < len(self.__vertices):
            if not visited[current]:
                ret_str += "{0}, ".format(visitator_f(self.__vertices[current]))
                ret_list.append(self.__vertices[current])
                visited[current] = True
                count += 1

                for i in range(len(self.__vertices)-1, -1, -1):
                    if self.__matrix[current][i] != INF:
                        s.append(i)
            if len(s) == 0:
                break
            else:
                current = s.pop()
        return ret_str, ret_list

    def bfs(self, start: int, visitator_f) -> Tuple[str, list]:
        ret_str = "BFS Algorithm:\n"
        ret_list = []
        visited = [False for x in range(len(self.__vertices))]
        s = deque([])
        current = start
        count = 0

        while count < len(self.__vertices):
            if not visited[current]:
                ret_str += "{0}, ".format(visitator_f(self.__vertices[current]))
                ret_list.append(self.__vertices[current])
                visited[current] = True
                count += 1

                for i in range(len(self.__vertices) - 1, -1, -1):
                    if self.__matrix[current][i] != INF:
                        s.append(i)
            if len(s) == 0:
                break
            else:
                current = s.popleft()
        return ret_str, ret_list
