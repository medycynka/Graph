

class FindMaxClique:
    class Vertices:
        class Vertex:
            def __init__(self, id_ = -1):
                self.index_ = id_
                self.degree = -1

            def set_i(self, ii):
                self.index_ = ii

            def get_i(self) -> int:
                return self.index_

            def set_degree(self, dd):
                self.degree = dd

            def get_degree(self) -> int:
                return self.degree

        def __init__(self):
            self.vertices_ = []

        def dispose(self):
            self.vertices_.clear()

        def sort(self):
            sorted(self.vertices_, key=lambda vertex: vertex.get_degree(), reverse=True)

        def size(self) -> int:
            return len(self.vertices_)

        def push(self, ii):
            self.vertices_.append(self.Vertex(ii))

        def pop(self):
            self.vertices_.pop()

        def __getitem__(self, item) -> Vertex:
            return self.vertices_[item]

        def __setitem__(self, key, value):
            self.vertices_[key] = value

        def end(self) -> Vertex:
            return self.vertices_[self.size()-1]

        def init_colors(self):
            max_degree = self.vertices_[0].get_degree()

            for i in range(max_degree):
                self.vertices_[i].set_degree(max_degree+1)

            for i in range(max_degree, self.size()):
                self.vertices_[i].set_degree(max_degree + 1)

        def set_degrees(self, m: 'FindMaxClique'):
            psize_ = self.size()

            for i in range(psize_):
                d = 0

                for j in range(psize_):
                    if m.is_connection(self.vertices_[i].get_i(), self.vertices_[j].get_i()):
                        d += 1

                self.vertices_[i].set_degree(d)

    class ColorVector:
        def __init__(self, s_: int = 0):
            self.colors = []
            self.init_(s_)

        def init_(self, csize_: int):
            self.colors = [None for x in range(csize_)]

        def push(self, ii: int):
            self.colors.append(ii)

        def pop(self):
            self.colors.pop()

        def rewind(self):
            self.colors.clear()

        def size(self) -> int:
            return len(self.colors)

        def __getitem__(self, item) -> int:
            return self.colors[item]

        def __setitem__(self, key, value):
            self.colors[key] = value

    class Counter:
        def __init__(self):
            self.i1 = 0
            self.i2 = 0

        def set_i1(self, ii: int):
            self.i1 = ii

        def get_i1(self) -> int:
            return self.i1

        def set_i2(self, ii: int):
            self.i2 = ii

        def get_i2(self) -> int:
            return self.i2

        def inc_i1(self):
            self.i1 += 1

        def inc_i2(self):
            self.i2 += 1

        def set_both(self, i1_: int, i2_: int):
            self.i1 = i1_
            self.i2 = i2_

    def __init__(self, matrix, size_: int, tt: float):
        self.V = self.Vertices()
        self.C = [self.ColorVector() for x in range(size_+2)]
        self.QMAX = self.ColorVector(size_)
        self.Q = self.ColorVector(size_)
        self.grid_ = matrix
        self.step_cout = 0
        self.level = 1
        self.time_search_limit = tt
        self.S = [self.Counter() for x in range(size_+2)]

        for i in range(size_):
            self.V.push(i)

        for i in range(size_+1):
            self.C[i].init_(size_+1)

    def steps(self) -> int:
        return self.step_cout

    def max_clique_algorithm(self, max_clique):
        self.__max_clique_algorithm(max_clique)

    def is_connection(self, i: int, j: int) -> bool:
        return self.grid_[i][j]

    def __max_clique_algorithm(self, max_clique):
        self.V.set_degrees(self)
        self.V.sort()
        self.V.init_colors()

        for i in range(self.V.size()+1):
            self.S[i].set_both(0, 0)

        self.__expand(self.V)
        max_clique = (max_clique[:self.QMAX.size()] + [self.ColorVector()]*(self.QMAX.size() - len(max_clique)))
        for i in range(self.QMAX.size()):
            max_clique[i] = self.QMAX[i]
    
    def __first_cut(self, pi: int, A: ColorVector):
        for i in range(A.size()):
            if self.is_connection(pi, A[i]):
                return True
    
    def __second_cut(self, A: Vertices, B: Vertices):
        for i in range(A.size()-1):
            if self.is_connection(A.end().get_i(), A[i].get_i()):
                B.push(A[i].get_i())
    
    def __sort_by_colors(self, R: Vertices):
        j = 0
        maxno = 1
        min_k = self.QMAX.size() - self.Q.size() + 1
        self.C[1].rewind()
        self.C[2].rewind()
        
        for i in range(R.size()):
            pi = R[i].get_i()
            k = 1
            
            while self.__first_cut(pi, self.C[k]):
                k += 1
            
            if k > maxno:
                maxno = k
                self.C[maxno+1].rewind()
            
            self.C[k].push(pi)
            
            if k < min_k:
                R[j].set_degree(k)
                j += 1
        
        if j > 0:
            R[j-1].set_degree(0)
        if min_k <= 0:
            min_k = 1
            
        for k in range(min_k, maxno+1):
            for i in range(self.C[k].size()):
                R[j].set_i(self.C[k][i])
                R[j].set_degree(k)
                j += 1

    def __expand(self, R: Vertices):
        self.S[self.level].set_both(self.S[self.level].get_i1() + self.S[self.level-1].get_i1() - 
                                    self.S[self.level].get_i2(), self.S[self.level-1].get_i1())
        while R.size() != 0:
            if self.Q.size() + R.end().get_degree() > self.QMAX.size():
                self.Q.push(R.end().get_i())
                Rp = self.Vertices()
                self.__second_cut(R, Rp)
                if Rp.size() != 0:
                    self.step_cout += 1
                    if float(self.S[self.level].get_i1())/self.step_cout < self.time_search_limit:
                        self.__sort_by_degree(Rp)
                    self.__sort_by_colors(Rp)
                    self.S[self.level].inc_i1()
                    self.level += 1
                    self.__expand(Rp)
                    self.level -= 1
                elif self.Q.size() > self.QMAX.size():
                    # print("step = {0}, current max clique size = {1}".format(self.step_cout, self.Q.size()))
                    self.QMAX = self.Q
                Rp.dispose()
                self.Q.pop()
            else:
                return 
            R.pop()

    def __sort_by_degree(self, R: Vertices):
        R.set_degrees(self)
        R.sort()


def desc_degree(vi: FindMaxClique.Vertices.Vertex, vj: FindMaxClique.Vertices.Vertex) -> bool:
    return vi.get_degree() > vj.get_degree()
