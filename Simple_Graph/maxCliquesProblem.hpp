#ifndef SIMPLE_GRAPH_MAXCLIQUESPROBLEM_HPP
#define SIMPLE_GRAPH_MAXCLIQUESPROBLEM_HPP

#pragma once
#include <iostream>
#include <algorithm>
#include <cassert>
#include <optional>
#include <algorithm>
#include <limits>

class FindMaxClique{
        class Vertices{
                class Vertex{
                    public:
                        inline void    set_i(const ssize_t ii) { i = ii; };
                        inline ssize_t get_i()           const { return i; };
                        inline void    set_degree(ssize_t dd)  { d = dd; };
                        inline ssize_t get_degree()      const { return d; };

                    private:
                        ssize_t i;
                        ssize_t d;
                };

                inline static bool desc_degree(const Vertex vi, const Vertex vj) { return (vi.get_degree() > vj.get_degree()); };

            public:
                explicit Vertices(ssize_t size) : size_(0) { v = new Vertex[size]; };
                ~Vertices() = default;

                inline void    dispose()                  { delete [] v; };
                inline void    sort()                     { std::sort(v, v+size_, desc_degree); };
                inline ssize_t size()               const { return size_; };
                inline void    push(const ssize_t ii)     { v[size_++].set_i(ii); };
                inline void    pop()                      { size_--; };
                inline Vertex& at(const ssize_t ii) const { return v[ii]; };
                inline Vertex& end()                const { return v[size_ - 1]; };
                inline void    init_colors();
                inline void    set_degrees(FindMaxClique&);

            private:
                Vertex *v;
                ssize_t size_;
        };

        class Coloring{
            public:
                Coloring()                             : size_(0), i(nullptr){};
                explicit Coloring(const ssize_t size_) : size_(size_), i(nullptr){ init(size_); };
                ~Coloring(){ delete [] i; };

                inline void     init(const ssize_t csize_) { i = new ssize_t[csize_]; rewind(); };
                inline void     push(const ssize_t ii)     { i[size_++] = ii; };
                inline void     pop()                      { size_--; };
                inline void     rewind()                   { size_ = 0; };
                inline ssize_t  size()               const { return size_; };
                inline ssize_t& at(const ssize_t ii) const { return i[ii]; };
                Coloring& operator=(const Coloring& dh){ for(auto j = 0; j < dh.size_; j++){ i[j] = dh.i[j]; } size_ = dh.size_; return *this; };

            private:
                ssize_t *i;
                ssize_t size_;
        };

        class Counter{
            public:
                Counter() : i1(0), i2(0){};

                inline void     set_i1(const ssize_t ii) { i1 = ii; };
                inline ssize_t  get_i1()           const { return i1; };
                inline void     set_i2(const ssize_t ii) { i2 = ii; };
                inline ssize_t  get_i2()           const { return i2; };
                inline void     inc_i1()                 { i1++; };

            private:
                ssize_t i1;
                ssize_t i2;
        };

        inline bool isConnection(const ssize_t i, const ssize_t j) const { return e[i][j]; };
        inline bool firstCut(ssize_t, const Coloring&);
        inline void secondCut(const Vertices&, Vertices&);
        inline void SortByColors(Vertices&);
        inline void expand(Vertices);
        inline void _maxCliqueAlgorithm(ssize_t*&, ssize_t&);
        inline void sortByDegree(Vertices &R) { R.set_degrees(*this); R.sort(); };

    public:
        FindMaxClique(const bool* const*, ssize_t, float = 0.025);
        ~FindMaxClique(){ delete [] C; delete [] S; V.dispose(); };

        inline ssize_t steps() const { return pk; };
        inline void    maxCliqueAlgorithm(ssize_t* &maxclique, ssize_t &size_) { _maxCliqueAlgorithm(maxclique, size_); };

    private:
        Vertices V;
        Coloring *C;
        Coloring QMAX;
        Coloring Q;
        const bool* const* e;
        ssize_t pk;
        ssize_t level;
        const float timeSearchLimit;
        Counter *S;
};

FindMaxClique::FindMaxClique(const bool* const* conn, const ssize_t size_, const float tt) : pk(0), level(1), timeSearchLimit(tt), V(size_), Q(size_), QMAX(size_) {
    assert(conn != nullptr && size_>0);
    for(auto i = 0; i < size_; i++) V.push(i);

    e = conn;
    C = new Coloring[size_ + 1];

    for(auto i=0; i < size_ + 1; i++) C[i].init(size_ + 1);

    S = new Counter[size_ + 1];
}

inline void FindMaxClique::_maxCliqueAlgorithm(ssize_t* &maxclique, ssize_t &size_) {
    V.set_degrees(*this);
    V.sort();
    V.init_colors();

    for(auto i = 0; i < V.size() + 1; i++) {
        S[i].set_i1(0);
        S[i].set_i2(0);
    }

    expand(V);
    maxclique = new ssize_t[QMAX.size()];

    for(auto i=0; i<QMAX.size(); i++) maxclique[i] = QMAX.at(i);

    size_ = QMAX.size();
}

inline void FindMaxClique::Vertices::init_colors() {
    const ssize_t max_degree = v[0].get_degree();

    for(auto i = 0; i < max_degree; i++) v[i].set_degree(i + 1);

    for(auto i = max_degree; i < size_; i++) v[i].set_degree(max_degree + 1);
}

inline void FindMaxClique::Vertices::set_degrees(FindMaxClique &m) {
    for(auto i = 0; i < size_; i++){
        auto d = 0;

        for(auto j = 0; j < size_; j++) if(m.isConnection(v[i].get_i(), v[j].get_i())) d++;

        v[i].set_degree(d);
    }
}

inline bool FindMaxClique::firstCut(const ssize_t pi, const Coloring &A) {
    for(auto i = 0; i < A.size(); i++)  if(isConnection(pi, A.at(i))) return true;

    return false;
}

inline void FindMaxClique::secondCut(const Vertices &A, Vertices &B) {
    for(auto i = 0; i < A.size() - 1; i++)if (isConnection(A.end().get_i(), A.at(i).get_i())) B.push(A.at(i).get_i());
}

inline void FindMaxClique::SortByColors(Vertices &R) {
    ssize_t j = 0;
    ssize_t maxno = 1;
    ssize_t min_k = QMAX.size() - Q.size() + 1;
    C[1].rewind();
    C[2].rewind();
    auto k = 1;

    for(auto i = 0; i < R.size(); i++){
        ssize_t pi = R.at(i).get_i();
        k = 1;
        while(firstCut(pi, C[k])) k++;

        if(k > maxno){
            maxno = k;
            C[maxno + 1].rewind();
        }

        C[k].push(pi);

        if(k < min_k) R.at(j++).set_i(pi);
    }

    if(j > 0) R.at(j-1).set_degree(0);
    if(min_k <= 0) min_k = 1;

    for(k = min_k; k <= maxno; k++)
        for(auto i = 0; i < C[k].size(); i++){
            R.at(j).set_i(C[k].at(i));
            R.at(j++).set_degree(k);
        }
}

inline void FindMaxClique::expand(Vertices R){
    S[level].set_i1(S[level].get_i1() + S[level - 1].get_i1() - S[level].get_i2());
    S[level].set_i2(S[level - 1].get_i1());

    while(R.size()){
        if(Q.size() + R.end().get_degree() > QMAX.size()){
            Q.push(R.end().get_i());
            Vertices Rp(R.size());
            secondCut(R, Rp);
            if(Rp.size()){
                if((float)S[level].get_i1()/++pk < timeSearchLimit) sortByDegree(Rp);

                SortByColors(Rp);
                S[level].inc_i1();
                level++;
                expand(Rp);
                level--;
            }
            else if(Q.size() > QMAX.size()){
                std::cout << "step = " << pk << " current max. clique size = " << Q.size() << std::endl;
                QMAX = Q;
            }

            Rp.dispose();
            Q.pop();
        }
        else return;

        R.pop();
    }
}

#endif //SIMPLE_GRAPH_MAXCLIQUESPROBLEM_HPP
