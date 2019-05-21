#ifndef SIMPLE_GRAPH_MAXCLIQUESPROBLEM_HPP
#define SIMPLE_GRAPH_MAXCLIQUESPROBLEM_HPP

#pragma once
#include <algorithm>
#include <cassert>

template <typename T = bool>
class FindMaxClique{
        class Vertices{
                class Vertex{
                    public:
                        inline void    set_i(const ssize_t ii) { index_ = ii; };
                        inline ssize_t get_i()           const { return index_; };
                        inline void    set_degree(ssize_t dd)  { degree_ = dd; };
                        inline ssize_t get_degree()      const { return degree_; };

                    private:
                        ssize_t index_;
                        ssize_t degree_;
                };

                inline static bool desc_degree(const Vertex vi, const Vertex vj) { return (vi.get_degree() > vj.get_degree()); };

            public:
                Vertices()                    : sizeV_(0), vertices_(nullptr){};
                explicit Vertices(ssize_t s_) : sizeV_(0), vertices_(new Vertex[s_]){};
                ~Vertices() = default;

                inline void    dispose()                  { delete [] vertices_; };
                inline void    sort()                     { std::sort(vertices_, vertices_+sizeV_, desc_degree); };
                inline ssize_t size()               const { return sizeV_; };
                inline void    push(const ssize_t ii)     { vertices_[sizeV_++].set_i(ii); };
                inline void    pop()                      { sizeV_--; };
                inline Vertex& at(const ssize_t ii) const { return vertices_[ii]; };
                inline Vertex& end()                const { return vertices_[sizeV_ - 1]; };
                inline void    init_colors();
                inline void    set_degrees(FindMaxClique&);

            private:
                Vertex *vertices_;
                ssize_t sizeV_;
        };

        class ColorVector{
            public:
                ColorVector()                             : sizeC_(0), colors_(nullptr){};
                explicit ColorVector(const ssize_t s_) : sizeC_(s_), colors_(nullptr){ init(s_); };
                ~ColorVector(){ delete [] colors_; };

                inline void     init(const ssize_t csize_)    { colors_ = new ssize_t[csize_]; rewind(); };
                inline void     push(const ssize_t ii)        { colors_[sizeC_++] = ii; };
                inline void     pop()                         { sizeC_--; };
                inline void     rewind()                      { sizeC_ = 0; };
                inline ssize_t  size()                  const { return sizeC_; };
                inline ssize_t& at(const ssize_t ii)    const { return colors_[ii]; };
                ColorVector& operator=(const ColorVector& dh) { for(auto j = 0; j < dh.sizeC_; j++){ colors_[j] = dh.colors_[j]; } sizeC_ = dh.sizeC_; return *this; };
                inline void     clearContent()                { delete [] colors_; };

            private:
                ssize_t *colors_;
                ssize_t sizeC_;
        };

        class Counter{
            public:
                Counter() : i1(0), i2(0){};
                ~Counter() = default;

                inline void     set_i1(const ssize_t ii) { i1 = ii; };
                inline ssize_t  get_i1()           const { return i1; };
                inline void     set_i2(const ssize_t ii) { i2 = ii; };
                inline ssize_t  get_i2()           const { return i2; };
                inline void     inc_i1()                 { i1++; };

            private:
                ssize_t i1;
                ssize_t i2;
        };

        inline bool isConnection(const ssize_t i, const ssize_t j) const { return grid[i][j]; };
        inline bool firstCut(ssize_t, const ColorVector&);
        inline void secondCut(const Vertices&, Vertices&);
        inline void SortByColors(Vertices&);
        inline void expand(Vertices);
        inline void _maxCliqueAlgorithm(ssize_t*&, ssize_t&);
        inline void sortByDegree(Vertices &R) { R.set_degrees(*this); R.sort(); };

    public:
        FindMaxClique(const T* const*, ssize_t, float);
        ~FindMaxClique(){ delete [] C; delete [] S; V.dispose(); };

        inline ssize_t steps() const { return stepsCount; };
        inline void    maxCliqueAlgorithm(ssize_t* &maxclique, ssize_t &size_) { _maxCliqueAlgorithm(maxclique, size_); };
        inline void    clearResources();

    private:
        Vertices V;
        ColorVector *C;
        ColorVector QMAX;
        ColorVector Q;
        const T* const* grid;
        ssize_t stepsCount;
        ssize_t level;
        const size_t nrOfVertices;
        const float timeSearchLimit;
        Counter *S;
};

template <typename T>
FindMaxClique<T>::FindMaxClique(const T* const* conn, const ssize_t size_, const float tt) : stepsCount(0), level(1), timeSearchLimit(tt), V(size_), Q(size_), QMAX(size_), nrOfVertices(size_){
    assert(conn != nullptr && size_>0);
    for(auto i = 0; i < size_; i++) V.push(i);

    grid = conn;
    C = new ColorVector[size_ + 1];

    for(auto i=0; i < size_ + 1; i++) C[i].init(size_ + 1);

    S = new Counter[size_ + 1];
}

template <typename T>
inline void FindMaxClique<T>::_maxCliqueAlgorithm(ssize_t* &maxclique, ssize_t &size_) {
    V.set_degrees(*this);
    V.sort();
    V.init_colors();

    for(auto i = 0; i < V.size() + 1; i++){
        S[i].set_i1(0);
        S[i].set_i2(0);
    }

    expand(V);
    maxclique = new ssize_t[QMAX.size()];

    for(auto i=0; i < QMAX.size(); i++) maxclique[i] = QMAX.at(i);

    size_ = QMAX.size();
}

template <typename T>
inline void FindMaxClique<T>::Vertices::init_colors() {
    const ssize_t max_degree = vertices_[0].get_degree();

    for(auto i = 0; i < max_degree; i++) vertices_[i].set_degree(i + 1);

    for(auto i = max_degree; i < sizeV_; i++) vertices_[i].set_degree(max_degree + 1);
}

template <typename T>
inline void FindMaxClique<T>::Vertices::set_degrees(FindMaxClique &m) {
    for(auto i = 0; i < sizeV_; i++){
        auto d = 0;

        for(auto j = 0; j < sizeV_; j++) if(m.isConnection(vertices_[i].get_i(), vertices_[j].get_i())) d++;

        vertices_[i].set_degree(d);
    }
}

template <typename T>
inline bool FindMaxClique<T>::firstCut(const ssize_t pi, const ColorVector &A) {
    for(auto i = 0; i < A.size(); i++)  if(isConnection(pi, A.at(i))) return true;

    return false;
}

template <typename T>
inline void FindMaxClique<T>::secondCut(const Vertices &A, Vertices &B) {
    for(auto i = 0; i < A.size() - 1; i++) if(isConnection(A.end().get_i(), A.at(i).get_i())) B.push(A.at(i).get_i());
}

template <typename T>
inline void FindMaxClique<T>::SortByColors(Vertices &R) {
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

template <typename T>
inline void FindMaxClique<T>::expand(Vertices R){
    S[level].set_i1(S[level].get_i1() + S[level - 1].get_i1() - S[level].get_i2());
    S[level].set_i2(S[level - 1].get_i1());

    while(R.size()){
        if(Q.size() + R.end().get_degree() > QMAX.size()){
            Q.push(R.end().get_i());
            Vertices Rp(R.size());
            secondCut(R, Rp);
            if(Rp.size()){
                if((float)S[level].get_i1()/++stepsCount < timeSearchLimit) sortByDegree(Rp);

                SortByColors(Rp);
                S[level].inc_i1();
                level++;
                expand(Rp);
                level--;
            }
            else if(Q.size() > QMAX.size()){
                //std::cout << "step = " << stepsCount << " current max. clique size = " << Q.size() << std::endl;
                QMAX = Q;
            }

            Rp.dispose();
            Q.pop();
        }
        else return;

        R.pop();
    }
}

template <typename T>
void FindMaxClique<T>::clearResources(){
    delete [] C;
    delete [] S;
    V.dispose();
    QMAX.clearContent();
    Q.clearContent();
}

#endif //SIMPLE_GRAPH_MAXCLIQUESPROBLEM_HPP
