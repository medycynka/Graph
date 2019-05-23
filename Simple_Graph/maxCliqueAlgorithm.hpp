#ifndef SIMPLE_GRAPH_MAXCLIQUEALGORITHM_HPP
#define SIMPLE_GRAPH_MAXCLIQUEALGORITHM_HPP

#pragma once
#include <algorithm>
#include <cassert>

template <typename T = bool>
class FindMaxClique{
    private:
        class Vertices{
            private:
                class Vertex{
                    public:
                        Vertex()                    : index_(-1), degree_(-1){};
                        explicit Vertex(ssize_t id) : index_(id), degree_(-1){};
                        ~Vertex() = default;

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
                Vertices()  = default;
                ~Vertices() = default;

                inline void    dispose()                  { vertices_.clear(); };
                inline void    sort()                     { std::sort(vertices_.begin(), vertices_.end(), desc_degree); };
                inline ssize_t size()               const { return vertices_.size(); };
                inline void    push(const ssize_t ii)     { vertices_.push_back(Vertex(ii)); };
                inline void    pop()                      { vertices_.pop_back(); };
                inline Vertex& at(const ssize_t ii)       { return vertices_[ii]; };
                inline Vertex& end()                      { return vertices_.back(); };
                inline void    init_colors();
                inline void    set_degrees(FindMaxClique&);

            private:
                std::vector<Vertex> vertices_;
        };

        class ColorVector{
            public:
                ColorVector()                          = default;
                explicit ColorVector(const ssize_t s_) { init(s_); };
                ~ColorVector() = default;

                inline void     init(const ssize_t csize_)    { colors_.resize(csize_); colors_.clear(); };
                inline void     push(const ssize_t ii)        { colors_.push_back(ii); };
                inline void     pop()                         { colors_.pop_back(); };
                inline void     rewind()                      { colors_.clear(); };
                inline ssize_t  size()                  const { return colors_.size(); };
                inline ssize_t& at(const ssize_t ii)          { return colors_[ii]; };
                ColorVector& operator=(const ColorVector& dh) { colors_ = dh.colors_; return *this; };

            private:
                std::vector<ssize_t> colors_;
        };

        class Counter{
            public:
                Counter() : i1(0), i2(0){};
                ~Counter() = default;

                inline void     set_i1(const ssize_t ii)                      { i1 = ii; };
                inline ssize_t  get_i1()                                const { return i1; };
                inline void     set_i2(const ssize_t ii)                      { i2 = ii; };
                inline ssize_t  get_i2()                                const { return i2; };
                inline void     inc_i1()                                      { i1++; };
                inline void     inc_i2()                                      { i2++; };
                inline void     setBoth(const ssize_t i1_, const ssize_t i2_) { i1 = i1_; i2 = i2_; };

            private:
                ssize_t i1;
                ssize_t i2;
        };

        inline bool isConnection(ssize_t i, ssize_t j) const { return (grid_.at(i).at(j) ? true : false); };
        inline bool firstCut(ssize_t, ColorVector&);
        inline void secondCut(Vertices&, Vertices&);
        inline void SortByColors(Vertices&);
        inline void expand(Vertices);
        inline void _maxCliqueAlgorithm(std::vector<ssize_t>&);
        inline void sortByDegree(Vertices &R) { R.set_degrees(*this); R.sort(); };

    public:
        FindMaxClique(std::vector<std::vector<std::optional<T>>>, ssize_t, float);
        ~FindMaxClique() = default;

        inline ssize_t steps()                                       const { return stepsCount; };
        inline void    maxCliqueAlgorithm(std::vector<ssize_t> &maxclique) { _maxCliqueAlgorithm(maxclique); };

    private:
        Vertices V;
        std::vector<ColorVector> C;
        ColorVector QMAX;
        ColorVector Q;
        std::vector<std::vector<std::optional<T>>> grid_;
        ssize_t stepsCount;
        ssize_t level;
        const float timeSearchLimit;
        std::vector<Counter> S;
};

template <typename T>
FindMaxClique<T>::FindMaxClique(std::vector<std::vector<std::optional<T>>> conn, const ssize_t size_, const float tt) : stepsCount(0), level(1), timeSearchLimit(tt), Q(size_), QMAX(size_){
    for(auto i = 0; i < size_; i++) V.push(i);

    grid_ = conn;
    C.resize(size_ + 1, ColorVector());

    for(auto i=0; i < size_ + 1; i++) C[i].init(size_ + 1);

    S.resize(size_ + 1, Counter());
}

template <typename T>
inline void FindMaxClique<T>::_maxCliqueAlgorithm(std::vector<ssize_t> &maxclique){
    V.set_degrees(*this);
    V.sort();
    V.init_colors();

    for(auto i = 0; i < V.size() + 1; i++) S[i].setBoth(0, 0);

    expand(V);
    maxclique.resize(QMAX.size());

    for(auto i=0; i < QMAX.size(); i++) maxclique[i] = QMAX.at(i);
}

template <typename T>
inline void FindMaxClique<T>::Vertices::init_colors() {
    const ssize_t max_degree = vertices_[0].get_degree();

    for(auto i = 0; i < max_degree; i++) vertices_[i].set_degree(i + 1);

    for(auto i = max_degree; i < vertices_.size(); i++) vertices_[i].set_degree(max_degree + 1);
}

template <typename T>
inline void FindMaxClique<T>::Vertices::set_degrees(FindMaxClique &m) {
    auto psize_ = vertices_.size();
    for(auto i = 0; i < psize_; i++){
        auto d = 0;

        for(auto j = 0; j < psize_; j++) if(m.isConnection(vertices_[i].get_i(), vertices_[j].get_i())) d++;

        vertices_[i].set_degree(d);
    }
}

template <typename T>
inline bool FindMaxClique<T>::firstCut(ssize_t pi, ColorVector &A) {
    for(auto i = 0; i < A.size(); i++)  if(isConnection(pi, A.at(i))) return true;

    return false;
}

template <typename T>
inline void FindMaxClique<T>::secondCut(Vertices &A, Vertices &B) {
    for(auto i = 0; i < A.size() - 1; i++) if(isConnection(A.end().get_i(), A.at(i).get_i())) B.push(A.at(i).get_i());
}

template <typename T>
inline void FindMaxClique<T>::SortByColors(Vertices &R) {
    ssize_t j = 0;
    ssize_t maxno = 1;
    ssize_t min_k = QMAX.size() - Q.size() + 1;
    C[1].rewind();
    C[2].rewind();
    size_t k;

    for(auto i = 0; i < R.size(); i++){
        auto pi = R.at(i).get_i();
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

    for(k = min_k; k <= maxno; k++){
        for(auto i = 0; i < C[k].size(); i++){
            R.at(j).set_i(C[k].at(i));
            R.at(j++).set_degree(k);
        }
    }
}

template <typename T>
inline void FindMaxClique<T>::expand(Vertices R){
    S[level].setBoth(S[level].get_i1() + S[level - 1].get_i1() - S[level].get_i2(), S[level - 1].get_i1());

    while(R.size()){
        if(Q.size() + R.end().get_degree() > QMAX.size()){
            Q.push(R.end().get_i());
            Vertices Rp;
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

#endif //SIMPLE_GRAPH_MAXCLIQUEALGORITHM_HPP
