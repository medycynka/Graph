#ifndef GRAPH_VERTICE_ITERATOR_HPP
#define GRAPH_VERTICE_ITERATOR_HPP

#pragma once

#include <iterator>
#include <vector>

namespace ads::ds::graph {

	namespace iterators {

        template <typename V>
        class VerticesIterator {
        protected:
            std::vector<V> v;
            size_t         curr_It;

        public:
            typedef VerticesIterator          self_type;
            typedef V                         value_type;
            typedef V&                        reference;
            typedef V*                        pointer;
            typedef std::forward_iterator_tag iterator_category;
            typedef int                       difference_type;

            VerticesIterator(const VerticesIterator& s)                           : v{ s.v }, curr_It{ s.curr_It } {};
            VerticesIterator(VerticesIterator&& s) noexcept                       : v{ s.v }, curr_It{ s.curr_It } {};
            VerticesIterator(std::vector<V>& ver_, std::size_t current_vertex_id) : v{ ver_ }, curr_It{ current_vertex_id } {};

            bool                   operator==(const VerticesIterator& vi) const { return (curr_It == vi.curr_It && v == vi.v); };
            bool                   operator!=(const VerticesIterator& vi) const { return !(*this == vi); };
            VerticesIterator&      operator++();
            VerticesIterator const operator++(int);
            VerticesIterator&      operator--();
            VerticesIterator const operator--(int);
            VerticesIterator&      operator+=(size_t count);
            VerticesIterator&      operator-=(size_t count);
            V&                     operator* ()                                 { return v.at(curr_It); };
            V*                     operator->()                           const { return *(v.at(curr_It)); };
            explicit               operator bool()                        const { return curr_It != ref.nrOfVertices(); };

            size_t                 id()       { return curr_It; };
            [[nodiscard]] size_t   id() const { return curr_It; };
        };

        template<typename V>
        VerticesIterator<V>& VerticesIterator<V>::operator++() {
            curr_It++;

            return *this;
        }

        template<typename V>
        VerticesIterator<V> const VerticesIterator<V>::operator++(int) {
            auto pom = *this;
            ++curr_It;

            return *this;
        }

        template<typename V>
        VerticesIterator<V>& VerticesIterator<V>::operator--() {
            curr_It--;

            return *this;
        }

        template<typename V>
        VerticesIterator<V> const VerticesIterator<V>::operator--(int) {
            auto pom = *this;
            --curr_It;

            return *this;
        }

        template<typename V>
        VerticesIterator<V>& VerticesIterator<V>::operator+=(size_t count) {
            curr_It += count;

            return *this;
        }

        template<typename V>
        VerticesIterator<V>& VerticesIterator<V>::operator-=(size_t count) {
            curr_It -= count;

            return *this;
        }

	}

}

#endif
