#ifndef GRAPH_BFS_ITERATOR_HPP
#define GRAPH_BFS_ITERATOR_HPP

#pragma once

#include <iterator>
#include <vector>
#include <queue>

namespace ads::ds::graph {

	namespace iterators {

		template <typename V, typename E>
        class bfsIterator {
        protected:
            std::vector<V>                             v;
            std::vector<std::vector<std::optional<E>>> nm;
            size_t                                     current;
            std::vector<bool>                          visited;
            std::queue<size_t>                         s;
            size_t                                     count;

        public:
            typedef bfsIterator               self_type;
            typedef V                         value_type;
            typedef V&                        reference;
            typedef V*                        pointer;
            typedef std::forward_iterator_tag iterator_category;
            typedef int                       difference_type;

            bfsIterator(const bfsIterator& other)                                                                             : v{ other.v }, nm { other.nm }, current{ other.current }, visited{ other.visited }, s{ other.s }, count{ other.count }, { s.v } {};
            bfsIterator(bfsIterator&& other) noexcept                                                                         : v{ other.v }, nm { other.nm }, current{ other.current }, visited{ other.visited }, s{ other.s }, count{ other.count }, { s.v } {};
            bfsIterator(std::vector<V>& ver_, std::vector<std::vector<std::optional<E>>>& graph, std::size_t starting_vertex) : v{ ver_ }, nm { graph }, current{ starting_vertex }, count{ 1 } { visited.resize(ver_.size(), false); };

            bool              operator==(const bfsIterator& vi) const { return (current == vi.current && v == vi.v && ref == vi.ref); };
            bool              operator!=(const bfsIterator& vi) const { return !(*this == vi); };
            bfsIterator&      operator++();
            bfsIterator const operator++(int);
            V&                operator* ()                            { return v.at(current); };
            V*                operator->()                      const { return *(v.at(current)); };
            explicit          operator bool()                   const { return count != v.size(); };

            size_t               id()       { return current; };
            [[nodiscard]] size_t id() const { return current; };
        };

        template<typename V, typename E>
        bfsIterator<V, E>& bfsIterator<V, E>::operator++() {
            visited.at(current) = true;

            if (count < v.size()) {
                for (auto i = 0; i < nm.at(current).size(); i++) if (nm.at(current).at(i)) s.push(i);

                if (!s.empty()) {
                    while (true) {
                        if (!s.empty() && !visited.at(s.front())) {
                            count++;
                            current = s.front();
                            break;
                        }

                        if (!s.empty()) {
                            s.pop();
                        }
                        else {
                            count = v.size();
                            current = v.size();
                            break;
                        }
                    }
                }
                else {
                    count = v.size();
                    current = v.size();
                }
            }
            else {
                count = v.size();
                current = v.size();
            }

            return *this;
        }

        template<typename V, typename E>
        bfsIterator<V, E> const bfsIterator<V, E>::operator++(int) {
            auto pom = *this;
            ++(*this);

            return *this;
        }

	}

}

#endif // !GRAPH_BFS_ITERATOR_HPP
