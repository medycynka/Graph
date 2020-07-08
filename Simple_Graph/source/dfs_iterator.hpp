#ifndef GRAPH_DFS_ITERATOR_HPP
#define GRAPH_DFS_ITERATOR_HPP

#pragma once

#include <iterator>
#include <vector>
#include <stack>

namespace ads::ds::graph {

	namespace iterators {

        template <typename V, typename E>
        class dfsIterator {
        protected:
            std::vector<V>                             v;
            std::vector<std::vector<std::optional<E>>> nm;
            size_t                                     current;
            std::vector<bool>                          visited;
            std::stack<size_t>                         s;
            size_t                                     count;

        public:
            typedef dfsIterator               self_type;
            typedef V                         value_type;
            typedef V&                        reference;
            typedef V*                        pointer;
            typedef std::forward_iterator_tag iterator_category;
            typedef int                       difference_type;

            dfsIterator(const dfsIterator& other)                                                                             : v{ other.v }, nm{ other.nm }, current{ other.current }, visited{ other.visited }, s{ other.s }, count{ other.count } {};
            dfsIterator(dfsIterator&& other) noexcept                                                                         : v{ other.v }, nm{ other.nm }, current{ other.current }, visited{ other.visited }, s{ other.s }, count{ other.count } {};
            dfsIterator(std::vector<V>& ver_, std::vector<std::vector<std::optional<E>>>& graph, std::size_t starting_vertex) : v{ ver_ }, nm{ graph }, current{ starting_vertex }, count{ 1 } { visited.resize(ver_.size(), false); };

            bool              operator==(const dfsIterator& vi) const { return (current == vi.current && v == vi.v && nm == vi.nm); };
            bool              operator!=(const dfsIterator& vi) const { return !(*this == vi); };
            dfsIterator&      operator++();
            dfsIterator const operator++(int);
            V&                operator* ()                            { return v.at(current); };
            V*                operator->()                      const { return *(v.at(current)); };
            explicit          operator bool()                   const { return count != v.size(); };

            size_t               id()       { return current; };
            [[nodiscard]] size_t id() const { return current; };
        };

        template<typename V, typename E>
        dfsIterator<V, E>& dfsIterator<V, E>::operator++() {
            visited.at(current) = true;

            if (count < v.size()) {
                for (auto i = nm.at(current).size() - 1; i != -1; i--) if (nm.at(current).at(i)) s.push(i);

                if (!s.empty()) {
                    while (true) {
                        if (!s.empty() && !visited.at(s.top())) {
                            count++;
                            current = s.top();
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
        dfsIterator<V, E> const dfsIterator<V, E>::operator++(int) {
            auto pom = *this;
            ++(*this);

            return *this;
        }

	}

}

#endif
