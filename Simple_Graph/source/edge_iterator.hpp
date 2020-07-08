#ifndef GRAPH_EDGE_ITERATOR_HPP
#define GRAPH_EDGE_ITERATOR_HPP

#pragma once

#include <iterator>
#include <vector>
#include <optional>

namespace ads::ds::graph {

	namespace iterators {

        template <typename E>
        class EdgesIterator {
        protected:
            size_t                                     curr_row;
            size_t                                     curr_col;
            std::vector<std::vector<std::optional<E>>> nm;

        public:
            typedef EdgesIterator             self_type;
            typedef E                         value_type;
            typedef E&                        reference;
            typedef E*                        pointer;
            typedef std::forward_iterator_tag iterator_category;
            typedef int                       difference_type;

            explicit EdgesIterator(std::vector<std::vector<std::optional<E>>>& graph);
            EdgesIterator(std::vector<std::vector<std::optional<E>>>& graph, std::size_t nm_row, std::size_t nm_col);
            EdgesIterator(const EdgesIterator& s)     : curr_row{ s.curr_row }, curr_col{ s.curr_col }, nm{ s.nm } {};
            EdgesIterator(EdgesIterator&& s) noexcept :curr_row{ s.curr_row }, curr_col{ s.curr_col }, nm{ s.nm } {};

            bool                            operator==(const EdgesIterator& ei) const { return (curr_row == ei.curr_row && curr_col == ei.curr_col && nm == ei.nm); };
            bool                            operator!=(const EdgesIterator& ei) const { return !(*this == ei); };
            EdgesIterator&                  operator++();
            EdgesIterator const             operator++(int);
            E&                              operator* ()                              { return nm.at(curr_row).at(curr_col).value(); };
            E*                              operator->()                        const { return *(nm.at(curr_row).at(curr_col).value()); };
            explicit                        operator bool()                     const { return curr_row != nm.size(); };

            std::pair<size_t, size_t>               id()         { return std::make_pair(curr_row, curr_col); };
            [[nodiscard]] std::pair<size_t, size_t> id()   const { return std::make_pair(curr_row, curr_col); };
            [[nodiscard]] size_t                    v1id() const { return curr_row; };
            [[nodiscard]] size_t                    v2id() const { return curr_col; };
        };

        template<typename E>
        EdgesIterator<E>::EdgesIterator(std::vector<std::vector<std::optional<E>>>& graph) : curr_row{ 0 }, curr_col{ 0 }, nm{ graph } {
            bool checker = (nm.at(curr_row).at(curr_col) ? true : false);

            while (!checker) {
                curr_col++;

                if (curr_col == nm.size()) {
                    curr_row++;
                    curr_col = 0;
                }

                checker = (curr_row == nm.size() ? true : (nm.at(curr_row).at(curr_col) ? true : false));
            }
        }

        template<typename E>
        EdgesIterator<E>::EdgesIterator(std::vector<std::vector<std::optional<E>>>& graph, std::size_t nm_row, std::size_t nm_col) : curr_row{ nm_row }, curr_col{ nm_col }, nm{ graph } {
            if (curr_row != graph.size() && curr_col != graph.size()) {
                bool checker = (nm.at(curr_row).at(curr_col) ? true : false);

                while (!checker) {
                    curr_col++;

                    if (curr_col == nm.size()) {
                        curr_row++;
                        curr_col = 0;
                    }

                    checker = (curr_row == nm.size() ? true : (nm.at(curr_row).at(curr_col) ? true : false));
                }
            }
        }

        template<typename E>
        EdgesIterator<E>& EdgesIterator<E>::operator++() {
            bool checker = false;

            while (!checker) {
                curr_col++;

                if (curr_col == nm.size()) {
                    curr_row++;
                    curr_col = 0;
                }

                checker = (curr_row == nm.size() ? true : (nm.at(curr_row).at(curr_col) ? true : false));
            }

            return *this;
        }

        template<typename E>
        EdgesIterator<E> const EdgesIterator<E>::operator++(int) {
            auto pom = *this;
            ++(*this);

            return pom;
        }

	}

}

#endif
