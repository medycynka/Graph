#include "source/graph.hpp"
#define BOOST_TEST_MODULE Graph_Test

#include <boost/test/included/unit_test.hpp>

BOOST_AUTO_TEST_SUITE(general_test)
BOOST_AUTO_TEST_CASE(constructor_tests){
    Graph<int, int> g1;
    g1.insertVertex(0);
    g1.insertVertex(1);
    g1.insertVertex(2);
    g1.insertEdge(0, 1, 1);
    g1.insertEdge(1, 2, 2);
    g1.insertEdge(2, 0, 3);
    g1.insertEdge(2, 1, 4);

    BOOST_CHECK_EQUAL(g1.nrOfVertices(), 3);
    BOOST_CHECK_EQUAL(g1.nrOfEdges(), 4);

    Graph<int, int> g2 = g1;
    BOOST_CHECK_EQUAL(g2.nrOfEdges(), g1.nrOfEdges());
    BOOST_CHECK_EQUAL(g2 == g1, true);
    g2.removeEdge(2, 1);
    BOOST_CHECK_EQUAL(g2 == g1, false);

    Graph<int, int> g3;
    g3 = g1;
    BOOST_CHECK_EQUAL(g3 != g2, true);
    BOOST_CHECK_EQUAL(g3 == g1, true);

    Graph<char, int> g4 = {'a', 'b', 'c', 'd', 'e'};
    BOOST_CHECK_EQUAL(g4.nrOfVertices(), 5);
    BOOST_CHECK_EQUAL(g4.nrOfEdges(), 0);

    Graph<char, int> g5 = std::pair<std::initializer_list<char>, std::initializer_list<std::pair<std::pair<size_t, size_t>, int>>>({{'a', 'b', 'c'}, {{{0, 1}, 1}, {{0, 2}, 4}, {{1, 2}, 1}, {{2, 1}, 5}}});
    BOOST_CHECK_EQUAL(g5.nrOfVertices(), 3);
    BOOST_CHECK_EQUAL(g5.nrOfEdges(), 4);
}

BOOST_AUTO_TEST_CASE(insert_remove_test){
    Graph<int, int> g;
    std::vector<int> correct = {0, 1, 2, 3, 4};
    g.insertVertex(0);
    g.insertVertex(1);
    g.insertVertex(2);
    g += 3;
    g += 4;
    BOOST_CHECK_EQUAL(g.nrOfVertices(), 5);
    BOOST_CHECK_EQUAL(g.getVertices() == correct, true);

    g.insertEdge(0, 0, 1);
    g.insertEdge(0, 1, 1);
    g += {{0, 3}, 1};
    g += {{1, 2}, 1};
    g += {{1, 3}, 1};
    g += {{1, 4}, 1};
    g += {{2, 0}, 1};
    g += {{3, 4}, 1};
    g += {{4, 0}, 1};
    g += {{4, 1}, 1};
    g += {{4, 2}, 1};
    g += {{4, 3}, 1};
    BOOST_CHECK_EQUAL(g.nrOfEdges(), 12);

    g.removeVertex(4);
    BOOST_CHECK_EQUAL(g.nrOfVertices(), 4);
    BOOST_CHECK_EQUAL(g.nrOfEdges(), 6);
    g -= {1, 3};
    BOOST_CHECK_EQUAL(g.nrOfEdges(), 5);
}
BOOST_AUTO_TEST_SUITE_END()