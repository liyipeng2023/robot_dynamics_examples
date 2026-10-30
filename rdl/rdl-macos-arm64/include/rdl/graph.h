// Copyright (C) 2025 Yipeng Li (yipengli.bice@gmail.com)
//
// This file is part of robot dynamics library.

#ifndef GRAPH_H
#define GRAPH_H

#include <dll_export.h>
#include <cstddef>
//#include <memory>

namespace RDL{
    using namespace std;
    // to represent the topology of a robot
    class Graph{
    public:
        virtual ~Graph() = default;
        /**
         * @brief Add an edge between two nodes.
         *
         * @param[in] i_ Parent node index.
         * @param[in] j_ Child node index.
         */
        virtual void add_edge(size_t i_, size_t j_) = 0;

        /**
         * @brief Initialize the graph with the specified number of nodes.
         *
         * Allocates internal data structures required for graph processing.
         *
         * @param[in] n_ Number of nodes in the graph.
         */
        virtual void initialize(size_t n_) = 0;

        /**
         * @brief Get the number of adjacent nodes.
         *
         * @param[in] index_ Node index.
         * @return Number of adjacent nodes.
         */
        virtual size_t get_adjacency_list_num(size_t index_) = 0;

        /**
         * @brief Get the adjacency list of a node.
         *
         * @param[in] index_ Node index.
         * @param[out] ad_list_ Output buffer for adjacent node indices.
         */
        virtual void get_adjacency_list(size_t index_, size_t* ad_list_) = 0;

        /**
         * @brief Get the number of nodes in the kappa set.
         *
         * @param[in] index_ Node index.
         * @return Number of elements in the kappa set.
         */
        virtual size_t get_kappa_num(size_t index_) = 0;

        /**
         * @brief Get the kappa set of a node.
         *
         * @param[in] index_ Node index.
         * @param[out] kappa_ Output buffer for the kappa set.
         */
        virtual void get_kappa(size_t index_, size_t* kappa_) = 0;

        /**
         * @brief Get the number of nodes in the mu set.
         *
         * @param[in] index_ Node index.
         * @return Number of elements in the mu set.
         */
        virtual size_t get_mu_num(size_t index_) = 0;

        /**
         * @brief Get the mu set of a node.
         *
         * @param[in] index_ Node index.
         * @param[out] mu_ Output buffer for the mu set.
         */
        virtual void get_mu(size_t index_, size_t* mu_) = 0;

        /**
         * @brief Get the parent node of a node.
         *
         * @param[in] index_ Node index.
         * @return Parent node index (lambda).
         */
        virtual size_t get_lambda(size_t index_) = 0;

        /**
         * @brief Get the number of nodes in the nu set.
         *
         * @param[in] index_ Node index.
         * @return Number of elements in the nu set.
         */
        virtual size_t get_nu_num(size_t index_) = 0;

        /**
         * @brief Get the nu set of a node.
         *
         * @param[in] index_ Node index.
         * @param[out] nu_ Output buffer for the nu set.
         */
        virtual void get_nu(size_t index_, size_t* nu_) = 0;

        /**
         * @brief Get the number of terminal nodes.
         *
         * @return Number of terminal (leaf) nodes.
         */
        virtual size_t get_ends_num() = 0;

        /**
         * @brief Get all terminal nodes.
         *
         * @param[out] end_ Output buffer for terminal node indices.
         */
        virtual void get_ends(size_t* end_) = 0;

        /**
         * @brief Get the number of nodes in the graph.
         *
         * @return Total number of nodes.
         */
        virtual size_t num() = 0;

        /**
         * @brief Remove all nodes and edges from the graph.
         */
        virtual void clear() = 0;

        /**
         * @brief Print the graph topology.
         */
        virtual void print() = 0;
    };
    //RDL_EXPORT shared_ptr<Graph> createGraph(size_t n);
    extern "C" {
        RDL_EXPORT Graph* createGraph(size_t n);
        RDL_EXPORT void deleteGraph(Graph* ptr);
    }
}
#endif