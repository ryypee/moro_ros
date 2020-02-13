#!/usr/bin/env python
import numpy as np
from itertools import permutations


def time_expand(graph, nodes, obstruction):
    """Create a time-expanded graph taking into account the given
    dynamic obstruction.

    Args:
        graph (ndarray): The original graph adjacency matrix, size (n,n)
        nodes (ndarray): Node coordinates in the graph, size (n,2)
        obstruction (array-like): Indeces of obstructed nodes, length t

    Returns:
        tuple: The time-expanded graph, size (tn,tn), and node
            coordinates in the new graph, size(tn,2)
    """
    # TODO
    expanded_graph = nodes = None

    return expanded_graph, nodes


def joint_graph(graph, nodes):
    """Create a joint graph for two agents based on the given single
    agent graph

    Args:
        graph (ndarray): The single agent graph adjacency matrix, size (n,n)
        nodes (ndarray): Node coordinates in the graph, size (n,2)

    Returns:
        tuple: The joint graph, size (n^2-n,n^2-n), and node coordinates in the
            joint graph, size (n^2-n,2,2), where the second axis is the two
            agents and the third axis is coordinates
    """
    # TODO
    joint_graph = nodes = None

    return joint_graph, nodes
