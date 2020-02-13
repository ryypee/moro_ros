#!/usr/bin/env python
import os
import sys
import unittest
import yaml
import rospkg
import matplotlib.pyplot as plt
import numpy as np
from moro_navigation import (
    ProbabilisticRoadmap,
    dijkstra,
    astar,
    dynamic_programming,
    smooth_path,
    time_expand,
    joint_graph
)
from moro_navigation.utils import DotDict

PKG = 'moro_navigation'


class TestProbabilisticRoadmap(unittest.TestCase):
    """Test probabilistic roadmap"""
    def setUp(self):
        rospack = rospkg.RosPack()
        filepath = rospack.get_path('moro_simulator') + '/world/maze_map.yaml'
        with open(filepath) as f:
            msg = DotDict(yaml.safe_load(f))
        self.prm = ProbabilisticRoadmap(msg)

    def test_figure_coordinates(self):
        index = self.prm._figure_coordinates(self.prm._origin)
        self.assertEqual([0, 0], index.tolist())

    def test_symmetric(self):
        np.testing.assert_allclose(self.prm.graph, self.prm.graph.T)

    def test_plot_map(self):
        test_dir = rospkg.get_test_results_dir() + '/' + PKG
        if not os.path.exists(test_dir):
            os.makedirs(test_dir)
        filepath = test_dir + '/prm.png'
        plt.figure()
        self.prm.plot()
        plt.savefig(filepath)
        self.assertTrue(True)


class TestShortestPath(unittest.TestCase):
    """Test shortest path algorithms"""
    def setUp(self):
        graph = 0.05 * np.eye(7)
        graph[0, 1] = 3
        graph[0, 2] = 2
        graph[1, 3] = 1
        graph[2, 3] = 3
        graph[3, 4] = 3
        graph[3, 5] = 1
        graph[4, 6] = 2
        self.graph = graph + graph.T

    def test_dijkstra(self):
        sp = [0, 1, 3, 4, 6]
        path = dijkstra(self.graph, 0, 6)
        self.assertEqual(sp, list(path))

    def test_astar(self):
        sp = [0, 1, 3, 4, 6]
        heuristic = [0., 3., 2., 3.5, 2., 3., 4.]
        path = astar(self.graph, 0, 6, heuristic)
        self.assertEqual(sp, list(path))

    def test_dynamic_programming(self):
        sp = [0, 1, 3, 4, 6]
        path = dynamic_programming(self.graph, 0, 6)
        self.assertEqual(sp, list(path))


class TestPathSmoothing(unittest.TestCase):
    """Test path smoothing algorithm"""
    def test_path_smoothing(self):
        test_dir = rospkg.get_test_results_dir() + '/' + PKG
        if not os.path.exists(test_dir):
            os.makedirs(test_dir)

        path = np.array([[0, 0],
                         [1, 0],
                         [1, 1],
                         [2, 1]])
        pos, vel, acc, jerk, time = smooth_path(path)
        plt.figure()
        ax1 = plt.subplot2grid((3, 2), (0, 0), rowspan=3)
        ax1.plot(pos[:, 0], pos[:, 1])
        ax1.set_xlabel('x position')
        ax1.set_ylabel('y position')
        ax2 = plt.subplot2grid((3, 2), (0, 1))
        ax2.plot(time, vel[:, 0], time, vel[:, 1])
        ax2.set_ylabel('Velocity')
        ax3 = plt.subplot2grid((3, 2), (1, 1), sharex=ax2)
        ax3.plot(time, acc[:, 0], time, acc[:, 1])
        ax3.set_ylabel('Acceleration')
        ax4 = plt.subplot2grid((3, 2), (2, 1), sharex=ax2)
        ax4.plot(time, jerk[:, 0], time, jerk[:, 1])
        ax4.set_ylabel('Jerk')
        ax4.set_xlabel('Time')
        for ax in [ax2, ax3]:
            plt.setp(ax.get_xticklabels(), visible=False)
        plt.tight_layout()
        filepath = test_dir + '/smoothing.png'
        plt.savefig(filepath)
        self.assertTrue(True)


class TestMultiagentPlanning(unittest.TestCase):
    """Test multiagent planning algorithms"""
    def setUp(self):
        graph = 0.05 * np.eye(7)
        graph[0, 1] = 3
        graph[0, 2] = 2
        graph[1, 3] = 1
        graph[2, 3] = 3
        graph[3, 4] = 3
        graph[3, 5] = 1
        graph[4, 6] = 2
        graph[0, 0] = 0.01
        self.graph = graph + graph.T
        self.nodes = np.array([[0., 0.],
                               [1., 1.],
                               [1., -1.],
                               [2., 0.],
                               [3., 1.],
                               [3., -1.],
                               [4., 0.]])
        self.num_nodes = 7

    def test_time_expansion(self):
        obstruction = [6, 4, 3, 1, 0]
        sp = [0, 0, 2, 3, 4, 6]
        graph, nodes = time_expand(self.graph, self.nodes, obstruction)
        path = dijkstra(graph, 0, nodes.shape[0] - 1)
        self.assertEqual(sp, [node % self.num_nodes for node in path])

    def test_joint_planning(self):
        def joint_id(row, col):
            return None if row == col else (
                row*(self.num_nodes - 1) + (col - (col > row)))

        def indiv_id(id):
            row = id/(self.num_nodes - 1)
            col = id % (self.num_nodes - 1)
            return [row, col + (col >= row)]

        sp = [[0, 6], [0, 4], [2, 3], [3, 1], [4, 0], [6, 0]]

        graph, nodes = joint_graph(self.graph, self.nodes)
        path = dijkstra(graph, joint_id(0, 6), joint_id(6, 0))
        self.assertEqual(sp, [indiv_id(p) for p in path])


class NavigationTestSuite(unittest.TestSuite):
    """Test suite to run all the tests"""
    def __init__(self):
        super(NavigationTestSuite, self).__init__()
        self.addTest(unittest.makeSuite(TestProbabilisticRoadmap))
        self.addTest(unittest.makeSuite(TestShortestPath))
        self.addTest(unittest.makeSuite(TestPathSmoothing))
        self.addTest(unittest.makeSuite(TestMultiagentPlanning))


if __name__ == '__main__':
    import rosunit
    rosunit.unitrun(PKG, 'test_navigation',
                    'moro_navigation_test.NavigationTestSuite', sys.argv)
