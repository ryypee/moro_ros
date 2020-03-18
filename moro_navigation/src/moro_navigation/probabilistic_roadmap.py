#!/usr/bin/env python
from skimage.draw import line, circle
from copy import copy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from utils import distance

# The values in the occupancy grid map
OCCUPIED = 100
FREE = 0
UNKNOWN = -1


class ProbabilisticRoadmap(object):
    """Provides probabilistic roadmapping in a given map.

    Attributes:
        graph (ndarray): An adjacency matrix of size (num_nodes,num_nodes)
            consisting of edge costs between nodes.
        nodes (ndarray): Node coordinates of size (num_nodes,2)
    """

    def __init__(self, og, inflation_radius=0.25):
        """Initialize the ProbabilisticRoadmap.

        Args:
            og (nav_msgs/OccupancyGrid): The map to use for roadmapping.
            inflation_radius (float, optional): How much obstacles are inflated
                in the map in meters. Default: 0.25
        """
        super(ProbabilisticRoadmap, self).__init__()

        self.nodes = None
        self.graph = None

        # Unpack the data from the occupancy grid
        self._resolution = og.info.resolution
        self._origin = np.array([og.info.origin.position.x,
                                 og.info.origin.position.y])
        self._xmin = self._origin[0]
        self._xmax = self._origin[0] + og.info.width * self._resolution
        self._ymin = self._origin[1]
        self._ymax = self._origin[1] + og.info.height * self._resolution

        self._og_map = np.array(og.data).reshape((og.info.height,
                                                  og.info.width))

        # Inflate the obstacles in the map by inflation_radius
        self._map = self._inflate_map(self._og_map, inflation_radius)

        # Create the graph. This fills out self.nodes and self.graph
        self.create_graph()

    def _figure_coordinates(self, position):
        """Get map figure coordinates for a position.

        Args:
            position (ndarray): Array of coordinates size (2,) or (n,2).
                For a single position also list or tuple of length 2.

        Returns:
            ndarray: Coordinates of position in map figure. Same size
                as position.
        """
        position = np.array(position)
        scaled = np.atleast_2d((position - self._origin) / self._resolution)
        return np.fliplr(scaled).astype(np.uint16).reshape(position.shape)

    def _is_free(self, position):
        """Check whether a position is free in the map.

        Args:
            position (ndarray): A single position to check, size (2,).
                Alternatively list or tuple of length 2.

        Returns:
            bool: Returns True if the position is free.
        """
        index = self._figure_coordinates(position)
        return self._map[tuple(index)] == FREE

    def _inflate_map(self, og, radius):
        """Inflate the obstacles in map by a given radius.

        Args:
            og (ndarray): Array representing an occupancy grid
            radius (double): Inflation radius in meters

        Returns:
            ndarray: Inflated occupancy grid. Same size as og.
        """
        # TODO
        og = copy(og)
        return og

    def _draw_sample(self):
        """Draw a random sample from the configuration space

        Returns:
            ndarray: Sampled coordinates, size (2,).
        """
        # TODO
        sample = None
        return sample

    def can_connect(self, a, b):
        """Check whether the connecting line segment between two points
        is unobstructed.

        Args:
            a (ndarray): Coordinates for first point, size (2,)
            b (ndarray): Coordinates for second point, size (2,)

        Returns:
            bool: Returns True if there are no obstacles between the points.
        """
        # TODO
        return False

    def create_graph(self):
        """Create the nodes and connections in the graph. Fills out the class
        attributes self.nodes and self.graph with the corresponding values.
        """
        # TODO
        # Save the results to the class attributes
        self.nodes = None
        self.graph = None

    def plot(self, path=None):
        """Plot the map, nodes and connections of the ProbabilisticRoadmap

        Args:
            path (list, optional): Highlight the nodes that make up the path
        """
        ax = plt.gca()
        extent = (self._xmin, self._xmax, self._ymin, self._ymax)
        ax.imshow(self._og_map, cmap='Greys', origin='lower', extent=extent)
        ax.imshow(self._map, cmap='Reds', origin='lower',
                  extent=extent, alpha=0.3)

        ax.plot(self.nodes[:, 0], self.nodes[:, 1], 'bo')

        source, sink = np.nonzero(self.graph)
        source = self.nodes[source]
        sink = self.nodes[sink]
        lc = LineCollection(np.stack((source, sink), axis=1),
                            linewidths=[1], colors=[(0, 0.75, 1, 1)])
        ax.add_collection(lc)

        ax.set_xlim((self._xmin, self._xmax))
        ax.set_ylim((self._ymin, self._ymax))

        if path:
            path = self.nodes[path]
            ax.plot(path[:, 0], path[:, 1], 'ro-', linewidth=2)
