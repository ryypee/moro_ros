#!/usr/bin/env python
import numpy as np


class DotDict(dict):
    """dot.notation access to dictionary attributes"""
    def __getattr__(*args):
        val = dict.get(*args)
        return DotDict(val) if type(val) is dict else val
    __setattr__ = dict.__setitem__
    __delattr__ = dict.__delitem__


def distance(a, b):
    """Computes the Euclidean distance between points.

    Args:
        a (ndarray): Point size (2,)
        b (ndarray): Point size (2,) or array of points size (n,2)

    Returns:
        ndarray: Array of distances size (1,) or (n,)
    """
    a = np.atleast_2d(a)
    b = np.atleast_2d(b)
    return np.linalg.norm(a - b, axis=1)


def decompose(a):
    """Decompose a vector into magnitude and direction.

    Args:
        a (ndarray): Vector to decompose

    Returns:
        tuple: magnitude, direction
    """
    magnitude = np.linalg.norm(a)
    direction = a/magnitude
    return magnitude, direction


def wrap_to_pi(angle):
    """Wrap the input angle between [-pi,pi)

    Args:
        angle (double): Angle in radians

    Returns:
        double: The input wrapped between [-pi,pi)
    """
    return (angle + np.pi) % (2 * np.pi) - np.pi
