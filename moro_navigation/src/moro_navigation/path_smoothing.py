#!/usr/bin/env python
import numpy as np
from quadprog import solve_qp


def smooth_path(path, num_steps=50, weight=1e-5, max_vel=0.5, max_acc=1.):
    """Return smooth position, velocity, acceleration, and jerk profiles, along
    with the associated time, for a path consisting of x,y coordinates defining
    line segments.

    Args:
        path (ndarray): Array of coordinates, size (n,2)
        num_steps (int, optional): Number of discretization steps for each line
            segment. Default: 50.
        weight (float, optional): Weight for adjusting tracking vs. smoothing.
            Default: 1e-5.
        max_vel (float, optional): Maximum velocity in m/s. Default: 0.5.
        max_acc (float, optional): Maximum acceleration in m/s**2. Default: 1.

    Returns:
        tuple: Position, velocity, acceleration, jerk, time. Time is size
            (n*num_steps,), other are size (n*num_steps,2)
    """
    # Form the variable vectors for a 4th order polynomial as a function of s,
    P = lambda s: np.array([[ TODO ]])  # position
    V = lambda s: np.array([[ TODO ]])  # velocity
    A = lambda s: np.array([[ TODO ]])  # acceleration
    J = lambda s: np.array([[ TODO ]])  # jerk

    # ... where s in the range [0,1]
    s = np.linspace(0, 1, num_steps)

    # The path consists of line segments connecting the individual coordinates
    num_segments = path.shape[0] - 1

    # The desired position on a line segment for any s is given by the line
    # equation
    pos_desired = lambda seg, s: TODO

    # The parameters to optimize are the coefficients of the polynomials for
    # the different line segments
    poly_order = 4
    num_params = (poly_order + 1)*num_segments
    param_idx = lambda seg: range(seg*(poly_order + 1),
                                  (seg + 1)*(poly_order + 1))

    # Formulate the problem in a way that it can be solved using quadratic
    # programming
    G_smooth = np.zeros((num_params, num_params))
    G_track = np.zeros((num_params, num_params))
    a = np.zeros((num_params, 2))

    for seg in range(num_segments):
        idx = param_idx(seg)
        for step in range(num_steps):
            G_smooth[np.ix_(idx, idx)] += np.dot(J(s[step]).T, J(s[step]))
            G_track[np.ix_(idx, idx)] += np.dot(P(s[step]).T, P(s[step]))
            a[idx, :] -= 2*P(s[step]).T*pos_desired(seg, s[step])
    G = 2*(weight*G_smooth + G_track)

    # Set equality constraints for the optimization
    # There are 4 global constraints and 3 for every line segment
    num_constraints = 4 + 3*(num_segments - 1)

    C = np.zeros((num_constraints, num_params))
    b = np.zeros((num_constraints, 2))

    # TODO: set global constraints (start and end nodes)

    # TODO: Assert smooth transition between line segments

    # Solve the polynomial coefficients using quadratic programming
    poly_coef = np.zeros((num_params, 2))
    poly_coef[:, 0] = solve_qp(G, -a[:, 0], -C.T, -b[:, 0],
                               meq=num_constraints)[0]
    poly_coef[:, 1] = solve_qp(G, -a[:, 1], -C.T, -b[:, 1],
                               meq=num_constraints)[0]

    # Calculate the smooth trajectory using the coefficients
    n = num_segments*num_steps

    pos = np.zeros((n, 2))
    vel = np.zeros((n, 2))
    acc = np.zeros((n, 2))
    jerk = np.zeros((n, 2))
    time = np.zeros(n)

    for seg in range(num_segments):
        idx = param_idx(seg)
        for step in range(num_steps):
            i = seg*num_steps + step
            time[i] = seg + s[step]
            pos[i, :] = np.dot(P(s[step]), poly_coef[idx, :])
            vel[i, :] = np.dot(V(s[step]), poly_coef[idx, :])
            acc[i, :] = np.dot(A(s[step]), poly_coef[idx, :])
            jerk[i, :] = np.dot(J(s[step]), poly_coef[idx, :])

    # Remove duplicate entries at beginning of segments
    valid_idx = np.unique(time, return_index=True)[1]
    pos = pos[valid_idx, :]
    vel = vel[valid_idx, :]
    acc = acc[valid_idx, :]
    jerk = jerk[valid_idx, :]
    time = time[valid_idx]

    # Time scaling for constraint on maximum velocity
    # Scale also upwards to utilize the full capacity of the base
    scaling_factor = TODO

    vel = scaling_factor*vel
    acc = TODO
    jerk = TODO
    time = TODO

    # Time scaling for constraint on maximum acceleration
    # Only scale down if limits are exceeded

    # TODO

    return pos, vel, acc, jerk, time
