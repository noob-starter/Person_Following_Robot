import numpy as np
from scipy.interpolate import CubicSpline

def get_points(waypoints):
    # Interpolation
    t = np.linspace(0, 1, len(waypoints))  # Parameter for interpolation
    cs = CubicSpline(t, waypoints, bc_type='natural')

    # Generate path points
    num_points = 1000
    t_new = np.linspace(0, 1, num_points)
    path_points = cs(t_new)

    return path_points
