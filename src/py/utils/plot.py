"""
All plotting functions are grouped here
"""

import matplotlib.pyplot as plt
import numpy as np
from Boundaries import Boundaries

# Variable for toggling on and off the plots, so that during debbugging there is no problem with showing them
PLOT = True


def plot_radar_locations(boundaries: Boundaries, radar_locations: np.ndarray) -> None:
    """Auxiliary function for plotting the radar locations"""
    plt.figure(figsize=(8, 8))
    plt.title("Radar locations in the map")
    plt.plot(
        [
            boundaries.min_lon,
            boundaries.max_lon,
            boundaries.max_lon,
            boundaries.min_lon,
            boundaries.min_lon,
        ],
        [
            boundaries.max_lat,
            boundaries.max_lat,
            boundaries.min_lat,
            boundaries.min_lat,
            boundaries.max_lat,
        ],
        label="Boundaries",
        linestyle="--",
        c="black",
    )
    plt.scatter(radar_locations[:, 1], radar_locations[:, 0], label="Radars", c="green")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.grid(True)
    plt.legend()
    if PLOT:
        plt.show()
    return


def plot_detection_fields(detection_map: np.ndarray, bicubic: bool = True) -> None:
    """Auxiliary function for plotting the detection fields"""
    plt.figure(figsize=(8, 8))
    plt.title("Radar detection fields")
    im = plt.imshow(
        # Added the origin=lower property so that it plots similar to the first graph
        X=detection_map,
        cmap="Greens",
        interpolation="bicubic" if bicubic else None,
        origin="lower",
    )
    plt.colorbar(im, label="Detection values")
    if PLOT:
        plt.show()
    return


def plot_solution(
    detection_map: np.ndarray, solution_plan: list, bicubic: bool = True
) -> None:
    """Auxiliary function for plotting the solution plan with markers in each POI"""
    plt.figure(figsize=(8, 8))
    plt.title("Solution plan")
    for i in range(len(solution_plan)):
        start_point = eval(solution_plan[i][0])
        plt.scatter(start_point[1], start_point[0], c="black", marker="*", zorder=2)
        path_array = np.zeros(shape=(len(solution_plan[i]), 2))
        for j in range(len(path_array)):
            path_array[j] = eval(solution_plan[i][j])
        plt.plot(path_array[:, 1], path_array[:, 0], zorder=1)
    final_point = eval(solution_plan[-1][-1])
    plt.scatter(
        final_point[1],
        final_point[0],
        c="black",
        marker="*",
        label=f"Waypoints",
        zorder=2,
    )
    im = plt.imshow(
        X=detection_map, cmap="Greens", interpolation="bicubic" if bicubic else None, origin = "lower"
    )
    plt.colorbar(im, label="Detection values")
    plt.legend()
    if PLOT:
        plt.show()
    return
