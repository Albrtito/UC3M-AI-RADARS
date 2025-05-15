# Required imports
from os import posix_openpt

import numpy as np
from Boundaries import Boundaries
from Location import Location
from numpy.linalg import det, matmul
from Radar import Radar
from tqdm import tqdm
from utils.debug import *

# Constant that avoids setting cells to have an associated cost of zero
EPSILON = 1e-4


class Map:
    """Class that models the map for the simulation"""

    def __init__(
        self,
        boundaries: Boundaries,
        height: np.int32,
        width: np.int32,
        radars: np.ndarray|None = None,
    ):
        self.boundaries = boundaries  # Boundaries of the map
        self.height = height  # Number of coordinates in the y-axis
        self.width = width  # Number of coordinates int the x-axis
        self.radars = radars  # List containing the radars (objects)

    def generate_radars(self, n_radars: np.int32) -> None:
        """Generates n-radars randomly and inserts them into the radars list"""
        # Select random coordinates inside the boundaries of the map
        lat_range = np.linspace(
            start=self.boundaries.min_lat, stop=self.boundaries.max_lat, num=self.height
        )
        lon_range = np.linspace(
            start=self.boundaries.min_lon, stop=self.boundaries.max_lon, num=self.width
        )
        rand_lats = np.random.choice(a=lat_range, size=n_radars, replace=False)
        rand_lons = np.random.choice(a=lon_range, size=n_radars, replace=False)
        self.radars = []  # Initialize 'radars' as an empty list

        # Loop for each radar that must be generated
        for i in range(n_radars):
            # Create a new radar
            new_radar = Radar(
                location=Location(latitude=rand_lats[i], longitude=rand_lons[i]),
                transmission_power=np.random.uniform(low=1, high=1000000),
                antenna_gain=np.random.uniform(low=10, high=50),
                wavelength=np.random.uniform(low=0.001, high=10.0),
                cross_section=np.random.uniform(low=0.1, high=10.0),
                minimum_signal=np.random.uniform(low=1e-10, high=1e-15),
                total_loss=np.random.randint(low=1, high=10),
                covariance=None,
            )

            # Insert the new radar
            self.radars.append(new_radar)
        return

    def get_radars_locations_numpy(self) -> np.ndarray:
        """Returns an array with the coordiantes (lat, lon) of each radar registered in the map"""
        locations = np.zeros(shape=(len(self.radars), 2), dtype=np.float32)
        for i in range(len(self.radars)):
            locations[i] = self.radars[i].location.to_numpy()
        return locations

    def get_distance_from_locations(
        self, location_1: Location, location_2: Location
    ) -> int:
        """
        Get the distance in meters between two geodesic locations. Both locations given as objects of class Location.
        """
        # Conversion factor
        k = 111000
        distance = k * np.sqrt(
            (location_1.latitude - location_2.latitude) ** 2
            + (location_1.longitude - location_2.longitude) ** 2
        )
        return distance

    def compute_possibilities_map(self) -> tuple[np.ndarray, np.float32, np.float32]:
        """
        Computes the possibility map for each defined point, also gives out the min and max possibilities in that map to ease later computations
        """
        # Start with an empty possibility map(full of zeroes)
        possibility_map = np.zeros(shape=(self.height, self.width), dtype=np.float32)
        set_min_max = 0
        min = 0
        max = 0
        for i in range(self.height):
            for j in range(self.width):
                if set_min_max == 0:
                    min = possibility_map[i, j]
                    max = possibility_map[i, j]
                    set_min_max = 1
                # Obtain the cell latitude and longitude
                lat = self.boundaries.min_lat + (
                    (self.boundaries.max_lat - self.boundaries.min_lat)
                    / (self.height - 1)
                    * i
                )
                lon = self.boundaries.min_lon + (
                    (self.boundaries.max_lon - self.boundaries.min_lon)
                    / (self.width - 1)
                    * j
                )
                for r in self.radars:
                    detection_possibility = r.compute_detection_level(lat, lon)
                    if detection_possibility > possibility_map[i, j]:
                        possibility_map[i, j] = detection_possibility

                if possibility_map[i, j] < min:
                    min = possibility_map[i, j]
                if possibility_map[i, j] > max:
                    max = possibility_map[i, j]

        return possibility_map, min, max

    def compute_detection_map(self) -> np.ndarray:
        """Computes the detection map for each coordinate in the map (with all the radars)"""
        # Start with an empty probability array
        detection_map = np.zeros(shape=(self.height, self.width), dtype=np.float32)
        # Get the possibility map
        possibility_map, min, max = self.compute_possibilities_map()
        A = max - min
        B = 1 - EPSILON
        # Go through each value in the array performing the change from possibility to probability
        for i in range(self.height):
            for j in range(self.width):
                detection_map[i, j] = ((possibility_map[i, j] - min) / A) * B + EPSILON

        return detection_map
