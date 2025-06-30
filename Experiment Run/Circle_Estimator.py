import numpy as np
from scipy.optimize import least_squares
from sklearn.neighbors import NearestNeighbors
import matplotlib.pyplot as plt
import pandas as pd
from typing import Optional
import time 

class CircleEstimator:
    def __init__(self, points: np.ndarray):
        """
        points: (N,3) or (N,2) numpy array. If (N,3), only x and z are used.
        """
        if points.shape[1] == 3:
            self.points = points[:, [0, 2]]  # Use x, z only
        elif points.shape[1] == 2:
            self.points = points
        else:
            raise ValueError("Input points must have shape (N,2) or (N,3)")
        self.filtered_points: Optional[np.ndarray] = None
        self.taubin_result: Optional[np.ndarray] = None
        self.final_result: Optional[np.ndarray] = None

    def statistical_outlier_removal(self, k=8, z_thresh=2.0) -> np.ndarray:
        points = self.points
        nbrs = NearestNeighbors(n_neighbors=k+1).fit(points)
        distances, _ = nbrs.kneighbors(points)
        mean_dists = distances[:, 1:].mean(axis=1)
        mean, std = np.mean(mean_dists), np.std(mean_dists)
        mask = np.abs(mean_dists - mean) < z_thresh * std
        self.filtered_points = points[mask]
        return self.filtered_points

    def taubin_circle_fit(self, points: Optional[np.ndarray]=None) -> np.ndarray:
        if points is None:
            points = self.filtered_points if self.filtered_points is not None else self.points
        x = points[:, 0]
        z = points[:, 1]
        A = np.column_stack([x, z, np.ones_like(x)])
        B = x**2 + z**2
        coeffs, _, _, _ = np.linalg.lstsq(A, B, rcond=None)
        a = coeffs[0] / 2
        b = coeffs[1] / 2
        r = np.sqrt(coeffs[2] + a**2 + b**2)
        self.taubin_result = np.array([a,b,r])
        return self.taubin_result
    
    def filter(self, do_plot: bool = True):
        filtered_pcd = self.statistical_outlier_removal()
        result = self.taubin_circle_fit(filtered_pcd)
        return result


    def fit_and_report(self, do_plot: bool = True):
        filtered = self.statistical_outlier_removal()
        taubin = self.taubin_circle_fit(filtered) # fast, but inaccurate
        df_result = taubin 
        # print(df_result)
        return df_result
