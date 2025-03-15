import numpy as np
import matplotlib.pyplot as plt

class Bezier_curve:
    def __init__(self, start, dest, max_bend_angle, num_points):
        self.start = start
        self.dest = dest
        self.max_bend_angle = np.deg2rad(max_bend_angle)  # Convert degrees to radians
        self.control_points = self.interpolate_control_points()
        self.num_points = num_points

    def interpolate_control_points(self):
        # Interpolate additional control points for a curved trajectory
        start = np.array(self.start)
        dest = np.array(self.dest)
        diff = dest - start
        angle = np.arctan2(diff[1], diff[0])  # Angle of the line connecting start and destination
        # Limit the angle within the range of -max_bend_angle to max_bend_angle
        angle = np.clip(angle, -self.max_bend_angle, self.max_bend_angle)
        mid = start + np.array([np.cos(angle + np.pi / 2), np.sin(angle + np.pi / 2)]) * np.linalg.norm(diff) * 0.5
        control_points = np.array([start, mid, dest])
        return control_points

    def bezier_curve(self, t):
        # Bézier curve formula
        n = len(self.control_points) - 1
        point = np.zeros(2)
        for i, p in enumerate(self.control_points):
            point += np.math.comb(n, i) * (1 - t)*(n - i) * t*i * p
        return point

    def compute_curve_points(self):
        # Generate points along the Bézier curve
        curve_points = []
        for t in np.linspace(0, 1, self.num_points):
            point = self.bezier_curve(t)
            curve_points.append(point)
        return curve_points

    def plot_curve(self):
        # Plot the points on the Bézier curve
        curve_points = self.compute_curve_points()
        curve_x, curve_y = zip(*curve_points)
        plt.plot(curve_x, curve_y, marker='o', linestyle='-')
        plt.title('Curved Trajectory')
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.grid(True)
        plt.show()

# Example usage
start = [0, 0]
dest = [0, 2]
max_bend_angle = 30  # Maximum bend angle in degrees
num_points = 5  # Number of points along the curve
curve = Bezier_curve(start, dest, max_bend_angle, num_points)
curve.plot_curve()
