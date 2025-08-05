import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import os

def plot_ellipse(ax, ob_x, ob_y, a, b, theta, color, alpha=0.5):
    """Plots a rotated ellipse on the given 2D axes."""
    ellipse = Ellipse((ob_x, ob_y), 2 * a, 2 * b, angle=np.rad2deg(theta),
                      facecolor=color, alpha=alpha)
    ax.add_patch(ellipse)

def visualize_from_files():
    """
    Reads trajectory, path, and obstacle data from files and generates
    both a 2D and a 3D (x-y-t) plot.
    """
    files_to_check = ["mpc_trajectory.txt", "reference_trajectory.txt", "global_path.txt", "obstacles.txt", "config.txt"]
    for f in files_to_check:
        if not os.path.exists(f):
            print(f"Error: Data file '{f}' not found.")
            print("Please ensure you have run the C++ program first to generate the data files.")
            return

    try:
        # 1. 读取配置文件
        config = {}
        with open("config.txt") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 2:
                    key, value = parts
                    config[key] = float(value)
        N = int(config.get('N', 50))
        # 使用 replan_period 作为每个预测步长的时间
        time_step = config.get('replan_period', 0.1)

        # 2. 读取数据文件
        mpc_trajectory = np.loadtxt("mpc_trajectory.txt")
        reference_trajectory = np.loadtxt("reference_trajectory.txt")
        global_path = np.loadtxt("global_path.txt")
        obstacles = np.loadtxt("obstacles.txt")
        if obstacles.ndim == 1:
            obstacles = obstacles.reshape(1, -1)

    except (IOError, ValueError) as e:
        print(f"Error reading or parsing data files: {e}")
        return

    # --- 2D Visualization ---
    fig_2d, ax_2d = plt.subplots(figsize=(10, 10))
    ax_2d.set_title("MPC Trajectory Visualization (2D Projection)")
    ax_2d.set_xlabel("X (m)")
    ax_2d.set_ylabel("Y (m)")

    ax_2d.plot(global_path[:, 0], global_path[:, 1], label="Global Path", color="gray", linestyle=":")
    ax_2d.plot(mpc_trajectory[:, 0], mpc_trajectory[:, 1], label="MPC Predicted Trajectory", color="blue", marker="o", markersize=4)
    ax_2d.plot(reference_trajectory[:, 0], reference_trajectory[:, 1], label="Reference Trajectory", color="green", linestyle="--")

    if obstacles.size > 0:
        num_obs_sequences = len(obstacles) // N
        colors = ["red", "orange", "purple", "brown"]
        for i in range(num_obs_sequences):
            color = colors[i % len(colors)]
            ob_path_x = [obstacles[i * N + j][0] for j in range(N)]
            ob_path_y = [obstacles[i * N + j][1] for j in range(N)]
            ax_2d.plot(ob_path_x, ob_path_y, color=color, linestyle='--', linewidth=1, label=f"Obstacle {i+1} Path")
            for j in [0, N - 1]:
                ob = obstacles[i * N + j]
                plot_ellipse(ax_2d, ob[0], ob[1], ob[2], ob[3], ob[4], color, alpha=0.6 if j == 0 else 0.3)

    ax_2d.scatter(mpc_trajectory[0, 0], mpc_trajectory[0, 1], s=100, color="red", zorder=3, label="Start Position")
    ax_2d.legend()
    ax_2d.axis("equal")
    ax_2d.grid(True)

    # --- 3D (X-Y-T) Visualization ---
    fig_3d = plt.figure(figsize=(12, 9))
    ax_3d = fig_3d.add_subplot(111, projection='3d')
    ax_3d.set_title("MPC Trajectory in X-Y-Time Space")
    ax_3d.set_xlabel("X (m)")
    ax_3d.set_ylabel("Y (m)")
    ax_3d.set_zlabel("Time (s)")

    # Create time axes using the replan_period
    time_mpc = np.arange(len(mpc_trajectory)) * time_step
    time_ref = np.arange(len(reference_trajectory)) * time_step

    ax_3d.plot(mpc_trajectory[:, 0], mpc_trajectory[:, 1], time_mpc, label="MPC Predicted Trajectory", color="blue", marker="o", markersize=3)
    ax_3d.plot(reference_trajectory[:, 0], reference_trajectory[:, 1], time_ref, label="Reference Trajectory", color="green", linestyle="--")

    if obstacles.size > 0:
        num_obs_sequences = len(obstacles) // N
        time_obs = np.arange(N) * time_step
        for i in range(num_obs_sequences):
            color = colors[i % len(colors)]
            ob_path_x = [obstacles[i * N + j][0] for j in range(N)]
            ob_path_y = [obstacles[i * N + j][1] for j in range(N)]
            ax_3d.plot(ob_path_x, ob_path_y, time_obs, color=color, linestyle='--', linewidth=2, label=f"Obstacle {i+1} Path")

    ax_3d.legend()
    ax_3d.grid(True)

    plt.show()

if __name__ == '__main__':
    visualize_from_files()
