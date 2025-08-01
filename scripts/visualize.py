import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse

def plot_ellipse(ax, ob_x, ob_y, a, b, theta, color):
    """Plots a rotated ellipse on the given axes."""
    ellipse = Ellipse((ob_x, ob_y), 2 * a, 2 * b, angle=np.rad2deg(theta),
                      edgecolor=color, facecolor=color, alpha=0.5)
    ax.add_patch(ellipse)

def visualize_from_files():
    """
    Reads trajectory, path, and obstacle data from files and generates a plot.
    """
    try:
        # 1. 读取数据文件
        mpc_trajectory = np.loadtxt("mpc_trajectory.txt")
        reference_trajectory = np.loadtxt("reference_trajectory.txt")
        global_path = np.loadtxt("global_path.txt")
        obstacles = np.loadtxt("obstacles.txt")
        if obstacles.ndim == 1:
            obstacles = obstacles.reshape(1, -1)

    except IOError as e:
        print(f"Error reading data files: {e}")
        print("Please ensure you have run the C++ program first to generate the data files.")
        return

    # 2. 创建绘图
    fig, ax = plt.subplots(figsize=(10, 10))

    # 3. 绘制轨迹
    ax.plot(global_path[:, 0], global_path[:, 1], label="Global Path", color="gray", linestyle=":")
    ax.plot(mpc_trajectory[:, 0], mpc_trajectory[:, 1], label="MPC Predicted Trajectory", color="blue", marker="o", markersize=4)
    ax.plot(reference_trajectory[:, 0], reference_trajectory[:, 1], label="Reference Trajectory", color="green", linestyle="--")

    # 4. 绘制障碍物
    if obstacles.size > 0:
        for i, ob in enumerate(obstacles):
            ob_x, ob_y, a, b, theta = ob
            color = "red" if i == 0 else "orange"
            plot_ellipse(ax, ob_x, ob_y, a, b, theta, color)

    # 5. 绘制起始点
    ax.scatter(mpc_trajectory[0, 0], mpc_trajectory[0, 1], s=100, color="red", zorder=3, label="Start Position")

    # 6. 设置图像属性
    ax.set_title("MPC Trajectory Visualization (from Python)")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.axis("equal")
    ax.grid(True)

    # 7. 显示图像
    plt.show()

if __name__ == '__main__':
    visualize_from_files()
