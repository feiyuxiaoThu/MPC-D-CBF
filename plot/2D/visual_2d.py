import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from matplotlib.patches import Ellipse
import os

def plot_ellipse(ax, ob_x, ob_y, a, b, theta, color, alpha=0.5, linestyle='-'):
    """Plots a rotated ellipse on the given axes."""
    # Darken the edge color for better contrast
    edge_color = mcolors.to_rgb(color)
    edge_color = tuple(c * 0.7 for c in edge_color)

    ellipse = Ellipse((ob_x, ob_y), 2 * a, 2 * b, angle=np.rad2deg(theta),
                      facecolor=color, alpha=alpha,
                      edgecolor=edge_color, linestyle=linestyle, linewidth=1.5)
    ax.add_patch(ellipse)

def visualize_from_files():
    """
    Reads trajectory, path, and obstacle data from files and generates a plot.
    """
    files_to_check = ["mpc_trajectory.txt", "reference_trajectory.txt", "global_path.txt", "obstacles.txt", "config.txt", "corridor_bounds.txt"]
    for f in files_to_check:
        if not os.path.exists(f):
            print(f"Error: Data file '{f}' not found.")
            print("Please ensure you have run the C++ program first to generate the data files in the same directory.")
            return

    try:
        # 1. 读取配置文件
        config = {}
        with open("config.txt") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) == 2:
                    key, value = parts
                    config[key] = int(value)
        N = config.get('N', 25) # Default to 25 if not found

        # 2. 读取数据文件
        mpc_trajectory = np.loadtxt("mpc_trajectory.txt")
        reference_trajectory = np.loadtxt("reference_trajectory.txt")
        global_path = np.loadtxt("global_path.txt")
        obstacles = np.loadtxt("obstacles.txt")
        corridor_bounds = np.loadtxt("corridor_bounds.txt")
        if obstacles.ndim == 1:
            obstacles = obstacles.reshape(1, -1)

    except (IOError, ValueError) as e:
        print(f"Error reading or parsing data files: {e}")
        return

    # 3. 创建绘图
    fig, ax = plt.subplots(figsize=(10, 10))

    # 4. 绘制轨迹
    ax.plot(global_path[:, 0], global_path[:, 1], label="Global Path", color="gray", linestyle=":")
    ax.plot(mpc_trajectory[:, 0], mpc_trajectory[:, 1], label="MPC Predicted Trajectory", color="blue", marker="o", markersize=4)
    ax.plot(reference_trajectory[:, 0], reference_trajectory[:, 1], label="Reference Trajectory", color="green", linestyle="--")

    # Highlight the terminal goal point
    terminal_goal = reference_trajectory[-1, :]
    ax.scatter(terminal_goal[0], terminal_goal[1], s=200, color="red", marker='*', zorder=5, label="Terminal Goal")

    # 绘制走廊边界
    if corridor_bounds.size > 0:
        left_x, left_y = corridor_bounds[:, 0], corridor_bounds[:, 1]
        right_x, right_y = corridor_bounds[:, 2], corridor_bounds[:, 3]
        ax.plot(left_x, left_y, color='k', linestyle='--', linewidth=0.8, label='Corridor Boundary')
        ax.plot(right_x, right_y, color='k', linestyle='--', linewidth=0.8)
        
        # 填充走廊区域
        x_fill = np.concatenate([left_x, right_x[::-1]])
        y_fill = np.concatenate([left_y, right_y[::-1]])
        ax.fill(x_fill, y_fill, color='grey', alpha=0.2)

    # 5. 绘制障碍物
    if obstacles.size > 0:
        num_obs_sequences = len(obstacles) // N
        colors = ["red", "orange", "purple", "brown", "cyan", "magenta"]
        for i in range(num_obs_sequences):
            color = colors[i % len(colors)]
            # 绘制障碍物的预测轨迹
            ob_path_x = [obstacles[i * N + j][0] for j in range(N)]
            ob_path_y = [obstacles[i * N + j][1] for j in range(N)]
            ax.plot(ob_path_x, ob_path_y, color=color, linestyle='--', linewidth=1)

            # 只在第一个和最后一个时间步绘制椭圆，以增强区分度
            for j in [0, N - 1]:
                ob_index = i * N + j
                if ob_index < len(obstacles):
                    ob = obstacles[ob_index]
                    ob_x, ob_y, a, b, theta = ob
                    # 起点: 更透明, 虚线边缘
                    if j == 0:
                        alpha = 0.4
                        linestyle = '--'
                    # 终点: 更实体, 实线边缘
                    else:
                        alpha = 0.8
                        linestyle = '-'
                    plot_ellipse(ax, ob_x, ob_y, a, b, theta, color, alpha=alpha, linestyle=linestyle)


    # 6. 绘制起始点
    ax.scatter(mpc_trajectory[0, 0], mpc_trajectory[0, 1], s=100, color="red", zorder=3, label="Start Position")

    # 7. 设置图像属性
    ax.set_title("MPC Trajectory Visualization")
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.legend()
    ax.axis("equal")
    ax.grid(True)

    # 8. 显示图像
    plt.savefig("2D.png")

if __name__ == '__main__':
    visualize_from_files()
