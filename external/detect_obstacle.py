import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # Creation of lidar data array
    shape = (1,360)
    lidar_data = np.ones(shape) * 3500
    lidar_data += np.random.normal(0, 10, shape)
    print(f"Lidar data: {lidar_data}")

    # Creation of a square obstacle
    square_length = 10
    square_idx = 45
    square_proximity = 180
    square_angle = -3
    square = np.zeros_like(lidar_data)
    square[0, square_idx:square_idx + square_length] = np.arange(square_proximity, square_idx + square_length-square_idx + square_proximity) + np.arange(0,square_angle*square_length,square_angle)
    print(f"Square data: {square}")

    # Insertion of the obstacle into the lidar data
    lidar_data_obstacles = lidar_data - square
    print(f"Lidar data with obstacle: {lidar_data_obstacles}")

    # Creation of a round obstacle
    square_length = 75
    square_idx = 280
    sphere = np.zeros_like(lidar_data)
    sphere[0, square_idx:square_idx + square_length] = np.sin(np.linspace(0, np.pi,square_length)) * 2 * square_length
    print(f"Sphere data: {sphere}")

    # Insertion of the obstacles into the lidar data
    lidar_data_square = lidar_data - square
    lidar_data_sphere = lidar_data - sphere
    lidar_data_obstacles = lidar_data - square - sphere
    print(f"Lidar data with square: {lidar_data_square}")
    print(f"Lidar data with sphere: {lidar_data_sphere}")
    print(f"Lidar data with obstacles: {lidar_data_obstacles}")

    # Plot of the lidar data curves
    fig, axs = plt.subplots(4,1)

    axs[0].plot(lidar_data.T)
    axs[0].set_title("Clean Lidar Data")
    axs[1].plot(lidar_data_square.T)
    axs[1].set_title("Lidar Data With Square")
    axs[2].plot(lidar_data_sphere.T)
    axs[2].set_title("Lidar Data With Sphere")
    axs[3].plot(lidar_data_obstacles.T)
    axs[3].set_title("Lidar Data With Both Obstacles")
    fig.tight_layout()
    plt.show()