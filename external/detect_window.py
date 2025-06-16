from detect_aruco import detect_aruco_positions
from PIL import Image
import matplotlib.pyplot as plt


## Necessary imports for the node
import numpy as np
from itertools import combinations

def detect_scale_outliers(scales, threshold=8.0):
    if len(scales) == 0:
        return []

    scales = np.array(scales)
    median = np.median(scales)

    # Compute ratio
    ratios = scales / median

    # Keep values not too far from the median
    mask = (ratios < threshold) & (ratios > (1 / threshold))

    return mask.tolist()

def detect_position_outliers(x, y, threshold_distance=3, alignment_tolerance=0.1):
    if len(x) == 0 or len(y) == 0:
        return []

    x = np.array(x)
    y = np.array(y)
    n = len(x)

    # Step 1: Find aligned pairs
    aligned_indices = set()
    for i, j in combinations(range(n), 2):
        dx = abs(x[i] - x[j])
        dy = abs(y[i] - y[j])

        if dx < alignment_tolerance or dy < alignment_tolerance:
            aligned_indices.add(i)
            aligned_indices.add(j)

    if not aligned_indices:
        return [False] * n  # No good pairs found, all are outliers

    aligned_indices = list(aligned_indices)
    aligned_x = x[aligned_indices]
    aligned_y = y[aligned_indices]

    # Step 2: Distance to all others inside aligned set
    median_x = np.median(aligned_x)
    median_y = np.median(aligned_y)
    distances = np.sqrt((aligned_x - median_x)**2 + (aligned_y - median_y)**2)
    mask_aligned = distances < threshold_distance

    # List of initially good indices
    kept_indices = np.array(aligned_indices)[mask_aligned].tolist()

    # Step 3: Allow aligned "outliers" if even number
    if len(kept_indices) % 2 == 0:
        remaining_indices = np.setdiff1d(aligned_indices, kept_indices)
        for idx in remaining_indices:
            for kept_idx in kept_indices:
                if abs(x[idx] - x[kept_idx]) < alignment_tolerance or abs(y[idx] - y[kept_idx]) < alignment_tolerance:
                    kept_indices.append(idx)
                    break  # If aligned, keep it

    # Step 4: Build output mask
    mask_final = [i in kept_indices for i in range(n)]

    return mask_final

def detect_windows(x, y, ids, scales, scale_threshold=8.0, position_threshold_distance=3, alignment_tolerance=0.1):
    """
    Detects window centers from ArUco marker positions.

    Args:
        x (list of float): Normalized X positions of markers (0 to 1).
        y (list of float): Normalized Y positions of markers (0 to 1).
        ids (list of int): Detected marker IDs.
        scales (list of float): Marker sizes (scales).
        scale_threshold (float): Threshold for scale outlier detection.
        position_threshold_distance (float): Distance threshold for position outlier detection.
        alignment_tolerance (float): Tolerance to consider markers aligned.

    Returns:
        tuple: (list of x_centers, list of y_centers, list of tuples of marker IDs)
    """
    if len(x) == 0 or len(y) == 0 or len(ids) == 0 or len(scales) == 0:
        return [], [], []

    x = np.array(x)
    y = np.array(y)
    ids = np.array(ids)
    scales = np.array(scales)

    # Step 1: Remove scale outliers
    scale_mask = detect_scale_outliers(scales, threshold=scale_threshold)
    x = x[scale_mask]
    y = y[scale_mask]
    ids = ids[scale_mask]
    scales = scales[scale_mask]

    if len(x) < 2:
        return [], [], []  # Not enough markers

    # Step 2: Remove position outliers
    position_mask = detect_position_outliers(x, y, threshold_distance=position_threshold_distance, alignment_tolerance=alignment_tolerance)
    x = x[position_mask]
    y = y[position_mask]
    ids = ids[position_mask]

    if len(x) < 2:
        return [], [], []  # Again, not enough markers

    # Prepare results
    x_centers = []
    y_centers = []
    window_ids = []

    n = len(x)
    used_indices = set()

    # Step 3: Find potential windows
    for group_size in [4, 3, 2]:
        for indices in combinations(range(n), group_size):
            if any(idx in used_indices for idx in indices):
                continue  # Don't reuse markers

            x_points = x[list(indices)]
            y_points = y[list(indices)]

            min_x, max_x = np.min(x_points), np.max(x_points)
            min_y, max_y = np.min(y_points), np.max(y_points)

            width = max_x - min_x
            height = max_y - min_y

            if group_size == 2:
                if width < alignment_tolerance or height < alignment_tolerance:
                    x_center = np.mean(x_points)
                    y_center = np.mean(y_points)
                    x_centers.append(x_center)
                    y_centers.append(y_center)
                    window_ids.append(tuple(ids[list(indices)]))
                    used_indices.update(indices)
            else:
                if width > alignment_tolerance and height > alignment_tolerance:
                    if group_size == 3:
                        # Estimate missing marker
                        missing_x = min_x if list(x_points).count(min_x) == 1 else max_x
                        missing_y = min_y if list(y_points).count(min_y) == 1 else max_y
                        x_center = (np.sum(x_points) + missing_x) / 4
                        y_center = (np.sum(y_points) + missing_y) / 4
                    else:
                        x_center = np.mean(x_points)
                        y_center = np.mean(y_points)

                    x_centers.append(x_center)
                    y_centers.append(y_center)
                    window_ids.append(tuple(ids[list(indices)]))
                    used_indices.update(indices)

    return x_centers, y_centers, window_ids

def plot_points_on_image(image, x, y, labels, circle_radius=10, circle_color='red', text_color='red'):
    """
    Plots points with labels on an image.

    Args:
        image (np.ndarray): The image as a 2D or 3D numpy array.
        x (list or np.ndarray): List of x coordinates (normalized between 0 and 1).
        y (list or np.ndarray): List of y coordinates (normalized between 0 and 1).
        labels (list): List of labels (any type, converted to string).
        circle_radius (int): Radius of circles to draw.
        circle_color (str): Color of the circle markers.
        text_color (str): Color of the labels.
    """
    if len(x) != len(y) or len(x) != len(labels):
        raise ValueError("x, y, and labels must have the same length.")

    # Make sure inputs are arrays
    x = np.array(x)
    y = np.array(y)
    labels = [str(label) for label in labels]  # Force everything to string

    height, width = image.shape[:2]

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.imshow(image, cmap='gray' if len(image.shape) == 2 else None)

    for xi, yi, label in zip(x, y, labels):
        pixel_x = xi * width
        pixel_y = yi * height
        # Draw circle
        circ = plt.Circle((pixel_x, pixel_y), circle_radius, color=circle_color, fill=False, linewidth=2)
        ax.add_patch(circ)
        # Draw label
        ax.text(pixel_x + circle_radius*1.2 , pixel_y, label, color=text_color, fontsize=12, weight='bold')

    plt.axis('off')
    plt.show()

if __name__ == "__main__":

    # Load the image
    image_path = "template_images/d2.jpg"

    # Convert the image to a PIL object then to a numpy array
    image = Image.open(image_path)
    image_numpy = np.array(image)

    # Convert the image to grayscale
    image_data = image.convert("L")

    # Get the dimensions of the image
    width, height = image_data.size

    # Convert the image to a 1D numpy array
    image_data = np.reshape(image_data, (1, -1))

    # Let the code convert the 1D array to a 2D array and detect markers
    x, y, ids, scales = detect_aruco_positions(image_data, width, height)

    # Print the results
    print("Markers x:", x)
    print("Markers y:", y)
    print("Detected IDs:", ids)
    print("Scales:", scales)

    # Remove outliers
    scale_outliers = detect_scale_outliers(scales)
    position_outliers = detect_position_outliers(x, y)
    windows_x, windows_y, windows_ids  = detect_windows(x, y, ids, scales)

    print("Scales outliers:", scale_outliers)
    print("Position outliers:", position_outliers)
    print("Detected windows x:", windows_x)
    print("Detected windows y:", windows_y)
    print("Detected windows ids:", windows_ids)

    # Plot the points on the image
    plot_points_on_image(image_numpy, x + windows_x, y + windows_y, ids + windows_ids, circle_radius=10, circle_color='red', text_color='red')