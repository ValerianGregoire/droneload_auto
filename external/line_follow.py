import cv2
import numpy as np
from skimage.morphology import skeletonize
from skimage.color import rgb2gray

def normalize_coords(coords, img_shape):
    h, w = img_shape[:2]
    return [(x / w, y / h) for (x, y) in coords]

def subsample_points(points, step=10):
    return points[::step]

def extract_line_points(mask, step=10):
    # Skeletonize mask
    skeleton = skeletonize(mask.astype(np.uint8))
    
    # Extract coordinates
    y_idxs, x_idxs = np.nonzero(skeleton)
    coords = list(zip(x_idxs, y_idxs))

    # Subsample and normalize
    return normalize_coords(subsample_points(coords, step), mask.shape)

def line_following_from_image(img, resize_shape=(160, 120), step=10):
    # Resize
    img_resized = cv2.resize(img, resize_shape, interpolation=cv2.INTER_AREA)

    # Convert to HSV
    hsv = cv2.cvtColor(img_resized, cv2.COLOR_RGB2HSV)

    # Define color thresholds (tune as needed)
    green_lower = np.array([40, 50, 50])
    green_upper = np.array([90, 255, 255])
    
    yellow_lower = np.array([20, 100, 100])
    yellow_upper = np.array([35, 255, 255])

    # Create masks
    green_mask = cv2.inRange(hsv, green_lower, green_upper)
    yellow_mask = cv2.inRange(hsv, yellow_lower, yellow_upper)

    # Morphological operations to clean up
    kernel = np.ones((3, 3), np.uint8)
    green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
    yellow_mask = cv2.morphologyEx(yellow_mask, cv2.MORPH_OPEN, kernel)

    # Convert to boolean mask for skeletonize
    green_skel = extract_line_points(green_mask > 0, step)
    yellow_skel = extract_line_points(yellow_mask > 0, step)

    return green_skel, yellow_skel
