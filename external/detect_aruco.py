import cv2
import numpy as np
from PIL import Image

def detect_aruco_positions(image_data, width=400, height=400):

    # imaga_data is an array of shape (1, height*width) with dtype uint8
    # Convert the image data to a 2D array
    image = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width))

    # Define the dictionary you are using (4x4_50 is a common choice)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
    parameters = cv2.aruco.DetectorParameters()

    # Create the detector
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # Detect the markers
    corners, ids, _ = detector.detectMarkers(image)

    markers_x = []
    markers_y = []
    detected_ids = []
    scales = []

    if ids is not None:
        total_area = width * height

        for corner, marker_id in zip(corners, ids.flatten()):
            # Compute the center point of the marker
            c = corner[0]
            center_x = np.mean(c[:, 0])
            center_y = np.mean(c[:, 1])

            # Normalize between 0 and 1
            norm_x = center_x / width
            norm_y = center_y / height

            # Compute the area of the marker using the shoelace formula
            area = 0.5 * np.abs(
                c[0,0]*c[1,1] + c[1,0]*c[2,1] + c[2,0]*c[3,1] + c[3,0]*c[0,1]
                - c[1,0]*c[0,1] - c[2,0]*c[1,1] - c[3,0]*c[2,1] - c[0,0]*c[3,1]
            )

            # Normalize the area between 0 and 1
            norm_area = area / total_area

            markers_x.append(norm_x)
            markers_y.append(norm_y)
            detected_ids.append(marker_id)
            scales.append(norm_area)

    return markers_x, markers_y, detected_ids, scales

# Example usage
if __name__ == "__main__":

    # Load the image (make sure to use a grayscale image)
    image_path = "template_images/window_3_arucos_broken.png"
    image_data = Image.open(image_path).convert("L")
    width, height = image_data.size # Get the dimensions of the image

    # Convert the image to a 1D numpy array
    image_data = np.reshape(image_data, (1, -1))

    # Let the code convert the 1D array to a 2D array and detect markers
    x, y, ids, scales = detect_aruco_positions(image_data, width, height)

    # Print the results
    print("Markers x:", x)
    print("Markers y:", y)
    print("Detected IDs:", ids)
    print("Scales:", scales)
