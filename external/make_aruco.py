import cv2
import sys

def make_aruco(id_):
    # Define the dictionary we want to use
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)

    # Generate a marker
    marker_size = 400
    marker_image = cv2.aruco.generateImageMarker(aruco_dict, id_, marker_size)
    cv2.imwrite(f'markers/marker_{id_}.png', marker_image)

if __name__ == "__main__":
    if "-h" in sys.argv or "--help" in sys.argv or len(sys.argv) == 1:
        print("Usage: python make_aruco.py <marker_id1> <marker_id2> ...")
        print("Generates ArUco markers with the specified IDs.")
        print("Example: python make_aruco.py 0 1 2")
        sys.exit(0)

    for id_ in sys.argv[1:]:
        make_aruco(int(id_))
    print(f"Markers {sys.argv[1:]} generated.")