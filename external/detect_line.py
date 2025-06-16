import cv2
import numpy as np
import matplotlib.pyplot as plt
from skimage.morphology import skeletonize
from skimage.util import invert
from collections import deque

# JOB TO BE DONE :
# - Subscribe to the topic to try to see if unflatten works
# - Try to do somethinf with resolution and the size of the image to reintegrate into the actual x y of the drone 
# - Check the robustness 

class LineDetect:
    def __init__(self,image):
        self.image = cv2.imread(image)
        if self.image is None:
            raise ValueError(f"Image {image} not found or unable to read.")
        self.processed_image = None
        self.skeleton = None
        self.skeleton_uint8 = None
        self.endpoints = None
        self.path = None

        # In order to unflatten the image 
        # self.image = image.reshape((self.image.shape[0], self.image.shape[1], 3))

        # Create a publisher 
        # Create a subscriber

# Function to process the image
    def process_image(self):
        # 1. Convert to HSV color space
        HSV_image = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)

        # In this range, we can find the green and yellow colors
        # 2. Create a mask for green and yellow colors
        mask_green= cv2.inRange(HSV_image, (36,0,0), (70,255,255))
        mask_yellow= cv2.inRange(HSV_image, (15, 0, 0), (36, 255, 255))

        # Add the mask together
        mask =cv2.bitwise_or(mask_green, mask_yellow)

        # Make an and logic operation with the original image
        output = cv2.bitwise_and(HSV_image, HSV_image, mask=mask)

        # Resize the image by a factor 2 for less computation
        output = cv2.resize(output, (int(output.shape[0]/2), int(output.shape[1]/2)), interpolation=cv2.INTER_AREA)

        # Convert the image to grayscale for less computation and skeletonize
        output = cv2.cvtColor(output, cv2.COLOR_HSV2BGR)
        self.processed_image = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

        return self.processed_image

# Function to skeletonize the image
    def skeletonize_image(self):
        # 2. Threshold to binary (assuming line is dark on light background)
        _, binary = cv2.threshold(self.processed_image, 50, 255, cv2.THRESH_BINARY)

        # 3. Convert to boolean for skeletonize
        binary_bool = binary > 0

        # 4. Skeletonize
        self.skeleton = skeletonize(binary_bool)

        # 5. Convert back to uint8 for display
        self.skeleton_uint8 = (self.skeleton * 255).astype(np.uint8)
        
        return self.skeleton_uint8

# Function to get the neighbors of a point in the skeleton
    def get_neighbors(self, pt):
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                x, y = pt[0] + dx, pt[1] + dy
                if 0 <= x < self.skeleton.shape[0] and 0 <= y < self.skeleton.shape[1]:
                    if self.skeleton[x, y]:
                        neighbors.append((x, y))
        return neighbors

# Function to find endpoints in the skeleton
    def find_endpoints(self):
        self.endpoints = []
        coords = np.column_stack(np.where(self.skeleton))
        for pt in coords:
            if len(self.get_neighbors(tuple(pt))) == 1:
                self.endpoints.append(tuple(pt))
        return self.endpoints

# Function to perform BFS to find the longest path from a given start point
    def bfs_longest_path(self, start):
        visited = set()
        queue = deque()
        queue.append((start, [start]))
        max_path = []
        while queue:
            current, path = queue.popleft()
            visited.add(current)
            neighbors = [n for n in self.get_neighbors(current) if n not in path]
            if not neighbors and len(path) > len(max_path):
                max_path = path
            for neighbor in neighbors:
                queue.append((neighbor, path + [neighbor]))
        self.path = max_path
        return max_path

# Function to compute the longest path in the skeleton
    def compute_longest_path(self):
        if self.endpoints is None:
            self.find_endpoints()

        longest = []
        for ep in self.endpoints:
            path = self.bfs_longest_path(ep)
            if len(path) > len(longest):
                longest = path
        self.path = longest
        return longest

# Function to run the entire process
    def run(self):
        self.process_image()
        self.skeletonize_image()
        self.find_endpoints()
        self.compute_longest_path()
        self.plot_path()

# Function to plot the path on the skeleton
    def plot_path(self):
        if self.path is None:
            raise ValueError("No path computed. Call compute_longest_path() first.")
        
        y = [self.skeleton.shape[1]-coord[0] for coord in self.path]
        x = [coord[1] for coord in self.path]

        plt.plot(x, y, marker='o', markersize=2, linestyle='-')
        plt.title("Line Following Path")
        plt.xlabel("X Coordinate")
        plt.ylabel("Y Coordinate")
        plt.xlim(left=0)
        plt.ylim(bottom=0)
        plt.xlim(right=self.skeleton.shape[1])
        plt.ylim(top=self.skeleton.shape[0])
        plt.show()

if __name__ == "__main__":
    Line = LineDetect('image.png')
    Line.run()

    