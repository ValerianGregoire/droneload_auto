from ultralytics import YOLO
import cv2


def detect_picture(image_data, width=400, height=400):
    pictures_x = []
    pictures_y = []
    classes = []
    confidences = []
    scales = []

    # Load YOLOv8 model
    model = YOLO(weights_path)
    resized_img = cv2.resize(image_data, (width, height))
    # resized_img = cv2.resize(image_data, (width, height))

    # Run inference
    results = model(resized_img)

    # Compute the center point of the found object
    for box in results[0].boxes:
        c = box.xyxy[0]
        center_x = (c[0] + c[2]) / 2
        center_y = (c[1] + c[3]) / 2

        # Normalize between 0 and 1
        norm_x = center_x / width
        norm_y = center_y / height

        # Compute the area of the bounding box using the shoelace formula
        area = (c[2] - c[0]) * (c[3] - c[1])

        # Normalize the area between 0 and 1
        norm_area = area / (width * height)

        pictures_x.append(float(norm_x))
        pictures_y.append(float(norm_y))
        classes.append(model.names[int(box.cls[0])])
        confidences.append(float(box.conf[0]))
        scales.append(float(norm_area))
    return pictures_x, pictures_y, classes, confidences, scales

if __name__ == "__main__":
    # --- Configuration ---
    weights_path = 'weights/best.pt'
    # image_paths = ["clownfish.png", "puffin.png", "shark.png", "lighthouse.png"]
    # image_paths = [f"p{i}.jpg" for i in range(1, 8)]
    image_paths = ['pics.jpg']
    target_size = (500, 500)

    # Process each image
    for image_path in image_paths:
        image_path = f"template_images/{image_path}"
        # Load and resize the image
        img = cv2.imread(image_path)
        if img is None:
            print(f"Error loading {image_path}")
            continue

        pictures_x, pictures_y, classes, confidences, scales = detect_picture(img, target_size[0], target_size[1])

        # Print the results
        print(f"Results for {image_path}:")
        img = cv2.resize(img, target_size)
        for x, y, cls, conf, scl in zip(pictures_x, pictures_y, classes, confidences, scales):
            print(f"Class: {cls}, X: {x:.2f}, Y: {y:.2f}, Confidence: {conf:.2f}, Scale: {scl:.2f}")
            # Optional: Show the image with indicated positions
            cv2.circle(img, (int(x * target_size[0]), int(y * target_size[1])), 5, (200, 200, 0), -1)
            cv2.putText(img, f"{cls}:{conf:.2f}", (int(x * target_size[0]), int(y * target_size[1]) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, (200, 200, 0), 2)
        cv2.imshow("Detected Positions", img)
        cv2.resizeWindow("Detected Positions", target_size[0], target_size[1])
        cv2.waitKey(0)
        cv2.destroyAllWindows()
