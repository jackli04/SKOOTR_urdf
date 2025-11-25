import cv2
import numpy as np
import matplotlib.pyplot as plt

def process_and_show(img):
    # Preprocess
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (9, 9), 2)

    # Detect circle
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=gray.shape[0] / 2,
        param1=100,
        param2=30,
        minRadius=50,
        maxRadius=0
    )
    if circles is None:
        print("No circles detected.")
        return

    # Extract the circle
    x0, y0, r = circles[0][0]
    x0, y0, r = float(x0), float(y0), float(r)
    center_int = (int(round(x0)), int(round(y0)))
    radius = int(r)

    # Annotate
    vis = img.copy()
    # green circle
    cv2.circle(vis, center_int, radius, (0, 255, 0), 2)
    # red center dot
    cv2.circle(vis, center_int, 5, (0, 0, 255), -1)

    # Display with matplotlib
    plt.figure(figsize=(6, 6))
    plt.imshow(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
    plt.axis('off')
    plt.show()

def main():
    cam_index = 1  # or 0 if your C920 is the first camera
    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT,1080)
    if not cap.isOpened():
        print(f"Unable to open camera at index {cam_index}")
        return

    print("Press 's' to snap & process, 'q' to quit")
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # live preview
        cv2.imshow("C920 Live", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            # snapshot and process
            process_and_show(frame)
        elif key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
