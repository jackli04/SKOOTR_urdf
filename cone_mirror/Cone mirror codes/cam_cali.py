import cv2
import numpy as np

def main():
    cam_index = 1
    cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"Unable to open camera at index {cam_index}")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)


    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Convert to grayscale and blur to reduce noise
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, (9, 9), 2)

        # Detect circles using HoughCircles
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=gray.shape[0] / 8,
            param1=80,
            param2=50,
            minRadius=0,
            maxRadius=0
        )

        if circles is not None:
            # Round and convert to integers
            circles = np.round(circles[0]).astype("int")
            for (x, y, r) in circles:
                # Draw the circle outline in green
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
                # Mark the center in blue
                cv2.circle(frame, (x, y), 5, (255, 0, 0), -1)

        # Display the result
        cv2.imshow("Circle Detection", frame)
        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
