import cv2

cam_index = 1

cap = cv2.VideoCapture(cam_index, cv2.CAP_DSHOW)
if not cap.isOpened():
    print(f"Unable to open camera at index {cam_index}")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv2.CAP_PROP_EXPOSURE, -4)

cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
cap.set(cv2.CAP_PROP_FOCUS, 255)

cap.set(cv2.CAP_PROP_AUTO_WB, 0)
cap.set(cv2.CAP_PROP_WB_TEMPERATURE, 4500)

print("AE:", cap.get(cv2.CAP_PROP_AUTO_EXPOSURE),
      "EXP:", cap.get(cv2.CAP_PROP_EXPOSURE))
print("AF:", cap.get(cv2.CAP_PROP_AUTOFOCUS),
      "FOCUS:", cap.get(cv2.CAP_PROP_FOCUS))

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed")
        break

    # Center circle
    h, w = frame.shape[:2]
    center = (w // 2, h // 2)
    radius = 5
    color = (0, 0, 255)
    thickness = -1
    cv2.circle(frame, center, radius, color, thickness)

    cv2.imshow("C920", frame)
    cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()
