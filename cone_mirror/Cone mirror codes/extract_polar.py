import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('./capture_20250805_011615.png')

# Preprocess
grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
grayimg = cv2.GaussianBlur(grayimg, (9, 9), 2)

# Detect circle
circles = cv2.HoughCircles(
    grayimg,
    cv2.HOUGH_GRADIENT,
    dp=1.2,
    minDist=grayimg.shape[0] / 2,
    param1=100,
    param2=30,
    minRadius=50,
    maxRadius=0
)

if circles is None:
    raise RuntimeError("No circles detected.")
    exit()

# Extract the circle
x0, y0, r = circles[0][0]
x0, y0, r = float(x0), float(y0), float(r)
center = (x0, y0)

# Change radius for edge bubbles
radius = int(r)

# Show the circle
vis = img.copy()
cv2.circle(vis, (int(x0), int(y0)), radius, (0, 255, 0), 2)

# Show the center
center_int = (int(round(x0)), int(round(y0)))
color = (0, 0, 255)
thickness = -1
cv2.circle(vis, center_int, 5, color, thickness)

# Display
plt.figure(figsize=(6, 6))

plt.imshow(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.show()


