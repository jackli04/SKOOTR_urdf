import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('./test.png')

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
radius = int(r * 0.8)


# Simple Polar transform
dsize = (360, radius)
polar = cv2.warpPolar(
    img,
    dsize,
    center,
    radius,
    cv2.WARP_POLAR_LINEAR
)

# Rotate
polar = cv2.rotate(polar, cv2.ROTATE_90_CLOCKWISE)

# Display
plt.figure(figsize=(6, 6))

plt.imshow(cv2.cvtColor(polar, cv2.COLOR_BGR2RGB))
plt.axis('off')
plt.show()

