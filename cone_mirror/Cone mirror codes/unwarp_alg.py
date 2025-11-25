import cv2
import numpy as np
import matplotlib.pyplot as plt

# Extract the circle from the image, return: { circle ( x, y ), radius } 
def extract_polar(img, dp=1.2, param1=100, param2=30, min_radius=50):
    # Preprocess
    grayimg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grayimg = cv2.GaussianBlur(grayimg, (9, 9), 2)

    # Detect circle
    circles = cv2.HoughCircles(
        grayimg,
        cv2.HOUGH_GRADIENT,
        dp=dp,
        minDist=grayimg.shape[0] / 2,
        param1=param1,
        param2=param2,
        minRadius=min_radius,
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

    return center, radius

# Unwarp algorithm
def unwarp(img, center, radius):
    # Parameters
    f_epx = 1360
    f_emm = 3.88
    pixel_per_mm = f_epx / f_emm

    h_mount = 30
    h_px = h_mount * pixel_per_mm

    phi_min = -1.107
    phi_max = 1.107

    xc, yc = center
    # 1) sample radius and angle
    r     = np.linspace(0, radius, 1080)         # radial from 0…R
    theta = np.linspace(0, 2*np.pi, 1920, endpoint=False)

    # 2) make a grid of (r,θ)
    rr, th = np.meshgrid(r, theta, indexing='xy')
    #    rr.shape = (out_w, out_h), so we’ll swap axes back later

    # 3) map back to Cartesian image coords
    map_x = (xc + rr * np.cos(th)).astype(np.float32)
    map_y = (yc + rr * np.sin(th)).astype(np.float32)

    # 4) remap
    pano = cv2.remap(img, map_x, map_y, 
                     interpolation=cv2.INTER_LINEAR,
                     borderMode=cv2.BORDER_CONSTANT)

    # cv2.remap gave us shape (out_w, out_h,…), so transpose
    pano = pano.transpose(1, 0, *([] if pano.ndim==2 else [2]))
    return pano


if __name__ == "__main__":
    image = cv2.imread('./capture_test.png')

    center, radius = extract_polar(image)

    polar = unwarp(image, center, radius)

    # Display
    plt.figure(figsize=(6, 6))

    plt.imshow(cv2.cvtColor(polar, cv2.COLOR_BGR2RGB))
    plt.axis('off')

    plt.tight_layout()
    plt.show()

