import cv2

img = cv2.imread("./test.png")
h, w = img.shape[:2]

x_center, y_center = w // 2, h // 2

cv2.circle(img, (x_center, y_center), 5, (0, 0, 255), -1)

cv2.imshow("center", img)
cv2.waitKey(0)
cv2.destroyAllWindows()