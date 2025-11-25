import cv2 as cv, time

cap = cv.VideoCapture(1, cv.CAP_DSHOW)
cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 1080)
cap.read()

cap.set(cv.CAP_PROP_AUTOFOCUS, 0)
time.sleep(0.2)
cap.set(cv.CAP_PROP_FOCUS, 200) 
time.sleep(0.1)

print("AF:", cap.get(cv.CAP_PROP_AUTOFOCUS),
      "FOCUS:", cap.get(cv.CAP_PROP_FOCUS))

cap.set(cv.CAP_PROP_AUTO_EXPOSURE, 0.25)
cap.set(cv.CAP_PROP_EXPOSURE, -7)
print("AE:", cap.get(cv.CAP_PROP_AUTO_EXPOSURE),
      "EXP:", cap.get(cv.CAP_PROP_EXPOSURE))
