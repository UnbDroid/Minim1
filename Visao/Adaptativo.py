import cv2
import numpy as np

video = cv2.VideoCapture(0)

while True:
    _, img = video.read()
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # _, img_bin = cv2.threshold(img_filtered, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    img_filtered = cv2.GaussianBlur(img, (5, 5), 0)
    thresh = cv2.adaptiveThreshold(img,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,2)

    cv2.imshow("Original", img)
    cv2.imshow("Binario", thresh)

    if cv2.waitKey(30) == 27:
        break
