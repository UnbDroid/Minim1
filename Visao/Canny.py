import cv2
import numpy as np

video = cv2.VideoCapture(0)
while(True):
    flag, img = video.read()

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img_filtered = cv2.GaussianBlur(gray, (5, 5), 0)
    img_bin = cv2.threshold(img_filtered, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    edges = cv2.Canny(img_filtered,100,200)

    cv2.namedWindow("Original")
    cv2.imshow("Original", img)
    cv2.imshow("Canny", edges)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


cv2.destroyAllWindows()
