
# %%

import matplotlib.pyplot as plt
import cv2 as cv
import numpy as np
import imutils as im


def empty(a): pass


CAMERA_WIDTH = 320
HEIGHT_FACTOR = 0.7

cv.namedWindow('Color Window')
cv.resizeWindow('Color Window', 240, 320)
cv.createTrackbar('R Min', 'Color Window', 0, 255, empty)
cv.createTrackbar('R Max', 'Color Window', 255, 255, empty)
cv.createTrackbar('G Min', 'Color Window', 0, 255, empty)
cv.createTrackbar('G Max', 'Color Window', 255, 255, empty)
cv.createTrackbar('B Min', 'Color Window', 0, 255, empty)
cv.createTrackbar('B Max', 'Color Window', 255, 255, empty)

img_original = cv.imread('ring_pic_2.jpg', 1)
yCrCb = cv.cvtColor(img_original, cv.COLOR_BGR2YCR_CB)
img_scale = cv.resize(yCrCb, (240, 320))
img_scale_orig = cv.resize(img_original, (240, 320))

while True:
    # r_min = cv.getTrackbarPos('R Min', 'Color Window')
    # r_max = cv.getTrackbarPos('R Max', 'Color Window')
    # g_min = cv.getTrackbarPos('G Min', 'Color Window')
    # g_max = cv.getTrackbarPos('G Max', 'Color Window')
    # b_min = cv.getTrackbarPos('B Min', 'Color Window')
    # b_max = cv.getTrackbarPos('B Max', 'Color Window')

    # lower_range = np.array([r_min, g_min, b_min])
    # upper_range = np.array([r_max, g_max, b_max])
    lower_range = np.array([0, 178, 0])
    upper_range = np.array([209, 255, 111])

    thresh = cv.inRange(img_scale, lower_range, upper_range)
    thresh = cv.GaussianBlur(thresh, (5, 15), 0)
    bitwise = cv.bitwise_and(img_scale_orig, img_scale_orig, mask=thresh)

    cv.imshow('Original Image', img_scale_orig)
    cv.imshow('Thresholded', thresh)
    cv.imshow('Bitwise', bitwise)
    k = cv.waitKey(1) & 0xFF
    if k == ord('m'):
        mode = not mode
    elif k == 27:
        break
cv.destroyAllWindows()
# %%
