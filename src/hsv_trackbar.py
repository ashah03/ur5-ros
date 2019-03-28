from urllib import urlopen

import cv2
import numpy as np

webcam = cv2.VideoCapture(0)

def nothing(x):
    pass


# Creating a window for later use
cv2.namedWindow('hsvTrackbar', cv2.WINDOW_NORMAL)
cv2.resizeWindow('hsvTrackbar', 1000,1000)


# cv.CreateTrackbar(trackbarName, windowName, value, count, onChange)  None
cv2.createTrackbar('Hue', 'hsvTrackbar', 0, 180, nothing)  # default 0 205 255 69 8 12
cv2.createTrackbar('Sat', 'hsvTrackbar', 205, 255, nothing)
cv2.createTrackbar('Val', 'hsvTrackbar', 255, 255, nothing)
cv2.createTrackbar('Hrange', 'hsvTrackbar', 69, 127, nothing)
cv2.createTrackbar('Srange', 'hsvTrackbar', 69, 127, nothing)
cv2.createTrackbar('Vrange', 'hsvTrackbar', 69, 127, nothing)

def get_image():
    url = "http://192.168.178.193:4242/current.jpg?type=color"
    resp = urlopen(url)
    image = np.asarray(bytearray(resp.read()), dtype="uint8")
    image = cv2.imdecode(image, cv2.IMREAD_COLOR)
    return image

while True:
    frame = get_image()

    frame = cv2.GaussianBlur(frame, (5, 5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # get info from track bar and apply to result
    hue = cv2.getTrackbarPos('Hue', 'hsvTrackbar')
    sat = cv2.getTrackbarPos('Sat', 'hsvTrackbar')
    val = cv2.getTrackbarPos('Val', 'hsvTrackbar')
    hrange = cv2.getTrackbarPos('Hrange', 'hsvTrackbar')
    srange = cv2.getTrackbarPos('Srange', 'hsvTrackbar')
    vrange = cv2.getTrackbarPos('Vrange', 'hsvTrackbar')

    colorLower = (hue - hrange, sat - srange, val - vrange)
    colorUpper = (hue + hrange, sat + srange, val + vrange)

    filteredFrame = cv2.inRange(hsv, colorLower, colorUpper)
    colorCutout = cv2.bitwise_and(frame, frame, mask=filteredFrame)

    cv2.imshow('hsvTrackbar', colorCutout)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

webcam.release()
cv2.destroyAllWindows()