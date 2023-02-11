import cv2 as cv
import os

print("\nBy ordinal:")
for id in range(10):
    cs = cv.VideoCapture(id)
    print(f"{id}: {cs.isOpened()}")

print("\nBy path:")
for id in os.listdir("/dev/v4l/by-path"):
    cs = cv.VideoCapture(id)
    print(f"{id}: {cs.isOpened()}")

print("\nBy ID:")
for id in os.listdir("/dev/v4l/by-id"):
    cs = cv.VideoCapture(id)
    print(f"{id}: {cs.isOpened()}")
