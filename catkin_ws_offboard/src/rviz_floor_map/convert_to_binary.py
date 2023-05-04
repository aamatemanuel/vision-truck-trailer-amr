#!/usr/bin/env python3
import cv2  # Import OpenCV

# read the image file
# img_name = 'maps_src/random_warehouse'
img_name = 'maps_src/Ushape'

img = cv2.imread(img_name + '.png')

ret, bw_img = cv2.threshold(img, 220, 255, cv2.THRESH_BINARY)

# converting to its binary form
bw = cv2.threshold(img, 240, 255, cv2.THRESH_BINARY)

# Display and save image
cv2.imshow("Binary", bw_img)
cv2.imwrite(img_name + "_binary.png", bw_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
