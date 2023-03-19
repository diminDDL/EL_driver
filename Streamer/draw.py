# render a x - 512px ; y - 16px image that comes in text form
# 0000000000000000 - would mean a blank column
# 1111111111111111 - would mean a full column
# the text is separated by a newline
# the image is saved as a png

import sys
import cv2
import numpy as np

# read the text file
with open("out.txt", 'r') as f:
    lines = f.readlines()

# render the image
img = np.zeros((112, 16, 3), np.uint8)
for i, line in enumerate(lines):
    for j, c in enumerate(line):
        if c == '1':
            img[i, j] = (255, 255, 255)

# save the image
cv2.imwrite('out.png', img)




