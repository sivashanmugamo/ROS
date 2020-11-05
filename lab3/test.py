from PIL import Image
import numpy as np
import cv2

a= np.array(Image.open('images/image1.jpg'))
b= cv2.imread('images/image1.jpg')#, cv2.IMREAD_ANYCOLOR)

# cv2.imshow('test', b)
# cv2.waitKey(0)

print(a.shape)
print(b.shape)