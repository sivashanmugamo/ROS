from PIL import Image
import numpy as np
import cv2
import os

loc= 'images/image1.jpg'

for root, directory, files in os.walk('./images/'):
    i= 1
    for each_file in files:
        loc= root+'/'+each_file

        a= Image.open(loc).convert('RGB')

        (w, h)= a.size

        b= a.crop((0, 0, w, h/3))
        g= a.crop((0, h/3, w, (2*h/3)-1))
        r= a.crop((0, 2*h/3, w, h))

        b= b.split()[2]
        g= g.split()[1]
        r= r.split()[0]

        tst= Image.merge('RGB', (r, g, b))
        tst.save('test'+str(i)+'.png')
        i+=1