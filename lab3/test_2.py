from PIL import Image
import numpy as np

loc= 'images/image1.jpg'

a= Image.open(loc) #.convert('RGB')

(w, h)= a.size

b= np.array(a.crop((0, 0, w, h/3)))
g= np.array(a.crop((0, h/3, w, (2*h/3)-1)))
r= np.array(a.crop((0, 2*h/3, w, h)))

# tst= np.zeros((3, 3, 3))
# print(tst)

(bw, bh)= b.shape
(gw, gh)= g.shape
(rw, rh)= r.shape

# print(b.shape, g.shape, r.shape)
for bx in range(0, bw-10):
    for by in range(0, bh-10):
        b[bx:bx+10, by:by+10]
        for gx in range(0, gw-10):
            for gy in range(0, gh-10):
                g[gx:gx+10, gy:gy+10]