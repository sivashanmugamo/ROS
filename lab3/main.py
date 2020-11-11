# Importing required libraries
import os
import sys
import math
import numpy as np

from PIL import Image, ImageChops, ImageFilter

images_location= sys.argv[-1]

def extract_file_location(location):
    '''
    Extracts the paths of all images in a given directory

    Input:
        location - string
    Output:
        files_paths - list of strings
    '''

    file_paths= dict()

    for root, directory, files in os.walk(location):
        for each_file in files:
            file_paths[each_file]= os.path.join(root, each_file)

    return file_paths

def extract_image_data(img_path):
    '''
    Reads image data from the given path

    Input:
        img_path - string
    Output:
        img_data - dictionary with PIL jpeg image file (both L & RGB data)
    '''

    img_data= dict()
    if os.path.isfile(img_path):
        temp_img= Image.open(img_path)
        img_data['Original']= dict()
        img_data['Original']['L']= temp_img
        img_data['Original']['RGB']= temp_img.convert('RGB')
        return img_data
    else:
        FileNotFoundError

def drop_border(image):
    '''
    Removes the border around the given image
    Ref: https://www.stackoverflow.com/questions/19271692/removing-borders-from-an-image-in-python

    Input:
        image - PIL jpeg image file
    Output:
        cropped image - PIL jpeg image file
    '''

    img_bg= Image.new(
        mode= image.mode, 
        size= image.size, 
        color= image.getpixel((0, 0))
    )
    img_dif= ImageChops.difference(
        image1= image, 
        image2= img_bg
    )
    img_dif= ImageChops.add(
        image1= img_dif, 
        image2= img_dif, 
        scale= 2.0, 
        offset= -100
    )
    img_box= img_dif.getbbox()
    if img_box:
        return image.crop(img_box)
    else:
        print('Could not crop')

def channel_overlay(img_data):
    '''
    Merges the R, G, & B channel, and saves the image as 'image*-color.png'

    Input:
        img_data - Dictionary of PIL jpeg image data (both L & RGB)
    Output:
        Saves color images in main directory
    '''

    for img_key, img_data in img_data.items():
        image= img_data['Original']['RGB']
        (img_w, img_h)= image.size

        img_b= image.crop((0, 0, img_w, math.floor(img_h/3)))
        img_g= image.crop((0, math.floor(img_h/3), img_w, math.floor(2*img_h/3)))
        img_r= image.crop((0, math.floor(2*img_h/3), img_w, img_h-1))

        img_b= img_b.split()[2]
        img_g= img_g.split()[1]
        img_r= img_r.split()[0]

        Image.merge('RGB', (img_r, img_g, img_b)).save(img_key[:6]+'-color.png')

def ssd(img_data, mode):
    '''
    Calcualtes the SSD between the reference/base channel (red) and the other channels, finds the displacement vector, aligns the images,
    and saves the image as 'image*-ssd.png'
    Ref: 
        1. SSD - https://www.w3resource.com/python-exercises/math/python-math-exercise-11.php
        2. Image windowing - https://medium.com/outco/how-to-solve-sliding-window-problems-28d67601a66

    Input:
        img_data - Dictionary of PIL jpeg image data (both L & RGB)
        mode - integer (0 or 1) - denotes the usage of orignal image (0) or borderless image (1)
    Output:
        Saves aligned images in  main directory
    '''

    for img_key, each_img_data in img_data.items():
        print('----------------- SSD '+img_key+'--------------------')
        if mode in [0, 1]:
            if mode == 0:
                image= each_img_data['Original']['RGB']
            elif mode == 1:
                image= each_img_data['Borderless']['RGB']
            
            (img_w, img_h)= image.size

            img_b= image.crop((0, 0, img_w, math.floor(img_h/3))).split()[2]
            img_g= image.crop((0, math.floor(img_h/3), img_w, math.floor(2*img_h/3))).split()[1]
            img_r= image.crop((0, math.floor(2*img_h/3), img_w, img_h-1)).split()[0]

            img_b_edge= img_b.filter(ImageFilter.FIND_EDGES)
            img_g_edge= img_g.filter(ImageFilter.FIND_EDGES)
            img_r_edge= img_r.filter(ImageFilter.FIND_EDGES)

            img_b= np.array(img_b)
            img_g= np.array(img_g)
            img_r= np.array(img_r)

            temp_ssd_g= np.inf
            temp_ssd_b= np.inf
            save_i= save_j= 0

            for i in range(-25, 25):
                for j in range(-25, 25):
                    ssd_g= np.sum(np.power(np.subtract(img_r_edge, np.roll(img_g_edge, (i, j), axis= (0, 1))), 2))
                    ssd_b= np.sum(np.power(np.subtract(img_r_edge, np.roll(img_b_edge, (i, j), axis= (0, 1))), 2))

                    if ssd_g<=temp_ssd_g:
                        temp_ssd_g= ssd_g
                        save_gi= i
                        save_gj= j
                        save_img_g= np.roll(img_g, (i, j), axis= (0, 1))

                    if ssd_b<=temp_ssd_b:
                        temp_ssd_b= ssd_b
                        save_bi= i
                        save_bj= j
                        save_img_b= np.roll(img_b, (i, j), axis= (0, 1))

            print('Green displacement :'+str((save_gi, save_gj)))
            print('Blue displacement :'+str((save_bi, save_bj)))

            img_r= Image.fromarray(img_r)
            img_g= Image.fromarray(save_img_g)
            img_b= Image.fromarray(save_img_b)

            Image.merge('RGB', [img_r, img_g, img_b]).save(img_key[:6]+'-ssd.png')
                    
        else:
            print('Invalid mode - Mode can either be 1 or 0')

def ncc(img_data, mode):
    '''
    Calcualtes the NCC (Normalized Cross Correlation) between the reference/base channel (red) and the other channels, finds the displacement vector, aligns the images,
    and saves the image as 'image*-ssd.png'
    Ref: 
        1. NCC wiki - https://en.wikipedia.org/wiki/Cross-correlation#Normalized_cross-correlation
        2. Image windowing - https://medium.com/outco/how-to-solve-sliding-window-problems-28d67601a66

    Input:
        img_data - Dictionary of PIL jpeg image data (both L & RGB)
        mode - integer (0 or 1) - denotes the usage of orignal image (0) or borderless image (1)
    Output:
        Saves aligned images in  main directory
    '''

    for img_key, each_img_data in img_data.items():
        print('----------------- NCC '+img_key+'--------------------')
        if mode in [0, 1]:
            if mode == 0:
                image= each_img_data['Original']['RGB']

            elif mode == 1:
                image= each_img_data['Borderless']['RGB']
                (img_w, img_h)= image.size
                image= image.crop((5, 5, img_w-5, img_h-5))
                (img_w, img_h)= image.size

                img_b= image.crop((0, 0, img_w, math.floor(img_h/3))).split()[2]
                img_g= image.crop((0, math.floor(img_h/3), img_w, math.floor(2*img_h/3))).split()[1]
                img_r= image.crop((0, math.floor(2*img_h/3), img_w, img_h)).split()[0]

                if(img_r.size[1]+img_g.size[1]+img_b.size[1])%3==0:
                    pass
                elif(img_r.size[1]+img_g.size[1]+img_b.size[1])%3==1:
                    img_b= image.crop((0, 0, img_w, math.floor(img_h/3))).split()[2]
                    img_g= image.crop((0, math.floor(img_h/3), img_w, math.floor(2*img_h/3))).split()[1]
                    img_r= image.crop((0, math.floor(2*img_h/3), img_w, img_h-1)).split()[0]
                elif(img_r.size[1]+img_g.size[1]+img_b.size[1])%3==2:
                    img_b= image.crop((0, 0, img_w, math.floor(img_h/3))).split()[2]
                    img_g= image.crop((0, math.floor(img_h/3), img_w, math.floor(2*img_h/3)-1)).split()[1]
                    img_r= image.crop((0, math.floor(2*img_h/3), img_w, img_h-1)).split()[0]

            img_b_edge= img_b.filter(ImageFilter.FIND_EDGES)
            img_g_edge= img_g.filter(ImageFilter.FIND_EDGES)
            img_r_edge= img_r.filter(ImageFilter.FIND_EDGES)

            img_b= np.array(img_b)
            img_g= np.array(img_g)
            img_r= np.array(img_r)

            temp_ssd_g= -1
            temp_ssd_b= -1
            save_i= save_j= 0

            for i in range(-25, 25):
                for j in range(-25, 25):
                    ncc_g= np.corrcoef(np.array(img_r_edge).ravel(), np.roll(img_g_edge, (i, j), axis= (0, 1)).ravel())
                    ncc_b= np.corrcoef(np.array(img_r_edge).ravel(), np.roll(img_b_edge, (i, j), axis= (0, 1)).ravel())

                    if ncc_g[0][1] >= temp_ssd_g:
                        temp_ssd_g= ncc_g[0][1]
                        save_gi= i
                        save_gj= j
                        save_img_g= np.roll(img_g, (i, j), axis= (0, 1))

                    if ncc_b[0][1] >= temp_ssd_b:
                        temp_ssd_b= ncc_b[0][1]
                        save_bi= i
                        save_bj= j
                        save_img_b= np.roll(img_b, (i, j), axis= (0, 1))

            print('Green displacement :'+str((save_gi, save_gj)))
            print('Blue displacement :'+str((save_bi, save_bj)))

            img_r= Image.fromarray(img_r)
            img_g= Image.fromarray(save_img_g)
            img_b= Image.fromarray(save_img_b)

            Image.merge('RGB', [img_r, img_g, img_b]).save(img_key[:6]+'-ncc.png')
                    
        else:
            print('Invalid mode - Mode can either be 1 or 0')

if __name__ == '__main__':
    image_paths= extract_file_location(location= images_location)
    
    image_data= dict()
    for image_key, image_path in image_paths.items():
        image_data[image_key]= extract_image_data(img_path= image_path)

    for image_key, each_image_data in image_data.items():
        image_data[image_key]['Borderless']= dict()
        image_data[image_key]['Borderless']['L']= drop_border(image= each_image_data['Original']['L'])
        image_data[image_key]['Borderless']['RGB']= drop_border(image= each_image_data['Original']['RGB'])

    # Part-1
    # Color-channel overlay
    channel_overlay(img_data= image_data)

    # Part-2
    # Sum Squared Differences
    ssd(img_data= image_data, mode= 0)

    # Normalized Cross Correlation
    ncc(img_data= image_data, mode= 1)
