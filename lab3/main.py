import os
from PIL import Image, ImageChops
import cv2
import numpy as np

images_location= './images'

def extract_file_location(location):
    '''
    Generates a list of paths of all available images in the given directory

    Input:
        location - String
    Output:
        file_paths - Dictionary of strings
    '''
    file_paths= dict()

    for root, directory, files in os.walk(location):
        for each_file in files:
            file_paths[each_file] = root+'/'+each_file
    
    return file_paths

def extract_image_data(files):
    '''
    Reads image data from given path

    Input:
        files- Dictionary of strings
    Output:
        img_data- Dictionary of numpy arrays
    '''
    img_data= dict()

    for img, each_path in files.items():
        img_data[img] = dict()

        img_data[img]['cv2']= cv2.imread(each_path)
        # img_data[img]['pil']= np.array(Image.open(each_path))
        img_data[img]['pil']= Image.open(each_path)

    return(img_data)

# def image_processing(images):
#     for each_image in images:
#         Image.fromarray(each_image[:342][:]).show()
#         # break

# if __name__ == '__main__':
#     image_paths= extract_file_location(location= images_location)
#     image_data= extract_image_data(files= image_paths)
#     image_processing(images= image_data)

def drop_border(image):
    tst= Image.new(image.mode, image.size, image.getpixel((0, 0)))
    dif= ImageChops.difference(image, tst)
    dif= ImageChops.add(dif, dif, 2.0, -100)
    box= dif.getbbox()
    if box:
        # image.crop(box).show()
        print(image.crop(box).size)
    # dif.show()

if __name__ == '__main__':
    image_paths= extract_file_location(location= images_location)
    image_data= extract_image_data(files= image_paths)
    
    for img, data in image_data.items():
        print(data['pil'].size)
        drop_border(data['pil'])
        break