#!/usr/bin/python

# 2.12 Lab 4 object detection: a node for de-noising
# Luke Roberto Oct 2017

import numpy as np
import cv2  # OpenCV module

import math

image_folder_path = "/home/robot/me212lab4/catkin_ws/src/me212cv/images/" # TODO: Use relative paths

def main():
    
    # Path to different images
    image1 = image_folder_path + "Example_lena_denoise_noisy.jpg"
    image2 = image_folder_path + "Flower_noisy.jpg"
    
    print(image1)
    print(image2)
    # Read in image file
    cv_image = cv2.imread(image1)
    
    
    # 2. visualize it in a cv window
    cv2.imshow("Original_Image", cv_image)
    cv2.waitKey(5)
     
    # 3. Denoise image
    dn_cv_image = cv2.fastNlMeansDenoising(cv_image, 10, 14, 21)
    cv2.imshow("Denoised_Image", dn_cv_image)
    cv2.waitKey(0)
    print("Denoising...")
    
if __name__=='__main__':
    main()
    
