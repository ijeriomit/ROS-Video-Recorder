#!/usr/bin/env python2
""" Manipulates images to be used in Video Recorder"""

import numpy as np
from theia_msgs.msg import ZoneAlarm
#pylint: disable=import-error
import cv2

def image_size_correction(image, target_width, target_height):
    """function correct the size of images"""
    image = image.copy()
    if (np.size(image, 1), np.size(image, 0)) != (target_width, target_height):
        if check_aspect_ratio(image.shape, target_width, target_height):
            if np.size(image, 1) != target_width:
                image = resize_by_ratio(image, width=target_width)
            elif np.size(image, 0) != target_height:
                image = resize_by_ratio(image, height=target_height)
        else:
            if np.size(image, 1) > target_width:
                image = crop_width(image, target_width)
            elif np.size(image, 1) < target_width:
                image = pad_width(image, target_width)
            if np.size(image, 0) > target_height:
                image = crop_height(image, target_height)
            elif np.size(image, 0) < target_height:
                image = pad_height(image, target_height)
    return image

def check_aspect_ratio(shape, target_width, target_height):
    """check the aspect ratio of an image"""
    return float("{0:.2f}".format(float(shape[1])/float(shape[0]))) == float("{0:.2f}".format(float(target_width)/float(target_height)))

# pylint: disable=c-extension-no-member, invalid-name
def resize_by_ratio(image, width=None, height=None):
    """resize images of similar aspect ratios"""
    # initialize the dimensions of the image to be resized and grab the image size
    dim = None
    (h, w) = image.shape[:2]
    if width is None and height is None:
        return image
    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the dimensions
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        # calculate the ratio of the width and construct the dimensions
        r = width / float(w)
        dim = (width, int(h * r))
    #pylint: disable=c-extension-no-member
    resized = cv2.resize(image, dim, interpolation=cv2.INTER_AREA)
    return resized

def border_image(orbimage, border_depth):
    """ adds a border which changes color based on the level of violation"""
    black = [0, 0, 0]
    if orbimage.violation_level == ZoneAlarm.WARNING:
        color = [0, 255, 255]
    elif orbimage.violation_level == ZoneAlarm.VIOLATION:
        color = [0, 0, 255]
    else:
        color = black
    innerbordersize = int(border_depth * .75)
    outerbordersize = int(border_depth * .25)
    newimage = orbimage.image.copy()
    #pylint: disable=c-extension-no-member
    newimage = cv2.copyMakeBorder(newimage, innerbordersize, innerbordersize, innerbordersize, innerbordersize,
                                  cv2.BORDER_CONSTANT, value=color)
    #pylint: disable=c-extension-no-member
    newimage = cv2.copyMakeBorder(newimage, outerbordersize, outerbordersize, outerbordersize, outerbordersize,
                                  cv2.BORDER_CONSTANT, value=black)

    return newimage
def stitch_images(images, logo, no_image):
    """Stitches together the frames from all cameras"""
        # Take Orb images and make a tiled image
        # Make a huge grid video using all images from all orbs
        #            ___________________
        #            |     |     |     |
        #            |  1  |  2  |  3  |
        #            |_____|_____|_____|
        #            |     |     |     |
        #            |  4  | LOGO|  5  |
        #            |_____|_____|_____|
        #            |     |     |     |
        #            |  6  |  7  |  8  |
        #            |_____|_____|_____|
    if len(images) < 8:
        #pylint: disable=unused-variable
        for i in range(len(images), 9):
            images.append(no_image)
    stitched_image = np.vstack((np.hstack((images[0], images[1], images[2])),
                                np.hstack((images[3], logo, images[4])),
                                np.hstack((images[5], images[6], images[7]))))
    return stitched_image

def add_text_to_image(image, text):
    '''Write the date and time on the image'''
    image = image.copy()
    org = (10, image.shape[0] - 10)
    #pylint: disable=c-extension-no-member
    font_face = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1
    color = (75, 25, 230) #RED
    thickness = 2
    line_type = 2 #cv2.LINE_AA
    #pylint: disable=c-extension-no-member
    image = cv2.putText(image, text, org, font_face, font_scale, color, thickness, line_type)
    return image

def pad_width(image, target_width):
    '''pads the width of the image'''
    left = -(-(target_width - np.size(image, 1))//2) #ceiling division
    right = left
    if (target_width - np.size(image, 1))%2 != 0:
        right -= 1
    return np.pad(image, ((0, 0), (left, right), (0, 0)), 'constant', constant_values=(0, 0))

def pad_height(image, target_height):
    '''pads the height of the image'''
    top = -(-(target_height - np.size(image, 0))//2) #ceiling division
    bottom = top
    if (target_height - np.size(image, 0))%2 != 0:
        bottom -= 1
    return np.pad(image, ((top, bottom), (0, 0), (0, 0)), 'constant', constant_values=(0, 0))

def crop_width(image, target_width):
    '''crops the width of the image'''
    x = np.size(image, 1)
    start = x//2-(target_width//2)
    return image[0:np.size(image, 0), start:start+target_width]

def crop_height(image, target_height):
    '''crops the height of the image'''
    y = np.size(image, 0)
    start = y//2-(target_height//2)
    return image[start:start+target_height, 0:np.size(image, 1)]
