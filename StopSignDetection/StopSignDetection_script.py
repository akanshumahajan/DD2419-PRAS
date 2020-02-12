 #!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Stop sign detection

@author: Fredrik Forsberg
"""

import cv2
import numpy as np

###


def get_color_mask(img, bgr_color, hue_tolerance=15):
    # Color detection based on https://henrydangprg.com/2016/06/26/color-detection-in-python-with-opencv/
    hsv_color = bgr2hsv(bgr_color)
    hue = hsv_color[0][0][0]
    
    lower_hsv = np.asarray([max(hue-hue_tolerance, 0), 100, 100], dtype=np.uint8)
    upper_hsv = np.asarray([min(hue+hue_tolerance, 255), 255, 255], dtype=np.uint8)
    
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    return cv2.inRange(hsv_img, lower_hsv, upper_hsv)

#


def bgr2hsv(bgr_color):
    return cv2.cvtColor(np.array(bgr_color, dtype=np.uint8).reshape(1, 1, 3), cv2.COLOR_BGR2HSV)

#


def draw_colored_polygons(img, bgr_color, n_edges_list, hue_tolerance=15, morph_kernel_size=5, return_mask=False):
    # Blur image
    blurred_img = cv2.GaussianBlur(img, (5, 5), 0)
    
    # Get a mask based on a colour (within a set tolerance)
    mask = get_color_mask(blurred_img, bgr_color, hue_tolerance)
    
    # Morphological opening and closing
    kernel = np.ones((morph_kernel_size, morph_kernel_size), np.uint8)
    morphed_mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel), cv2.MORPH_CLOSE, kernel)
    
    contours, hierarchy = cv2.findContours(morphed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for contour in contours:
        # Polygon contour detection based on https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/
        peri = cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, 0.04 * peri, True)
        # Check number of approximated corners
        for n_edges in n_edges_list:
            if len(approx) == n_edges:
                # Draw on img
                cv2.drawContours(img, [contour], 0, (0, 255, 0), 3)
    
    if not return_mask:
        return img
    else:
        return img, mask
    
    
    
if __name__ == "__main__":
    read_type = 'video'  # 'video'
    
    if read_type == 'image':
        file_name = 'stopsign.jpg'
        bgr_color = np.array([0,  0, 180], dtype=np.uint8)
        # file_name = '1200px-Sweden_road_sign_B2.svg.png'
        # bgr_color = np.array([40,  30, 210], dtype=np.uint8)
        hue_tolerance = 10
        n_edges = 8
        
        img = cv2.imread(file_name, 1)
        # img = cv2.resize(img, (int(img.shape[1]*.5), int(img.shape[0]*.5)), interpolation=cv2.INTER_AREA) 
        
        img, mask = draw_colored_polygons(img, bgr_color, [n_edges], return_mask=True, hue_tolerance=hue_tolerance)
        
        
        cv2.imshow('mask', mask)
        cv2.imshow('img', img)
         
        while True:
          k = cv2.waitKey(0)
          if(k == 27):
            break
         
        cv2.destroyAllWindows()
    
    elif read_type == 'video':
        cap = cv2.VideoCapture(0)
        
        if (cap.isOpened()== False): 
            print("Error opening video stream or file")
        
        else:
            bgr_color = np.array([0,  0, 180], dtype=np.uint8)
            hue_tolerance = 10
            n_edges = 8
            
            while(cap.isOpened()):
                # Capture frame-by-frame
                ret, img = cap.read()
                if ret == True:
                    # Display the resulting frame
                    img, mask = draw_colored_polygons(img, bgr_color, [n_edges], return_mask=True, 
                                                      hue_tolerance=hue_tolerance)
                    
                    cv2.imshow('mask', mask)
                    cv2.imshow('img', img)
                 
                    # Press Q on keyboard to  exit
                    if cv2.waitKey(25) & 0xFF == ord('q'):
                        break
                    
                    if cv2.waitKey(25) & 0xFF == ord('s'):
                        # Save as image
                        cv2.imwrite('camera_output.png', img)
                 
                # Break the loop
                else:
                    break
            
            cap.release()
            cv2.destroyAllWindows()
        
        
    