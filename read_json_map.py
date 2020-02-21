#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on Fri Feb 21 16:00:19 2020

Transform a json world file to a grid 

@author: Fredrik Forsberg
"""

import json
import numpy as np
import cv2

###


def read_json(path):
    with open(path) as f:
        json_dict = json.load(f)
        
    return json_dict


def create_grid(json_dict, grid_res=10):
    # grid_res is the number of rows/columns per meter => 10 means that each cell represents 1 dm x 1 dm
    offset = json_dict['airspace']['min'][:2]
    max_point = json_dict['airspace']['max'][:2]
    
    x_len = max_point[0] - offset[0]
    y_len = max_point[1] - offset[1]
    
    grid = np.zeros((x_len*grid_res, y_len*grid_res), dtype=np.byte)
    
    return grid, np.asarray(offset, dtype=np.float64)


def draw_walls(json_dict, grid, offset, grid_res=10):
    # line_res is how many points per meter there are to be between the start and the stop points of a wall
    line_res = 10 * grid_res
    for  d in json_dict['walls']:
        start = np.array(d['plane']['start'][:2]) - offset
        stop = np.array(d['plane']['stop'][:2]) - offset
        
        n_points = int(np.linalg.norm(start - stop) * line_res)
        
        for point in np.linspace(start, stop, n_points):
            floored_point = np.floor(point * grid_res).astype(dtype=np.int)
            grid[floored_point[0]][floored_point[1]] = 1
            
    return grid
    

###
    

if __name__ == '__main__':
    json_file = '~/dd2419_ws/src/course_packages/dd2419_resources/worlds_json/tutorial_1.world.json'
    json_dict = read_json(json_file)
    grid, offset = create_grid(json_dict, grid_res=10)
    grid = draw_walls(json_dict, grid, offset, grid_res=10)
    
    # Plot the grid as an image just to make sure it works
    img = np.zeros(tuple(list(grid.shape) + [3]))
    img[:,:,0] = grid * 255
    img[:,:,1] = grid * 255
    img[:,:,2] = grid * 255
    
    cv2.imshow("2D map", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    