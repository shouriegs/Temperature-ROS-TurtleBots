# -*- coding: utf-8 -*-
"""
Created on Sat May 15 22:22:06 2021

@author: Prithvi Bharadwaj M
"""
import yaml
import skimage.io as io
import csv
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import cv2
from PIL import Image

def RowColFromXY(x,y):                #gets the pixel location for a given real coordinate
    colNo=(x-leftBotLoc[0])/resolution
    rowNo=rows-((y-leftBotLoc[1])/resolution)
    return [int(rowNo),int(colNo)]


im = io.imread("Mapc.pgm")

plt.figure()
plt.imshow(im,cmap="gray")

# print(im.shape)
rows=im.shape[0]     #number of rows in the image
cols=im.shape[1]     #number of columns in the image

f = open("Map.yaml","r")
parsed_yaml_file = yaml.load(f, Loader=yaml.FullLoader)

resolution=parsed_yaml_file["resolution"]
origin=[0,0]
origin[0]=parsed_yaml_file["origin"][0]
origin[1]=parsed_yaml_file["origin"][1]
# print(origin)

leftBotLoc=[0,0]
leftBotLoc[0]=parsed_yaml_file["origin"][0]
leftBotLoc[1]=parsed_yaml_file["origin"][1]

data = []
fields = []

filename = "Final TempData.csv"
with open(filename, 'r') as csvfile:
    csvreader = csv.reader(csvfile)
    fields = next(csvreader)
    for row in csvreader:
        data.append(row) 

check = []
for i in range(len(data)):
    check.append(list(map(float,data[i])))

locs = []
for i in range(len(data)):
    locs.append(RowColFromXY(check[i][0], check[i][1]))
    for x in range(3):
        for y in range(3):
            im[locs[i][0]+x][locs[i][1]+y] = 0    
    
plt.figure()
plt.imshow(im,cmap="gray")

rgb = cv2.cvtColor(im,cv2.COLOR_GRAY2RGB)

maxTemp = max(l[2] for l in check)
minTemp = min(l[2] for l in check)
dif = maxTemp - minTemp

for i in range(len(data)):
    for x in range(3):
        for y in range(3):
            rgb[locs[i][0]+x][locs[i][1]+y][0] = 70 + (180/dif)*(check[i][2] - minTemp)
            rgb[locs[i][0]+x][locs[i][1]+y][2] = 180 - (180/dif)*(check[i][2] - minTemp)

legend_elements = [Patch(facecolor='blue', edgecolor='black', label='Min Temperature ' + str(minTemp)), Patch(facecolor='red', edgecolor='black', label='Max Temperature ' + str(maxTemp))]

plt.figure()
plt.imshow(rgb)
plt.legend(handles=legend_elements, loc = 'upper right', fontsize = 7)
plt.savefig('TempVisual with legend.png', dpi = 350)

result = Image.fromarray(rgb)
result.save('TempVisual plain.png')