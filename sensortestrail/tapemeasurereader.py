#use tesseract for extracting ocr
import time
import argparse
import cv2
import numpy as np
from os import listdir
from os.path import isfile, join
from matplotlib import pyplot as plt
import Image
import pytesseract


folder_input = '/constant_increasing_0deg/data'
#folder_output = '../results/thresholddetect'

filename = 'test0.bmp'

im = cv2.imread('ocrtest.jpeg')
rows,cols,temp = im.shape

M = cv2.getRotationMatrix2D((cols/2,rows/2),90,1)
dst = cv2.warpAffine(im,M,(cols,rows))

cv2.imshow('rotate',dst)

cv2.waitKey(0)

#print pytesseract.image_to_string(Image.open('TestPilotCollective-OCRK-2011-12-05.gif'))


