import numpy as np
import cv2 as cv

import matplotlib.pyplot as plt

# cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/lateral_no_circulo_Color.png')
cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/superior_sd_1_Color.png')
assert cimg is not None, "file could not be read, check with os.path.exists()"
cimg = cv.medianBlur(cimg,7)
img = cv.cvtColor(cimg,cv.COLOR_RGB2GRAY)

# Calculo de círculos
circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT_ALT,0.9,40,param1=200,param2=0.9,minRadius=5,maxRadius=0)
circles = np.uint16(np.around(circles))

# Pintar elementos sobre la imagen
for i in circles[0,:]:
    # draw the outer circle
    cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

circulos = circles[0,:]

if len(circulos) == 4:
    print("Pieza correcta")
elif len(circulos) == 3:
    print("Falta 1 círculo")
else:
    print("Falta ", 4-len(circulos), " circulos")

cv.imshow('img',img)
cv.imshow('cimg',cimg)
cv.waitKey(0)
cv.destroyAllWindows()