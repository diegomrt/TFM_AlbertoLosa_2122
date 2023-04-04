import numpy as np
import cv2 as cv

import matplotlib.pyplot as plt

cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/lateral_circulo_despl_Color.png')
# cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/lateral_sd_1_Color.png')
assert cimg is not None, "file could not be read, check with os.path.exists()"
cimg = cv.medianBlur(cimg,7)
img = cv.cvtColor(cimg,cv.COLOR_RGB2GRAY)

# Calculo de círculos
circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT_ALT,0.1,40,param1=200,param2=0.9,minRadius=10,maxRadius=0)
circles = np.uint16(np.around(circles))

# Pintar elementos sobre la imagen
for i in circles[0,:]:
    # draw the outer circle
    cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

circulos = circles[0,:]

circulos_ord = circulos[circulos[:,2].argsort()]

print(circulos_ord)

resta_1 = np.sqrt((int(circulos_ord[2][0]) - int(circulos_ord[1][0]))**2 + (int(circulos_ord[2][1]) - int(circulos_ord[1][1]))**2)
resta_2 = np.sqrt((int(circulos_ord[2][0]) - int(circulos_ord[0][0]))**2 + (int(circulos_ord[2][1]) - int(circulos_ord[0][1]))**2)

if resta_1 < resta_2:
    cociente = resta_1/resta_2
else:
    cociente = resta_2/resta_1

if cociente > 0.9:
    print("Centrado")
else:
    print("Desviado del centro")

print(resta_1, ", ", resta_2)

# cv.imshow('img',img)
# cv.imshow('cimg',cimg)
# cv.waitKey(0)
# cv.destroyAllWindows()