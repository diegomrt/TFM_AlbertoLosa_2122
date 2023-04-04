import numpy as np
import cv2 as cv

import matplotlib.pyplot as plt

cimg = cv.imread('/home/alberto/Imágenes/circle_Color.png')
cimg = cv.cvtColor(cimg,cv.COLOR_BGR2RGB)
assert cimg is not None, "file could not be read, check with os.path.exists()"
cimg = cv.medianBlur(cimg,7)
img = cv.cvtColor(cimg,cv.COLOR_BGR2GRAY)

blue = cimg[:,:,2]
img[blue < 100] = 0

ret, otsu = cv.threshold(blue,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
cv.imshow('blue',blue)
cv.imshow('otsu',otsu)
cv.imshow('img',img)
cv.waitKey(0)

# Calculo de círculos
circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT_ALT,0.1,40,param1=50,param2=0.9,minRadius=10,maxRadius=0)
circles = np.uint16(np.around(circles))

print(circles)

#Cálculo de líneas
edges = cv.Canny(img, 20, 50, apertureSize = 3)
cv.imshow('edges',edges)
cv.waitKey(0)

lines = cv.HoughLinesP(edges, 1, np.pi/180, 0, minLineLength=50, maxLineGap=30)

#Cálculo de bordes
img32 = np.float32(img)
dst = cv.cornerHarris(img32,3,3,0.02)

cv.imshow('borders',dst)
cv.waitKey(0)

# contours = cv.findContours(img,cv.RETR_EXTERNAL,cv.CHAIN_APPROX_NONE)

# cv.drawContours(cimg,contours,-1,(0,255,0),3)
# cv.imshow('Roi Rect ONLY', cimg)
# cv.waitKey(0)

# Histograma
histogram = cv.calcHist([img], [0], None, [256], [0, 256])

plt.figure()
plt.title('Histograma en escala de grises')
plt.xlabel('Bins')
plt.ylabel('# de pixeles')
plt.xlim((0, 256))

# Añadimos el histograma al gráfico.
plt.plot(histogram)

plt.show()


# Pintar elementos sobre la imagen
for i in circles[0,:]:
    # draw the outer circle
    cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
    # draw the center of the circle
    cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

for line in lines:
    x1, y1, x2, y2 = line[0]
    cv.line(cimg, (x1,y1), (x2,y2), (0,255,0), 1, cv.LINE_AA)

# Threshold for an optimal value, it may vary depending on the image.
cimg[dst>0.01*dst.max()]=[0,0,255]

cv.imshow('detected circles',cimg)
cv.waitKey(0)
cv.destroyAllWindows()