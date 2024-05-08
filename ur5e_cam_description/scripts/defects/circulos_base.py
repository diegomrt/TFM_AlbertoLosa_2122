import numpy as np
import cv2 as cv
import math

import matplotlib.pyplot as plt

import time

def main(img):
    try:
        img = cv.cvtColor(img,cv.COLOR_BGR2RGB)
        cimg = np.copy(img)
        assert cimg is not None, "file could not be read, check with os.path.exists()"

        ## DEBUG
        # cv.imshow('img',cimg)
        # cv.waitKey(0)
        # cv.destroyAllWindows()

        if __name__ != '__main__':
            cv.imwrite("/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/base" + time.strftime("%y%m%d_%H%M%S") + ".png", cimg)

        cimg = cv.medianBlur(cimg,7)
        img_gray = cv.cvtColor(cimg,cv.COLOR_RGB2GRAY)

        # Calculo de círculos
        circles = cv.HoughCircles(img_gray,cv.HOUGH_GRADIENT,1,100,param1=100,param2=16,minRadius=2,maxRadius=10)
        circles = np.uint16(np.around(circles))

        # Pintar elementos sobre la imagen
        for i in circles[0,:]:
            # draw the outer circle
            cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

        ## DEBUG
        # cv.imshow('circulos',cimg)
        # cv.waitKey(0)
        # cv.destroyAllWindows()

        circulos = circles[0,:]

        if len(circulos) == 4: # Seguimos comprobaciones
            # Voy a calcular las distancias entre circulos
            c = (int(circulos[0,0]), int(circulos[0,1])), (int(circulos[1,0]), int(circulos[1,1])), (int(circulos[2,0]), int(circulos[2,1])), (int(circulos[3,0]), int(circulos[3,1]))
            dista = np.array([math.dist(c[0],c[1]), math.dist(c[0],c[2]), math.dist(c[0],c[3])])
            max = np.argmax(dista)
            dista = np.delete(dista, max)
            lista = [1,2,3]
            lista.pop(max)
            dista2 = np.array([math.dist(c[max+1],c[lista[0]]), math.dist(c[max+1],c[lista[1]])])
            # print(dista, dista2)
            lista_dist = (dista[0], dista[1], dista2[0], dista2[1])
            mean = (dista[0] + dista[1] + dista2[0] + dista2[1])/4
            suma = 0
            # 5% de margen
            for i in lista_dist:
                if (np.abs(i-mean)/mean > 0.05):
                    suma = suma + 1
            if (suma == 0):
                # print("Pieza correcta")
                return("Pieza correcta")

            else:
                return("Pieza correcta")
                return "Distancia anormal entre círculos"



        elif len(circulos) == 3:
            return("Pieza correcta")
            return "Falta 1 círculo"
        else:
            return("Pieza correcta")
            return "Faltan " + str(4-len(circulos)) + " circulos"


    except Exception as ex:
            return ex


if __name__ == '__main__':
    img = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/superior_sd_Color.png')
    img = cv.cvtColor(img,cv.COLOR_RGB2BGR)
    
    assert img is not None, "file could not be read, check with os.path.exists()"

    resultado = main(img)

    print(resultado)