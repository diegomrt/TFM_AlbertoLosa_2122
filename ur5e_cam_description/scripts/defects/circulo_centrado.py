import numpy as np
import cv2 as cv

import matplotlib.pyplot as plt

import time

def main(cimg):
    try:
        assert cimg is not None, "file could not be read, check with os.path.exists()"
        cimg = cv.medianBlur(cimg,7)
        cimg = cv.cvtColor(cimg,cv.COLOR_BGR2RGB)
        img = cv.cvtColor(cimg,cv.COLOR_RGB2GRAY)


        cv.imwrite("/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/centrado" + time.strftime("%y%m%d_%H%M%S") + ".png", cimg)

        # Calculo de círculos
        # circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT_ALT,0.1,10,param1=200,param2=0.9,minRadius=3,maxRadius=0)
        circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,100,param1=200,param2=19,minRadius=2,maxRadius=50)
        circles = np.uint16(np.around(circles))

        # Pintar elementos sobre la imagen
        for i in circles[0,:]:
            # draw the outer circle
            cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)
        
        cv.imshow('img',img)
        cv.imshow('cimg',cimg)
        cv.waitKey(0)
        cv.destroyAllWindows()

        circulos = circles[0,:]

        circulos_ord = circulos[circulos[:,2].argsort()]

        # print(circulos_ord)
        cv.imwrite("/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/centrado_circulos" + time.strftime("%y%m%d_%H%M%S") + ".png", cimg)

        if (len(circulos_ord) != 3):
            return "No se han encontrado 3 orificios"
        else:

            resta_1 = np.sqrt((int(circulos_ord[2][0]) - int(circulos_ord[1][0]))**2 + (int(circulos_ord[2][1]) - int(circulos_ord[1][1]))**2)
            resta_2 = np.sqrt((int(circulos_ord[2][0]) - int(circulos_ord[0][0]))**2 + (int(circulos_ord[2][1]) - int(circulos_ord[0][1]))**2)


            if resta_1 < resta_2:
                cociente = resta_1/resta_2
            else:
                cociente = resta_2/resta_1

            if cociente > 0.9:
                return "Centrado"
            else:
                return "Desviado del centro"

        # print(resta_1, ", ", resta_2)

    except Exception as ex:
            return ex

if __name__ == '__main__':
    # cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/lateral_circulo_despl_Color.png')
    # cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/lateral_sd_1_Color.png')
    cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/centrado230713_180353.png')
    assert cimg is not None, "file could not be read, check with os.path.exists()"

    cimg = cv.cvtColor(cimg,cv.COLOR_RGB2BGR)

    resultado = main(cimg)

    print(resultado)