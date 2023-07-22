import numpy as np
import cv2 as cv

import matplotlib.pyplot as plt

import time

def main(cimg):
    try:
        cimg = cv.medianBlur(cimg,7)
        img = cv.cvtColor(cimg,cv.COLOR_RGB2GRAY)

        cv.imwrite("/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/deforme" + time.strftime("%y%m%d_%H%M%S") + ".png", cimg)
        cv.imshow('byn',img)
        cv.waitKey(0)
        cv.destroyAllWindows()

        # Calculo de círculos
        # circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT_ALT,15,40,param1=200,param2=0.9,minRadius=10,maxRadius=0)
        circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,50,param1=200,param2=27,minRadius=4,maxRadius=0)
        circles = np.uint16(np.around(circles))

        # Pintar elementos sobre la imagen
        for i in circles[0,:]:
            # draw the outer circle
            cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

        circulos = circles[0,:]

        circulos_ord = circulos[circulos[:,2].argsort()]

        circulo_grande = circulos_ord[len(circulos_ord)-1]

        imagen_recortada = img[int(circulo_grande[1]-circulo_grande[2]*1.6):int(circulo_grande[1]+circulo_grande[2]*1.6), \
            int(circulo_grande[0]-circulo_grande[2]*1.6):int(circulo_grande[0]+circulo_grande[2]*1.6)]

        ret, otsu = cv.threshold(imagen_recortada,0,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
        img_neg = otsu

        contours, _ = cv.findContours(img_neg,cv.RETR_TREE,cv.CHAIN_APPROX_NONE)
        cv.drawContours(imagen_recortada, contours, -1, (0,255,0), 3)

        M = cv.moments(contours[0])

        area = M["m00"]
        perimetro = cv.arcLength(contours[0], True)

        print("Area: ", area, "\nPerimetro: ", perimetro)
        print((perimetro**2)/(4*area))
        print(perimetro/(2*circulo_grande[2]))

        cv.imwrite("/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/deforme_circulos" + time.strftime("%y%m%d_%H%M%S") + ".png", cimg)
        cv.imwrite("/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/deforme_recortada" + time.strftime("%y%m%d_%H%M%S") + ".png", imagen_recortada)
        cv.imwrite("/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/deforme_otsu" + time.strftime("%y%m%d_%H%M%S") + ".png", img_neg)


        cv.imshow('img',img)
        cv.imshow('cimg',cimg)
        cv.imshow('imagen_recortada',imagen_recortada)
        cv.imshow('otsu',img_neg)
        cv.waitKey(0)
        cv.destroyAllWindows()

        if ((perimetro**2)/(4*area) < 3.55 and perimetro/(2*circulo_grande[2]) < 3.3):
            return "bien"
        else:
            return "mal"
    except Exception as ex:
            return ex

if __name__ == '__main__':
    # cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/lateral_circulo_despl_Color.png')
    # cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/lateral_no_circulo_Color.png')
    # cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/lateral_sd_1_Color.png')
    cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/deforme230713_162217.png')
    
    assert cimg is not None, "file could not be read, check with os.path.exists()"

    resultado = main(cimg)

    print(resultado)
