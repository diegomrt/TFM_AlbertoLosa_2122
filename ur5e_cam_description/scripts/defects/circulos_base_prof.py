import numpy as np
import cv2 as cv
import math

import matplotlib.pyplot as plt

import time

def main(img, img_prof):
    try:
        img = cv.cvtColor(img,cv.COLOR_BGR2RGB)
        cimg = np.copy(img)
        assert cimg is not None, "file could not be read, check with os.path.exists()"

        cv.imshow('img',img)
        cv.waitKey(0)
        cv.destroyAllWindows()

        cv.imwrite("/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/base" + time.strftime("%y%m%d_%H%M%S") + ".png", cimg)

        # cimg = cv.medianBlur(cimg,7)
        img_gray = cv.cvtColor(cimg,cv.COLOR_RGB2GRAY)

        # Calculo de círculos
        # circles = cv.HoughCircles(img_gray,cv.HOUGH_GRADIENT_ALT,0.9,40,param1=50,param2=0.9,minRadius=4,maxRadius=0)
        # circles = cv.HoughCircles(img_gray,cv.HOUGH_GRADIENT,1,100,param1=200,param2=10,minRadius=1,maxRadius=10)
        circles = cv.HoughCircles(img_gray,cv.HOUGH_GRADIENT,1,100,param1=200,param2=8,minRadius=3,maxRadius=10)
        circles = np.uint16(np.around(circles))

        # Pintar elementos sobre la imagen
        for i in circles[0,:]:
            # draw the outer circle
            cv.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

        cv.imshow('circulos',cimg)
        cv.waitKey(0)
        cv.destroyAllWindows()

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
            # 5% de margen de la "desviación éstandar"
            for i in lista_dist:
                if (np.abs(i-mean)/mean > 0.05):
                    suma = suma + 1
            if (suma == 0):
                print("Pieza correcta")

                ## Comprobación de la orientación de la pieza ##
                recta_hist = []
                recta_prof_hist = []
                # Me quedo con un circulo y con el que tiene en la misma horizontal
                dist_x = np.array([np.abs(c[0][0]-c[lista[0]][0]), np.abs(c[0][0]-c[lista[1]][0])])
                max2 = np.argmax(dist_x)
                elem_horiz = lista[max2]
                #Calculo la recta entre los dos por mínimos cuadrados
                m = (c[elem_horiz][1] - c[0][1])/(c[elem_horiz][0] - c[0][0])

                # Creación de imagenes
                copy_img = np.copy(img)
                cimg_hsv = cv.cvtColor(img,cv.COLOR_RGB2HSV)
                # canny = cv.Canny(cimg_hsv[:,:,2], 50, 200)
                # canny_esquinas = np.zeros((1,abs(c[0][0] - c[elem_horiz][0]),1),np.uint8)
                img_prof_esquinas = np.zeros((1,abs(c[0][0] - c[elem_horiz][0]),1),np.uint8)

                if (c[0][0] - c[elem_horiz][0] < 0):
                    for x in range (c[0][0], c[elem_horiz][0]):
                        y = int(m*(x-c[0][0]) + c[0][1])
                        recta_hist.append([img[y,x,0], img[y,x,1], img[y,x,2]])
                        recta_prof_hist.append([img_prof[y,x]])
                        copy_img[y, x] = [0,255,0]
                        if (img_prof[y,x] != 0):
                            img_prof_esquinas [0,x-c[0][0]] = img_prof[y,x]
                else:
                    for x in range (c[elem_horiz][0], c[0][0]):
                        y = int(m*(x-c[0][0]) + c[0][1])
                        # print(x, y)
                        recta_hist.append([img[y,x,0], img[y,x,1], img[y,x,2]])
                        recta_prof_hist.append([img_prof[y,x]])
                        copy_img[y, x] = [0,255,0]
                        if (img_prof[y,x] != 0):
                            img_prof_esquinas [0,x-c[elem_horiz][0]] = img_prof[y,x]

                max_prof = np.amax(img_prof_esquinas)
                min_prof = np.amin(img_prof_esquinas)

                if (((max_prof-min_prof)/max_prof) > 0.06):
                    return "Orientacion y"
                else:
                    return "Orientacion x"

                # cv.imshow('img_prof',img_prof)
                # cv.imshow('img_2',copy_img)
                # plt.figure()
                # plt.imshow(copy_img)
                # plt.figure()
                # plt.imshow(img)
                # plt.figure()
                # plt.imshow(img_prof)
                # plt.figure()
                # plt.plot(np.array(recta_hist)[:,0], color="blue")
                # plt.plot(np.array(recta_hist)[:,1], color="green")
                # plt.plot(np.array(recta_hist)[:,2], color="red")
                # plt.figure()
                # plt.plot(np.array(recta_prof_hist)[:])
                # plt.show()
                # cv.waitKey(0)
                # cv.destroyAllWindows()

            else:
                return "Distancia anormal entre círculos"



        elif len(circulos) == 3:
            return "Falta 1 círculo"
        else:
            return "Faltan " + 4-len(circulos) + " circulos"


    except Exception as ex:
            return ex


if __name__ == '__main__':
    # img = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/superior_sd_Color.png')
    # img = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/superior_sd_1_Color.png')
    img = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/base230713_191407.png')
    img = cv.cvtColor(img,cv.COLOR_RGB2BGR)
    
    assert img is not None, "file could not be read, check with os.path.exists()"

    print("Llamado desde python")
    resultado = main(img)

    print(resultado)