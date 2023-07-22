import numpy as np
import cv2 as cv
import math

import matplotlib.pyplot as plt


def main(img_prof, xmax, ymax, xmin, ymin):
    # try:
    y_med = round((ymax + ymin)/2)

    ## Comprobación de la orientación de la pieza ##
    recta_hist = []
    esquinas = []
    acum_10 =[]

    z_anterior = img_prof[y_med,xmin]

    for x in range (xmin, xmax):
        z_act = img_prof[y_med,x]
        resta : int =  int(z_act) - int(z_anterior)
        print(z_act, z_anterior, resta)
        
        if (z_act != 0):
            if (len(acum_10) < 10):
                acum_10.append(resta)
            else:
                acum_10 = acum_10[1:] + acum_10[:1]
                acum_10[9] = resta
                print(acum_10)
                diff_acum = sum(acum_10)
                if (abs(diff_acum) > 15):
                    esquinas.append([(x-5), y_med, z_act])
                    z_anterior = z_act
                    acum_10 =[]
                    continue
            recta_hist.append([z_act])
            if (abs(resta) > 20):
                esquinas.append([x, y_med, z_act])
                acum_10 =[]
            z_anterior = z_act

    print(esquinas)

    plt.figure()
    plt.plot(recta_hist)
    plt.show()


    if (len(esquinas) == 2):
        return "Orientación y"
    elif (len(esquinas) == 0):
        return "Orientación x"
    else:
        return("No se ha podido determinar la orientación de la pieza")


    # except Exception as ex:
    #         return ex


if __name__ == '__main__':
    # img = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/superior_sd_Color.png')
    # img = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/defects/superior_sd_1_Color.png')
    img = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/base230713_191407.png')
    img = cv.cvtColor(img,cv.COLOR_RGB2BGR)
    
    assert img is not None, "file could not be read, check with os.path.exists()"

    resultado = main(img)

    print(resultado)