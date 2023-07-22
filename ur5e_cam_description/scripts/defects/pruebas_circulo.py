import numpy as np
import cv2 as cv

import matplotlib.pyplot as plt

from scipy import ndimage as ndi

import skimage

cimg = cv.imread('/home/alberto/tfm_ws/src/TFM_AlbertoLosa_2122/ur5e_cam_description/scripts/img_reales/base_lejos.png')
# cimg = cv.cvtColor(cimg,cv.COLOR_BGR2RGB)
# cimg = cv.cvtColor(cimg,cv.COLOR_BGR2HSV)

prueba = "circulos"


assert cimg is not None, "file could not be read, check with os.path.exists()"

if (prueba == "segmentación_local"):

    cimg_gray = cv.cvtColor(cimg,cv.COLOR_RGB2GRAY)

    # cimg = cv.medianBlur(cimg,11)

    image = skimage.util.img_as_float(cimg_gray)

    thresh_niblack = skimage.filters.threshold_niblack(image, window_size=5, k=1.5)
    thresh_sauvola = skimage.filters.threshold_sauvola(image, window_size=5)

    binary_niblack = image > thresh_niblack
    binary_sauvola = image > thresh_sauvola


    # cimg_hsv = cv.cvtColor(img,cv.COLOR_RGB2HSV)
    canny = cv.Canny(cimg[:,:,1], 10, 30, apertureSize = 3, L2gradient = True)

    plt.figure()
    plt.imshow(binary_niblack, cmap=plt.cm.gray)
    plt.figure()
    plt.imshow(binary_sauvola, cmap=plt.cm.gray)
    plt.figure()
    plt.imshow(cimg_gray)
    plt.show()

    cv.imshow('detected circles',canny)
    cv.waitKey(0)
    cv.destroyAllWindows()

    # denoise image
    denoised = skimage.filters.rank.median(image, skimage.morphology.disk(2))

    # find continuous region (low gradient -
    # where less than 10 for this image) --> markers
    # disk(5) is used here to get a more smooth image
    markers = skimage.filters.rank.gradient(denoised, skimage.morphology.disk(5)) < 10
    markers = ndi.label(markers)[0]

    # local gradient (disk(2) is used to keep edges thin)
    gradient = skimage.filters.rank.gradient(denoised, skimage.morphology.disk(2))

    # process the watershed
    labels = skimage.segmentation.watershed(gradient, markers)

    # display results
    fig, axes = plt.subplots(nrows=2, ncols=2, figsize=(8, 8),
                            sharex=True, sharey=True)
    ax = axes.ravel()

    ax[0].imshow(image, cmap=plt.cm.gray)
    ax[0].set_title("Original")

    ax[1].imshow(gradient, cmap=plt.cm.nipy_spectral)
    ax[1].set_title("Local Gradient")

    ax[2].imshow(markers, cmap=plt.cm.nipy_spectral)
    ax[2].set_title("Markers")

    ax[3].imshow(image, cmap=plt.cm.gray)
    ax[3].imshow(labels, cmap=plt.cm.nipy_spectral, alpha=.5)
    ax[3].set_title("Segmented")

    for a in ax:
        a.axis('off')

    fig.tight_layout()
    plt.show()

elif (prueba == "circulos"):
    img_gray = cv.cvtColor(cimg,cv.COLOR_RGB2GRAY)

    # Calculo de círculos
    # circles = cv.HoughCircles(img_gray,cv.HOUGH_GRADIENT_ALT,0.9,40,param1=50,param2=0.9,minRadius=4,maxRadius=0)
    circles = cv.HoughCircles(img_gray,cv.HOUGH_GRADIENT,1,100,param1=200,param2=5,minRadius=1,maxRadius=10)
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