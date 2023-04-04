import IK_moveJ
from math import pi

x, y, z = IK_moveJ.main_euler_tcp(pi, pi/2, 0, 0, 0.5, 0.2)  ### Para ir con la cámara enfocando la pieza lateralmente

print(x, y, z)

# IK_moveJ.main_euler(pi, pi/2, 0, 0, 0.5, 0.3)  ### Para ir con la cámara enfocando la pieza lateralmente