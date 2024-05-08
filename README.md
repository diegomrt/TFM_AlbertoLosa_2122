# TFM_AlbertoLosa_2122
Trabajo de Fin de Máster de Alberto Losa. Máster en Ingeniería Industrial 

Para la simulación de todos los procesos posteriores es necesario clonar este repositorio como se indica a continuación:
```
git clone https://github.com/diegomrt/TFM_AlbertoLosa_2122.git TFM_AlbertoLosa_2122
```

## Creación urdf para simulación ur5e + pinza + Realsense d435i
Comenzamos descargando el pkg del modelo de los robots de Universal Robots:
```
git clone -b melodic-devel-staging https://github.com/ros-industrial/universal_robot.git
```
Es necesario incorporar al modelo urdf un plugin que permita a la cámara Realsense d435i capturar información cuando se encuentra simulada en el entrono ROS con Gazebo.

Para ello, utilizamos el plugin creado por pal-robotics además de un pkg de los modelos de las cámaras de RealSense creado por Issaiass:
```
git clone https://github.com/pal-robotics/realsense_gazebo_plugin
git clone https://github.com/issaiass/realsense2_description.git realsense2_description_issaias
```
Ahora debemos acudir al pkg creado como realsense_description_issaias y en urdf/_d435.urdf.xacro cambiar las líneas en las que se hace referencia al pkg realsense2_description y referenciarlo a realsense2_description_issaias.También se debe cambiar el nombre del pkg en el CMakeList.txt y en el package.xml.

Se crea un urdf que incorpora tanto la descripción del robot (ur5e) como el modelo de la cámara de RealSense con el plugin para la simulación en gazebo: **ur5e_robot_camera_sim.urdf.xacro** del pkg ur5e_cam_description.

Con MoveIt Setup Assistant realizamos el pkg **ur5e_cam_sim_moveit_config** (documento de apoyo [aquí](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html))

Este pkg ya se ha realizado y se encuentra en la carpeta TFM_AlbertoLosa_2122 al clonar este repositorio según se describe en el primer paso de este Readme.

Para la simulación de estos dos componentes es necesario lanzar el archivo demo_gazebo.launch de este pkg de la siguiente forma:
```
roslaunch ur5e_cam_sim_moveit_config demo_gazebo.launch
```


### Adición de vaccum grippers
Para usar estos grippers usamos el plugin de ["Vacuum Gripper" de Gazebo](https://docs.ros.org/en/noetic/api/gazebo_plugins/html/group__GazeboRosVacuumGripper.html). Aunque es necesario poner más de una, ya que no tiene la suficiente fuerza como para levantar objetos en simulación.

El urdf que contiene la definición de las pinzas es **grippers.urdf.xacro** del pkg ur5e_cam_description. Este urdf lo unimos al urdf del robot y la cámara en **ur5e_robot_camera-grippers_sim.urdf.xacro** del mismo pkg.

Otra vez, con MoveIt Setup Assistant realizamos el pkg para simulación **ur5e_cam-grippers_simulated_moveit_config**.

Para la simulación es necesario lanzar el archivo demo_gazebo.launch de este pkg de la siguiente forma:
```
roslaunch ur5e_cam-grippers_simulated_moveit_config demo_gazebo.launch
```
Para actuar sobre la pinza de vacío, el plugin un servicio llamado /ur5e/vacuum_gripper/grasping/on (para encender el vacío) y /ur5e/vacuum_gripper/grasping/off (para soltar el objeto). Para llamar a estos servicios ultilizamos las siguientes líneas de código:
```
rosservice call /ur5/vacuum_gripper/on "{}" 

rosservice call /ur5/vacuum_gripper/off "{}" 
```
El estado de la pinza se actualiza en el topic /ur5e/vacuum_gripper/grasping.

Además, se crea un scrip de python que actua sobre todas las pinzas de vacío que se incorporan (se introducen 8 debido a que una no tiene la fuerza suficiente como para sujetar un objeto) que se encuentra en ur5e_cam_description/scripts llamado **ur5_grippers.py**.

## Creación urdf para robot real ur5e + Pinza OnRobot VGC10 + Realsense d435i
Para la creación y el uso de este urdf para el robot real que incorpora tanto la pinza VGC10 de OnRobot y la cámara real de Realsense d435i es necesario instalar los siguientes pkgs:

Primero instalaremos el pkg que nos ofrece la Universidad de Osaka, debemos clonarlo en el src de nuestro workspace ejecutando el siguiente comando:
```
git clone https://github.com/Osaka-University-Harada-Laboratory/onrobot.git
```
En segundo lugar, se instala las herramientas oficiales que nos ofrece Intel para el control de sus cámaras Realsense siguiendo los pasos del [repositorio oficial de Intel](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy). El comado que se recomienda usar para instalarlo es:
```
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```

Con esto, se realiza el urdf que se utiliza para representar al robot real llamado **ur5e_robot_camera-grippers_real.urdf.xacro** del pkg ur5e_cam_description.

Con ello, se genera el pkg de configuración de MoveIt llamado: **ur5e_cam-grippers_real_moveit_config**.

## Conexión con el robot ur5e
Para la conexión con el robot real, seguimos las instrucciones descritas el el [repositorio oficial de UniversalRobots](https://github.com/UniversalRobots/Universal_Robots_ROS_Driver).
Instalamos este repositorio:
```
git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git
sudo apt update -qq
rosdep update
rosdep install --from-paths src --ignore-src -y
```

Es necesario instalar también el pkg de ur_msgs:
```
git clone https://github.com/ros-industrial/ur_msgs.git
```

Para controlar el robot con MoveIt hay que lanzar los siguientes .launch:
```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.10
roslaunch ur5e_cam-grippers_real_moveit_config demo_real.launch
```

Lanzamos la cámara (con alineamiento de la cámara de profundidad con la visibley nube de puntos) con:
```
roslaunch realsense2_camera rs_camera.launch align_depth:=true filters:=pointcloud
```

## USO DE YOLO
Se hace uso del repositorio creado por M. Bjelonic "YOLO ROS: Real-Time Object Detection for ROS", URL: https://github.com/leggedrobotics/darknet_ros, 2018.

E instalamos el pkg y compilamos con:
```
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Por último, se realiza un test de la instalación:
```
catkin build darknet_ros --no-deps --verbose --catkin-make-args run_tests
```

## ARRANQUE DEL ROBOT
Para ver la detección en funcionamiento es necesario lanzar los siguientes paquetes:

#### Para simulación:
Para comenzar se lanza el ur5e con MoveIt, Gazebo y Rviz:

```
roslaunch ur5e_cam-grippers_vgc10_simulated_moveit_config demo_gazebo.launch
```

Para que la detección tenga efecto es necesario colocar algún objeto en Gazebo (colocarlo justo debajo de la herramienta, ya que esa será la posición de visualización).

#### Para robot real:
Para el robot real es necesario lanzar el External Control:

```
roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.1.10
```

También hay que lanzar el nodo de la cámara Intel Realsense con los filtros seleccionados:

```
roslaunch realsense2_camera rs_camera.launch align_depth:=true filters:=pointcloud,hole_filling,temporal
```

Por último, se lanza MoveIt y Rviz:

```
roslaunch ur5e_cam-grippers_real_moveit_config demo_real.launch
```

## LANZAMIENTO DE LA APLICACIÓN

A continuación se explica los archivos a lanzar dependiendo de el caso del trabajo que se quiere realizar y se muestran unos videos de cada caso práctico.

### Caso práctico 1: 

En este caso practico se realiza pick and place de las manzanas y naranjas que se encuentren dentro de la imagen. Para ello, es necesario lanzar el archivo de python:

```
rosrun ur5e_cam_description states_machine_practice_caso1.py 
```

VIDEO

### Caso práctico 2: 

En este caso practico se realiza un estudio de los defectos de determinadas piezas, discretizadas mediante CNN. Para ello, es necesario lanzar el archivo de python:

```
rosrun ur5e_cam_description states_machine_practice_caso2.py 
```

VIDEO
