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
git clone https://github.com/issaiass/realsense2_description.git
```
Se crea un urdf que incorpora tanto la descripción del robot (ur5e) como el modelo de la cámara de RealSense con el plugin para la simulación en gazebo: **ur5e_robot_camera_sim.urdf.xacro** del pkg ur5e_cam_description.

Con MoveIt Setup Assistant realizamos el pkg ur5e_cam_sim_moveit_config (documento de apoyo [aquí](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html))

Este pkg ya se ha realizado y se encuentra en la carpeta TFM_AlbertoLosa_2122 al clonar este repositorio según se describe en el primer paso de este Readme.

Para la simulación de estos dos componentes es necesario lanzar el archivo demo_gazebo.launch de este pkg de la siguiente forma:
```
roslaunch ur5e_cam_sim_moveit_config demo_gazebo.launch
```


### Adición de vaccum grippers
Para usar estos grippers usamos el plugin de ["Vacuum Gripper" de Gazebo](https://docs.ros.org/en/noetic/api/gazebo_plugins/html/group__GazeboRosVacuumGripper.html). Aunque es necesario poner más de una, ya que no tiene la suficiente fuerza como para levantar objetos en simulación.

El urdf que contiene la definición de las pinzas es **grippers.urdf.xacro** del pkg ur5e_cam_description. Este urdf lo unimos al urdf del robot y la cámara en **ur5e_robot_camera-grippers_sim.urdf.xacro** del mismo pkg.

Otra vez, con MoveIt Setup Assistant realizamos el pkg para simulación ur5e_cam-grippers_sim_moveit_config.

Para la simulación es necesario lanzar el archivo demo_gazebo.launch de este pkg de la siguiente forma:
```
roslaunch ur5e_cam-grippers_sim_moveit_config demo_gazebo.launch
```
Para actuar sobre la pinza de vacío, el plugin un servicio llamado /ur5e/vacuum_gripper/grasping/on (para encender el vacío) y /ur5e/vacuum_gripper/grasping/off (para soltar el objeto). Para llamar a estos servicios ultilizamos las siguientes líneas de código:
```
rosservice call /ur5/vacuum_gripper/on "{}" 

rosservice call /ur5/vacuum_gripper/off "{}" 
```
El estado de la pinza se actualiza en el topic /ur5e/vacuum_gripper/grasping.

Problema actual: al hacer el call para activar la pinza de vacío, el estado en el topic no se actualiza (siempre en false).

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

# USO DE YOLO
Se hace uso del repositorio creado por M. Bjelonic "YOLO ROS: Real-Time Object Detection for ROS", URL: https://github.com/leggedrobotics/darknet_ros, 2018.

Si se dispone de una GPU Nvidia compatible con CUDA (con este software instalado), el procesamiento será notablemente más rápido (más información en el link del repositorio). 

Para instalar CUDA:
```
sudo apt-get update
sudo apt-get -y install nvidia-cuda-toolkit
```

E instalamos el pkg y compilamos con:
```
git clone https://github.com/leggedrobotics/darknet_ros.git
catkin_make -DCMAKE_BUILD_TYPE=Release
```

El catkin make 