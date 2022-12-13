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

## USO DE YOLO
Se hace uso del repositorio creado por M. Bjelonic "YOLO ROS: Real-Time Object Detection for ROS", URL: https://github.com/leggedrobotics/darknet_ros, 2018.

Si se dispone de una GPU Nvidia compatible con CUDA, el procesamiento será notablemente más rápido (más información en el link del repositorio). 

Primero hay que instalar los drivers de GPU Nvidia:

**Esta parte a continuación no se ha podido realizar de manera satisfactoria. Para continuar usando Yolo con la CPU (notablemente más lento ir a *1)**
```
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt install nvidia-driver-440
sudo reboot     --> CUIDADO! REINICIAMOS EL SISTEMA	
nvidia-smi
```

Para instalar CUDA se especifica la configuración y se dan las instrucciones para la instalación [aquí](https://developer.nvidia.com/cuda-toolkit) (en mi caso he instalado la versión 11.8 ya que las anteriores dan problemas):
```
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2004/x86_64/cuda-ubuntu2004.pin
sudo mv cuda-ubuntu2004.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/11.8.0/local_installers/cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
sudo dpkg -i cuda-repo-ubuntu2004-11-8-local_11.8.0-520.61.05-1_amd64.deb
sudo cp /var/cuda-repo-ubuntu2004-11-8-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda
```

En mi caso sale un error, por lo tanto hay que hacer:
```
sudo apt-get install aptitude
sudo aptitude install cuda
```
***1**

E instalamos el pkg y compilamos con:
```
git clone --recursive https://github.com/leggedrobotics/darknet_ros.git
catkin_make -DCMAKE_BUILD_TYPE=Release
```

Por último, se realiza un test de la instalación:
```
catkin build darknet_ros --no-deps --verbose --catkin-make-args run_tests
```
(En mi caso no he podido porque he tenido que hacer la compilación con catkin_make)

## MÁQUINA DE ESTADOS
Para ver la detección en funcionamiento es necesario lanzar los siguientes paquetes:

Para comenzar se lanza el ur5e con MoveIt, Gazebo y Rviz:

```
roslaunch ur5e_cam-grippers_simulated_moveit_config demo_gazebo.launch
```

Cuando se haya inicializado se laza el paquete Python que contiene la máquina de estados:
```
rosrun ur5e_cam_description states_machine_simulation.py
```

Para que la detección tenga efecto es necesario colocar algún objeto en Gazebo (colocarlo justo debajo de la herramienta, ya que esa será la posición de visualización).