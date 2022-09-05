# TFM_AlbertoLosa_2122
Trabajo de Fin de Máster de Alberto Losa. Máster en Ingeniería Industrial 

## Creación urdf ur5e + pinza + Realsense d435i
Descarga del pkg de los modelos de los robots de Universal Robots y de pkg de Realsense
```
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git
git clone https://github.com/IntelRealSense/realsense-ros
```
Unimos el urdf del ur5e (ur5e_robot.urdf.xacro) que se encuentra en el pkg ur_e_description y el urdf de la d345i (test_d435i_camera.urdf.xacro) del pkg realsense2_description: **ur5e_robot_camera_v2.urdf.xacro** del pkg ur5e_cam_description.

Con MoveIt Setup Assistant realizamos el pkg ur5e_cam_moveit_config (documento de apoyo [aquí](https://ros-planning.github.io/moveit_tutorials/doc/setup_assistant/setup_assistant_tutorial.html))

### Creación de pkg para simulación de la cámara en Gazebo
Es necesario incorporar al modelo urdf un plugin que permita a la cámara Realsense d435i capturar información cuando se encuentra simulada en el entrono ROS con Gazebo.

Para ello, utilizamos el plugin creado por pal-robotics además de un pkg creado por Issaiass:
```
git clone https://github.com/pal-robotics/realsense_gazebo_plugin
git clone https://github.com/issaiass/realsense2_description.git
```
Se cambia el nombre al pkg creado por Issaias debido a que coincide con el nombre del propio pkg creado por Realsense.
El nuevo urdf que incorpora el plugin: **ur5e_robot_camera_v3.urdf.xacro** del pkg ur5e_cam_description.
Por último, con MoveIt Setup Assistant realizamos el pkg para simulación ur5e_cam_simulated_moveit_config.
