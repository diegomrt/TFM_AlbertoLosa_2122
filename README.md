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
