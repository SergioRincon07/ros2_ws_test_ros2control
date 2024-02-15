[Enlace a GitHub](https://github.com/ros-controls/ros2_control_demos/blob/{REPOS_FILE_BRANCH}/example_1/doc/userdoc.rst)

# Ejemplo 1: RRBot

*RRBot*, o ''Robot Manipulador Revoluto-Revoluto'', es un brazo simple de 3 eslabones y 2 juntas que usaremos para demostrar varias características.

Es esencialmente un péndulo invertido doble y demuestra algunos conceptos de control interesantes dentro de un simulador y fue introducido originalmente para tutoriales de Gazebo.

Para *example_1*, el plugin de la interfaz de hardware se implementa teniendo solo una interfaz.

- La comunicación se realiza utilizando una API propietaria para comunicarse con la caja de control del robot.
- Los datos de todas las juntas se intercambian de una vez.
- Ejemplos: KUKA RSI

Los archivos URDF de *RRBot* se pueden encontrar en la carpeta ``description/urdf``.

## Pasos del tutorial

1. (Opcional) Para verificar que las descripciones de *RRBot* funcionan correctamente, usa los siguientes comandos de lanzamiento

   - **Local:**

     ```shell
     ros2 launch ros2_control_demo_example_1 view_robot.launch.py
     ```

   - **Docker:**

     Inicia el contenedor Docker ejecutando el siguiente comando:

     ```shell
     docker run -it --rm --name ros2_control_demos --net host ros2_control_demos ros2 launch ros2_control_demo_example_1 view_robot.launch.py gui:=false
     ```

     Ahora, necesitamos iniciar ``joint_state_publisher_gui`` así como ``rviz2`` para ver el robot, cada uno en sus propias terminales después de cargar nuestra instalación de ROS 2.

2. Para iniciar el ejemplo de *RRBot*, abre una terminal, carga tu espacio de trabajo de ROS2 y ejecuta su archivo de lanzamiento con

   - **Local:**

     ```shell
     ros2 launch ros2_control_demo_example_1 rrbot.launch.py
     ```

   - **Docker:**

     ```shell
     docker run -it --rm --name ros2_control_demos --net host ros2_control_demos ros2 launch ros2_control_demo_example_1 rrbot.launch.py gui:=false
     ```

   En la terminal de inicio, verás una gran cantidad de salida de la implementación de hardware mostrando sus estados internos.

3. Verifica si la interfaz de hardware se cargó correctamente, abriendo otra terminal y ejecutando

   - **Local:**

     ```shell
     ros2 control list_hardware_interfaces
     ```

   - **Docker:**

     ```shell
     docker exec -it ros2_control_demos ./entrypoint.sh bash
     ros2 control list_hardware_interfaces
     ```

4. Verifica si los controladores están en ejecución con

   - **Local:**

     ```shell
     ros2 control list_controllers
     ```

   - **Docker:**

     ```shell
     ros2 control list_controllers
     ```

5. Si obtienes una salida del paso anterior, puedes enviar comandos al *Controlador de Comando Directo*, ya sea:

   - Manualmente usando la interfaz de línea de comandos de ROS 2:

     - **Local:**

       ```shell
       ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
       - 0.5
       - 0.5"
       ```

     - **Docker:**

       ```shell
       ros2 topic pub /forward_position_controller/commands std_msgs/msg/Float64MultiArray "data:
       - 0.5
       - 0.5"
       ```

   - O puedes iniciar un nodo de demostración que envíe dos objetivos cada 5 segundos en un bucle:

     - **Local:**

       ```shell
       ros2 launch ros2_control_demo_example_1 test_forward_position_controller.launch.py
       ```

     - **Docker:**

       ```shell
       ros2 launch ros2_control_demo_example_1 test_forward_position_controller.launch.py
       ```

6. Cambiemos a un controlador diferente, el ``Controlador de Trayectoria de Articulación``. Carga el controlador manualmente con

   - **Local:**

     ```shell
     ros2 control load_controller joint_trajectory_position_controller
     ```

   - **Docker:**

     ```shell
     ros2 control load_controller joint_trajectory_position_controller
     ```

   Deberías obtener ``Controlador joint_trajectory_position_controller cargado correctamente``. Verifica el estado con

   - **Local:**

     ```shell
     ros2 control list_controllers
     ```

   - **Docker:**

     ```shell
     ros2 control list_controllers
     ```

   lo que te muestra que el controlador está cargado pero no configurado.

   Configure el controlador estableciéndolo en ``inactivo`` con

   - **Local:**

     ```shell
     ros2 control set_controller_state joint_trajectory_position_controller inactive
     ```

   - **Docker:**

     ```shell
     ros2 control set_controller_state joint_trajectory_position_controller inactive
     ```

   lo que debería dar ``Controlador joint_trajectory_position_controller configurado correctamente``.

   Como alternativa, puedes cargar el controlador directamente en estado ``inactivo`` mediante la opción para ``load_controller`` con

   - **Local:**

     ```shell
     ros2 control load_controller joint_trajectory_position_controller --set-state inactive
     ```

   - **Docker:**

     ```shell
     ros2 control load_controller joint_trajectory_position_controller --set-state inactive
     ```

   Deberías obtener el resultado ``Controlador joint_trajectory_position_controller cargado correctamente en estado inactivo``.

   Verifica si se cargó correctamente con

   - **Local:**

     ```shell
     ros2 control list_controllers
     ```

   - **Docker:**

     ```shell
     ros2 control list_controllers
     ```

   lo que ahora debería devolver:

   - **shell:**
      ``` shell
      joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
      forward_position_controller[forward_command_controller/ForwardCommandController] inactive
      joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] active
      ```


   Nota que el controlador está cargado pero aún ``inactivo``. Ahora puedes cambiar el controlador con

   - **Local:**

   ```shell
   ros2 control switch_controllers --activate joint_trajectory_position_controller --deactivate forward_position_controller
   ```

   - **Docker:**

   ```shell
   ros2 control switch_controllers --activate joint_trajectory_position_controller --deactivate forward_position_controller
   ```

   Nuevamente, verifica mediante

   - **Local:**

   ```shell
   ros2 control list_controllers
   ```

   - **Docker:**

   ```shell
   ros2 control list_controllers
   ```

   lo que ahora debería devolver
   - **shell:**

      ```shell
      joint_state_broadcaster[joint_state_broadcaster/JointStateBroadcaster] active
      forward_position_controller[forward_command_controller/ForwardCommandController] inactive
      joint_trajectory_position_controller[joint_trajectory_controller/JointTrajectoryController] active
      ```


   Envía un comando al controlador usando el nodo de demostración, que envía cuatro objetivos cada 6 segundos en un bucle con

   - **Local:**

   ```shell
   ros2 launch ros2_control_demo_example_1 test_joint_trajectory_controller.launch.py
   ```

   - **Docker:**

   ```shell
   ros2 launch ros2_control_demo_example_1 test_joint_trajectory_controller.launch.py
   ```

   ## Archivos utilizados para estas demostraciones

   - Archivo de lanzamiento: `rrbot.launch.py`
   - Controladores yaml: `rrbot_controllers.yaml`
   - Archivo URDF: `rrbot.urdf.xacro`

   - Descripción: `rrbot_description.urdf.xacro`
   - Etiqueta ``ros2_control``: `rrbot.ros2_control.xacro`

   - Configuración de RViz: `rrbot.rviz`
   - Configuración de objetivos de nodos de prueba:

   + `rrbot_forward_position_publisher`
   + `rrbot_joint_trajectory_publisher`

   - Plugin de interfaz de hardware: `rrbot.cpp`

   ## Controladores de esta demostración

   - ``Joint State Broadcaster`` ([repositorio de ros2_controllers](https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_state_broadcaster)): [documentación](https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_state_broadcaster/doc/userdoc.html)
   - ``Forward Command Controller`` ([repositorio de ros2_controllers](https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/forward_command_controller)): [documentación](https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/forward_command_controller/doc/userdoc.html)
   - ``Joint Trajectory Controller`` ([repositorio de ros2_controllers](https://github.com/ros-controls/ros2_controllers/tree/{REPOS_FILE_BRANCH}/joint_trajectory_controller)): [documentación](https://control.ros.org/{REPOS_FILE_BRANCH}/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html)
