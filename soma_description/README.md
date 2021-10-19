# Package **soma_description**

Robot model, visualization in Rviz, simulation in Gazebo, driving with rqt_robot_steering GUI

![robot model in Gazebo](../images/robot_model.png)

## Robot simulation

- Only Rivz

  ```
  roslaunch soma_description rviz.launch
  ```

- Only Gazebo

  - In first terminal :

    ```
    roslaunch soma_description gazebo.launch
    ```

  - In second terminal :

    ```
    roslaunch soma_description controllers.launch
    ```

- Both

  - In first terminal :

    ```
    roslaunch soma_description simulation.launch
    ```

  - In second terminal :

    ```
    roslaunch soma_description controllers.launch
    ```
