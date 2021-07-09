# Package **soma_description**

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
    roslaunch soma_description simulator.launch
    ```

  - In second terminal :

    ```
    roslaunch soma_description controllers.launch
    ```
