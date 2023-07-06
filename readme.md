## Setup

- Downloading the repository
    ```sh
    cd <your_workspace>/src
    ```
    - for https:
        ```sh
        git clone https://github.com/atom-robotics-lab/Enord_drone.git
        ```
    - for ssh:
        ```sh
        git clone git@github.com:atom-robotics-lab/Enord_drone.git
        ```

- Adding Model in PX4 directory
    ```sh
    cd <your_workspace>/src/Enord_drone
    ```
    ```sh
    ./setup.bash
    ```
    **Note:** Makes sure you have px4-autopilot installed in your root directory. For px4-autopilot installation visit: [Ros wit mavros isntallation](https://docs.px4.io/main/en/ros/mavros_installation.html) 

- Installing dependencies and texture file:
    ```sh
    cd <your_workspace>/src/Enord_drone
    ```
    ```sh
    ./download_texture.bash
    ```
    
- Installing mavlink and mavros ros package:
    ```sh
    sudo apt install ros-noetic-mavros ros-noetic-mavlink
    ```
    
- Building the package:
    ```sh
    cd <your_workspace>
    ```
    ```sh
    catkin_make
    ```

## Usage

- Running the task:
    - Firstly launch the simulation launch file by:
        ```sh
        roslaunch drone_controller iris.launch
        ```
    - After that run the aruco_detection and offboard script by using controller launch file:
        ```sh
        roslaunch drone_controller controller.launch
        ```
        
- Additional commands you might need for testing puroses:
    - Launching only the airbase simulation world without iris models:
        ```sh
        roslaunch airbase_world airbase.launch
        ```
    - Launching drone_description files:
        ```sh
        roslaunch pushpak_description (controller/display/gazebo).launch
        ```
        
    - Running scirpts individually:
        - Running aruco_detector.py script:
            ```sh
            rosrun drone_controller aruco_detector.py
            ```
        - Running off_board.py script:
            ```sh
            rosrun drone_controller off_board.py
            ```
