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

- Downloading texture file for airbase:
    ```sh
    cd <your_workspace>/src/enord_drone
    ```
    ```sh
    ./download_texture.bash
    ```
- building the package:
    ```sh
    cd <your_workspace>
    ```
    ```sh
    catkin_make
    ```

## Usage

- Running sim world:
    ```sh
    roslaunch airbase_world airbase.launch
    ```
- Running drone_description files:
    ```sh
    roslaunch pushpak_description (controller/display/gazebo).launch
    ```
