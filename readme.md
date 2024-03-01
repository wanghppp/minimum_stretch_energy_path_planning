# Minimum Stretch Energy Path Planning
## Result
![](result/result.gif)

## How to run
1.  Create your ROS workspace.
    ```
    mkdir  catkin_ws
    cd catkin_ws
    mkdir src
    catkin_make
    ```
2.  Put the package **gcoptr** and **map_gen** to src file.
3.  Compile the workspace.
    ```
    catkin_make
    ```
4.  RUN.
    ```
    source devel/setup.bash
    roslaunch gcopter curve_gen.launch
    ```
## Note
1.  All programs should run under **ROS(Robot Operating System)**.
2.  All programs are tested in **Ubuntu 18.04**, it can work well.