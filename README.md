# rviz_waypoint_navigator
GUI interface for waypoint_navigator using Rviz.

## Installation

```bash
sudo apt-get install python-wstool python-catkin-tools ros-indigo-cmake-modules
```

Recommended to setup a clean catkin workspace

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
catkin config --extend /opt/ros/indigo
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel
```

We use wstool to install dependencies and keep track of versions.
```
cd src
wstool init
wstool set --git waypoint_navigator git@github.com:ToniRV/rviz_waypoint_navigator.git -y
wstool update
wstool merge rviz_waypoint_navigator/install/gazebo_rotors_simulation.rosinstall
wstool update -j8
```

Source your newly created workspace
```
source ~/catkin_ws/devel/setup.bash
```

Alternatively, add this command to your bashrc to avoid having to source your workspace for each new terminal.
```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Finally, build the workspace:
```
catkin build
```

## Usage

1. In a new terminal (remember to source the workspace), type:

 ```
 $ roslaunch waypoint_navigator mav_sim.launch
 ```
 
3. In a new command window, type:

 ```
 $ roslaunch rviz_waypoint_navigator rviz_waypoint_navigator.launch
 ```

4. Launch RVIZ using the configuration in the ``rviz`` folder:
```
rviz -d ~/catkin_ws/src/rviz_waypoint_navigator/rviz/rviz_waypoint_navigator.rviz
```

Now you should see two screens:
a) Gazebo with the drone
b) RVIZ with a drawn drone with interactive markers shaped as a frame of reference.
You can click and drag along these arrows to move the drawn drone around.
Then, right click on it: a drop-menu should appear.
Click on ``Store pose as waypoint``.
Repeat as long as you want.
Then, right click again to see the menu and click ``Plan path through stored waypoints``.
You should see the drone moving.

See Issues section below if you encounter problems.

## Simulation
  ### Gazebo + RotorS

## Issues
If you do not see the drawn drone in Rviz, reset the InteractiveMarkers topic at the left side panel in Rviz by clicking on ``Update Topic`` and clicking to the topic which should be ``/firefly/rviz_waypoint_navigator/update``.
