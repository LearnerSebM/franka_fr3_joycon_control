## How to Clone and Configure This Repo

1. Install ROS2 Development Environment (If Not Installed)  

We recommend installing ros2 desktop for beginners: 
~~~
sudo apt install ros-humble-desktop
~~~
Also install development tools:
~~~
sudo apt install ros-dev-tools
~~~
Then init ros dev tools:
~~~
sudo rosdep init
rosdep update
~~~

2. Set Up a Workspace and Clone the Repo
~~~
mkdir -p franka_ros2_ws/src
cd franka_ros2_ws
git clone https://github.com/LearnerSebM/franka_fr3_joycon_control src
~~~

3. Initialize and Update Submodules
~~~
cd src
git submodule update --init --recursive
~~~
This command will load tag v2.0.4 in the *franka_ros2* repo and commit #c2cf050 in the *joycon* repo as submodules.

4. Install the Dependencies in the franka_ros2 submodule
~~~
cd franka_ros2
vcs import . < ./franka.repos --recursive --skip-existing
~~~

5. Detect and Install Project Dependencies of *franka_ros2* Submodule (If Never Developed on *franka_ros2*)
~~~
rosdep install --from-paths . --ignore-src --rosdistro humble -y
~~~

6. Create a New Virtual Environment for *joycon* SDK

Please make sure a proper version of conda is installed (for instance, *mini-forge*) in advance.
~~~
cd ../joycon-sdk
conda create -n joycon python=3.10
~~~
Then activate the virtual environment and install the required dependencies
~~~
conda activate joycon
pip install -e .
conda deactivate
~~~

7. Make Install the Joycon SDK
~~~
sudo apt-get update
sudo apt-get install -y dkms libevdev-dev libudev-dev cmake git
make install
~~~

8. Build the Whole Project

If building for the first time:
~~~
cd ../..  # cd to workspace path
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-skip joycon-robotics
~~~
Otherwise just use normal colcon build.


## How to Use this Repo

Please refer to the user handbook in the *joycon* repo to set up the joycon controller.

Open two integrated terminals in *VS Code* or *Cursor*.  
In the first terminal, cd to workspace path and launch the joycon:
~~~
source install/setup.bash
conda activate joycon
ros2 launch joycon_wrapper joycon.launch.py namespace:=/NS_1
~~~
The **namespace** parameter of launch command should match the namespace specified in the /src/joycon_control/joycon_control_bringup/config/franka.config.yaml, which in this case is "/NS_1"

In the second terminal, launch the ros2 controller after the first terminal reads "Joycon publisher node started, publishing frequency: 1000 Hz". There's no need to activate the joycon virtual environment in this terminal.  
cd to workspace path and then use following commands:
~~~
source install/setup.bash
ros2 launch joycon_control_bringup joycon_ik_controller.launch.py
~~~

The robot will reset its position first and then receive command from joycon. 