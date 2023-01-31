# C++ Nanodegree Capstone: ROS2 Tiago Explore 
This repository contains the submission code for the C++ Nanodegree Captstone project. I've chosen to use this project as a chance to deepen my ROS (Robot Operating System) skills. Three ROS nodes for perception, decision making and acting have been implemented as main part of this deliverable. The gazebo enviroment is utilized to simulate a Tiago robot model, communicating to the above mentioned ROS nodes. The [official ROS2 documentation](https://docs.ros.org/en/foxy/Tutorials.html) has been used as a starting point. Besides that, [A Concise Introduction to Robot Programming with ROS2](https://github.com/fmrico/book_ros2) was a great ressource to further study the topic. The screenshot down below shows the tiago robot model executing some action commands (published by the Act ROS node) within the gazebo simulation environment.    
![The tiago robot model follows some action commands in the gazebo simulation environment](tiago_gazebo_sample_image.JPG)


## Dependencies for Running Locally
* Ubuntu Linux 22.04 LTS    
* cmake >= 3.5
  * cmake might be pre installed on a Linux system   
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.2 
  * Linux: make is installed by default on most Linux distros
* gcc/g++ >= 9.4
  * Linux: gcc / g++ is installed by default on most Linux distros
 
 
## Basic Build Instructions
### Setup Sources
1. `sudo apt install software-properties-common`
2. `sudo add-apt-repository universe`
3. `sudo apt update && sudo apt install curl`
4. `sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`
5. `echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null`
### Install ROS2 Humble and corresponding development tools (see [this link](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) for the official documentation) 
1. `sudo apt update`
2. `sudo apt upgrade`
3. Please make sure that Ubuntu 22.04 LTS has been updated prior to the ROS2 humble installation (see troubleshooting section down below) 
4. `sudo apt install ros-humble-desktop`
5. `sudo apt install ros-dev-tools`
6. Activate ROS2 Foxy: `source /opt/ros/humble/setup.bash`
7. Optional: Default activation whenever a new terminal is opened:    
   `echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc`  
### Create ROS2 environment (in home directory), clone Github repo and import dependencies
1. `cd && mkdir -p ros2_ws/src && cd ros2_ws/src`
2. `git clone https://github.com/dschmoeller/ROS2TiagoCapstoneProject.git`  
3. `vcs import . < ROS2TiagoCapstoneProject/third_parties.repos`
4. `sudo rosdep init && rosdep update`
5. `cd ~/ros2_ws && rosdep install --from-paths src --ignore-src -r -y` 
### Build the project 
1. `cd ~/ros2_ws`
2. `colcon build --symlink-install`
3. Optional: Activate the workspace as overlay by default, whenever a new terminal is opened:    
   `echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc`  
### Run tiago simulation environment (in gazebo)  
1. Open a new terminal (Terminal 1) 
2. `source /opt/ros/humble/setup.bash`
3. `source ~/ros2_ws/install/setup.bash`
4. `ros2 launch br2_tiago sim.launch.py` 
### Run tiago explore code   
1. Open a new terminal (Terminal 2) 
2. `source /opt/ros/humble/setup.bash`
3. `source ~/ros2_ws/install/setup.bash`
4. `ros2 launch tiago_explore explore.launch.py` 
### Troubleshooting
- The ROS2 humble installation might fail, if Ubuntu 22.04 LTS hasn't been updated. 
- Due to early updates in Ubuntu 22.04 it is important that systemd and udev-related packages are updated before installing ROS 2. The installation of ROS 2’s dependencies on a freshly installed system without upgrading can trigger the removal of critical system packages. ([See official documentation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html))
- The colcon build might terminate with some warnings.
- When launching gazebo for the first time, it might take some additional time before everything is properly set up. It can be required to relaunch gazebo, if the communication to the explore nodes doesn't work. 



## Overview Code Structure
![Three node structure for solving arbitrary see think act problems](see_think_act_node_architecture.JPG)
The purpose of this project was to create a starting point for more sophisticated robot applications. The above picture shows the implemented ROS architecture, comprising the three nodes See, Think and Act. This architecture enables a separation of concern, such that the robot's perception does not depend on the its decision making and so forth. This means, that if one desires to change the robot's decision making policy, code only has to be changed within the Think module. Further, one could more easily exchange one of these three nodes by other implementations. It only has to be guaranteed that the nodes adhere to the respective interfaces (i.e. the ROS topics). The publish/subscribe paradigm has been implemented for node communication. The See node listens to any laser sensor information and stores the respective messages internally. It extracts relevant information, which is three distance measurements, in front of the robot and on its left and right side. These three distance values are stored and published in a dedicated world model message. Notice how one could easily extend the robot's world representation by connecting additional sensor modalities. Further, notice how the See node uses an internal timer to publish its information. This has been done in order to disconect the See node from the external laser signal publish frequency. The Think node subscribes to a world model and uses this information for the robot's decision making. The implemented policy is very simple, the robot always turns right if there are any obstacles close in front of it or on its left. The Act node serves as driver and translates high level robot commands into actual executable format, such as translational or rotational velocity information. The tiago robot model (in the gazebo simulation environment) has been connected to the above ROS nodes. A launch file has been implemented in order to start up all three nodes with their corresponding configurations compfortably with only one command. All three nodes are created and executed from the main file. Further, notice how remapping has been utilized to connect the tiago specific topics with the See node and the Think node.       


## Necessary Rubric Points  
### Rubric Point 1: The project demonstrates an understanding of C++ functions and control structures:   
  - if/else control structures have been used, e.g. in the Think::think_policy() method
  - ROS implicitly forces code to be organized into functions, e.g. the callback functions for publish/subscribe communication   
### Rubric Point 2: The project uses Object Oriented Programming techniques:   
  - Three classes (See, Think, Act) have been implented with corresponding attributes and methods   
### Rubric Point 3: Classes use appropriate access specifiers for class members:   
  - All class data members are explicitly specified as public, protected, or private in See.hpp, Think.hpp and Act.hpp
### Rubric Point 4: The project uses move semantics to move data, instead of copying it, where possible:   
  - The See::lidar_callback() method uses move semantics in order to pass incoming laser scan messages into a unique pointer, rather than making an expensive copy 
### Rubric Point 5: The project uses smart pointers instead of raw pointers:   
  - The publisher, subscriber and other attributes in See.hpp are implemented as shared and unique pointers


  
     

