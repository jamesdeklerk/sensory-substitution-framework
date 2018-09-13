# SSF - Sensory Substitution Framework
SSF is a ROS (Robot Operating System) based Framework for Sensory Substitution


<br>

## Setup the project

**Clone repo to a folder**

1. Clone the ROS project git (this git repo) into a folder (*NOTE: when/if editing the code, do it from this location*)
    - This folder can be placed where ever you would like to work from
    - We recommend the creating a folder on the desktop called ***git*** and cloning this repo into that folder
2. Build and run the project (follow the steps below)


**Setup for building project**

3. Create a folder called ***catkin_workspace***
    - This folder can be placed where ever you would like, so long as it's not a subfolder of ***ssf_package*** (i.e. so long as it's not part of the repo cloned earlier)
    - We recommend creating the folder on the desktop
4. Create a subfolder called ***src*** (i.e. ***catkin_workspace/src***)
5. Symlink the package (i.e. the folder called ***ssf_package***) to the ***catkin_workspace/src/*** directory. The easiest way to do this is:
    - Right click ***ssf_package***, select **Make Link**
    - A linked folder will be created (in the current directory) called ***Link to ssf_package***
    - Move the linked folder (***Link to ssf_package***) into ***catkin_workspace/src/***
    - Rename the linked folder to ***ssf_package*** (i.e. removing the "***Link to***" from the name)

**Building the project**

6. Open the terminal to the ***catkin_workspace*** directory
7. Then in the terminal run <code>catkin build ssf_package</code>
<br>
<br>

## Running the project

*This launches the all the nodes of the ssf package*


1. Open the terminal to the ***catkin_workspace*** directory
2. Then in the terminal run <code>source devel/setup.bash</code>
3. Then in the terminal run <code>roslaunch ssf_package default.launch</code>