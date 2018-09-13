# SSF - Sensory Substitution Framework
SSF is a ROS (Robot Operating System) based Framework for Sensory Substitution

---
---

## Setup the project

### Clone repo to a folder

1. Clone the ROS project git (this git repo) into a folder (*NOTE: when/if editing the code, do it from this location*)
    - This folder can be placed where ever you would like to work from
    - We recommend the creating a folder on the desktop called ***git*** and cloning this repo into that folder
2. Build and run the project (follow the steps below)


### Setting up catkin workspace (used to build the project)

3. Create a folder called ***catkin_workspace***
    - This folder can be placed where ever you would like, so long as it's not a subfolder of ***ssf_package*** (i.e. so long as it's not part of the repo cloned earlier)
    - We recommend creating the folder on the desktop
4. Create a subfolder called ***src*** (i.e. ***catkin_workspace/src***)
5. Symlink the package (i.e. the folder called ***ssf_package***) to the ***catkin_workspace/src/*** directory. The easiest way to do this is:
    - Right click ***ssf_package***, select **Make Link**
    - A linked folder will be created (in the current directory) called ***Link to ssf_package***
    - Move the linked folder (***Link to ssf_package***) into ***catkin_workspace/src/***
    - Rename the linked folder to ***ssf_package*** (i.e. removing the "***Link to***" from the name)

### Building the project

6. Open a terminal to the ***catkin_workspace*** directory
7. Then from that terminal run <code>catkin build ssf_package</code>

## Running the project *locally*

*This launches the all the nodes of the ssf package*

1. Open a terminal to the ***catkin_workspace*** directory
2. Then from that terminal run <code>source devel/setup.bash</code>
3. Then from that terminal run <code>roslaunch ssf_package default.launch</code>

## **Alternative:** Running the project across *multiple devices*

***NOTE** for this example:*<br>
*- Both devices must be connected to the same network*<br>
*- Both devices should have a build version of the cloned repo*

### Below are the steps for the main device (i.e. the one running roscore)

1. Open a terminal (in any directory)
2. Then from that terminal run <code>roscore</code>
3. Open a *new* terminal to the ***catkin_workspace*** directory
4. Then from that terminal run <code>source devel/setup.bash</code>
5. Then from that terminal run <code>hostname -I</code>
    - This will return the **main device's IP address** (___.___.___.___)
6. Then from that terminal run <code>export ROS_IP=___.___.___.___</code> (filling in the IP address found in the previous step)
7. Then from that terminal run the nodes you would like to run
    - e.g. In the same terminal, run  <code>rosrun ssf_package melosee_retinal_encoder.py</code>

### Below are the steps for a secondary device

1. Open a *new* terminal to the ***catkin_workspace*** directory
2. Then from that terminal run <code>source devel/setup.bash</code>
3. Then from that terminal run <code>export ROS_MASTER_URI=http://___.___.___.___:11311</code> (filling in the **main device's IP address** found in the previous section)
4. Then from that terminal run the nodes you would like to run
    - e.g. In the same terminal, run  <code>rqt</code>