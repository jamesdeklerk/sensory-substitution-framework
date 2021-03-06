# SSF - Sensory Substitution Framework
SSF is a ROS (Robot Operating System) based Framework for Sensory Substitution

**NOTE:** anything in < > should be replaced appropriately, e.g. "<file_name>.py" might become "my_script.py"

---

## Setup the project

#### Clone repo to a folder

1. Clone the ROS project git (this git repo) into a folder (*NOTE: when/if editing the code, do it from this location*)
    - This folder can be placed where ever you would like to work from
    - We recommend the creating a folder on the desktop called ***git*** and cloning this repo into that folder
2. Build and run the project (follow the steps below)


#### Setting up catkin workspace (used to build the project)

3. Create a folder called ***catkin_workspace***
    - This folder can be placed where ever you would like, so long as it's not a subfolder of ***ssf_package*** (i.e. so long as it's not part of the repo cloned earlier)
    - We recommend creating the folder on the desktop
4. Create a subfolder called ***src*** (i.e. ***catkin_workspace/src***)
5. Symlink the package (i.e. the folder called ***ssf_package***) to the ***catkin_workspace/src/*** directory.
    - The easiest way to do this is:
        - Right click ***ssf_package***, select **Make Link**
        - A linked folder will be created (in the current directory) called ***Link to ssf_package***
        - Move the linked folder (***Link to ssf_package***) into ***catkin_workspace/src/***
        - Rename the linked folder to ***ssf_package*** (i.e. removing the "***Link to***" from the name)
    - Or one can use the terminal command:
        - <code>ln -s ~/Desktop/code/ssf_package/ ~/Desktop/catkin_workspace/src/</code> (to create a symlink at ~/Desktop/catkin_workspace/src/ referencing the original folder ~/Desktop/code/ssf_package/)

#### <a name="building_the_project"></a>Building the project

6. Open a *new* terminal in the ***catkin_workspace*** directory
7. Then from that terminal run <code>catkin build</code> (if catkin is not installed run <code>sudo apt-get install python-catkin-tools</code>)

#### OPTIONAL: Add the source setup.bash command to your ~/.bashrc file

*The following is to add the source setup.bash command to your ~/.bashrc file, so that it will be executed every time that you open a new shell. Using this means one won't have to enter <code>source devel/setup.bash</code> every time.*

1. Open a *new* terminal in the ***catkin_workspace*** directory
2. Then from that terminal run <code>echo "source ~/Desktop/catkin_workspace/devel/setup.bash" >> ~/.bashrc</code> (assuming your workspace is on the desktop in the folder catkin_workspace)
3. Then from that terminal run <code>source ~/.bashrc</code>


---

## <a name="running_the_project_locally"></a>Running the project *locally*

*This launches the all the nodes of the ssf package*

1. Open a *new* terminal in the ***catkin_workspace*** directory
2. Then from that terminal run <code>source devel/setup.bash</code>
3. Then from that terminal run <code>roslaunch ssf_package default.launch</code>

---

## Recording & Playing Back Sessions

*This uses the standard ROS record and play features of [**rosbag**](http://wiki.ros.org/rosbag/Commandline)*<br>
***rosbag** is a set of tools for recording from and playing back to ROS topics*

#### <a name="playback"></a>Playback (of a ROS .bag file)
1. Download a test recording (i.e. a *.bag* file) to the directory of your choice
    - Test recordings can be found [here](https://www.google.com/search?q=Searching+for+test+.bag+recordings...Sorry%2C+I+still+need+upload+those...)
2. Open a *new* terminal in the directory used above
    - This directory should now contain the *.bag* file
3. Then from that terminal run <code>rosbag play -l example.bag</code> (assuming the recordings name was example.bag)
    - If you don't want the playback to loop, simply run <code>rosbag play example.bag</code>

#### Creating a recording (which is saved to a .bag file)
*This is section describes how to make a recording of currently running ROS topics*

1. Open a *new* terminal in the directory you would like to save the recording
2. To get a list of the avalable ROS topics:
    - In the same terminal, run <code>rostopic list</code>
3. To start recording the topics, in the same terminal, run <code>rosbag record topic_name_x topic_name_y topic_name_etc</code>
    - You can record as many topics as you like, in the above example, the data from 3 topics are being recorded, topics topic_name_x topic_name_y topic_name_etc
    - For example, if you wanted to record the depth and rgb info from a RealSense D435 or D415 (for which the relevant topics are: */camera/depth/image_ct_raw* and */camera/color/image_raw*), you would run <code>rosbag record /processed_color_image /processed_depth_image</code>
4. To stop the recording press **ctrl + c**
    - The recording of the chosen topics will be saved in a .bag file in the current directory, with the file name being a timestamp (e.g. *2018-09-13-23-59-01.bag*).

***

## **Advanced Alternative:** Running the project across *multiple devices*

***NOTE** for this example:*
- *Both devices must be connected to the same network*
    - OPTIONAL: For the fastest wireless connection, make one of the machines host a network (a hostednetwork), and the other connect to that network.
        - *NB: the master node must be run from the client machine NOT the host machine, hence:*
            - For the client machine (the one connected to hosted network), follow the steps below for the ***main device***
            - For the host machine (the one hosting the network), follow the steps below for the ***secondary device***
        - If you are using VMware (if not, ignore this step):
            - For the client machine make sure the Network Adapter in the virtual machines settings is set to Bridged: Connected directly to the physical network
            - For the host machine make sure the Network Adapter in the virtual machines settings is set to NAT: Used to share the host's IP address
    - NB: If you are using VMware, but aren't using a network hosted by one of the machines, then make sure the Network Adapter's for all the virtual machines settings are set to Bridged: Connected directly to the physical network
- Both devices should have a built version of the cloned repo*

#### Below are the steps for the *main device* (i.e. the one running roscore)

1. Open a *new* terminal in the ***catkin_workspace*** directory
2. Then from that terminal run <code>source devel/setup.bash</code>
3. Then from that terminal run <code>hostname -I</code> (NB: capital I)
    - This will return the **main device's IP address** (___.___.___.___)
4. Then from that terminal run <code>export ROS_IP=___.___.___.___</code> (filling in the IP address found in the previous step)
5. Then from that terminal run <code>roscore</code>
6. Then from that terminal run the node you would like to run
    - e.g. In the same terminal, run <code>roslaunch ssf_package evaluation.launch</code> (recommended)
7. If you want access to a nodes output, when launching that node, you should perform steps 1. 2. 4. 6.
    - e.g. For 6. try running <code>rosbag play -l example.bag</code> (recommended) *NOTE: example.bag will need to be downloaded to the **catkin_workspace** for this to run, see [this](#playback)*

#### Below are the steps for a *secondary device* (complete these steps on the secondary device)

1. Open a *new* terminal in the ***catkin_workspace*** directory
2. Then from that terminal run <code>source devel/setup.bash</code>
3. Then from that terminal run <code>export ROS_MASTER_URI=http://___.___.___.___:11311</code> (filling in the **main device's IP address** found in the previous section)
4. Then from that terminal run the node you would like to run
    - e.g. In the same terminal, run <code>roslaunch ssf_package dashboard.launch</code> (recommended)
    <br>or
    - e.g. In the same terminal, run <code>rqt</code>
5. Steps 1. 2. 3. 4. can be repeated done to run another node

---

## Set Parameters Using Dynamic Reconfigure
The following command allow you to change parameter values using [http://wiki.ros.org/rqt_reconfigure].
```bash
rosrun rqt_reconfigure rqt_reconfigure
```
---

## Common Issues
* For the software to run correctly, please ensure the following is installed:
    * <code>pip install opencv-python</code>
    * <code>pip install numpy</code>
    * <code>pip install matplotlib</code>
    * <code>sudo apt-get install libalut-dev</code>
    * <code>sudo apt-get install libopenal-dev</code>
    * <code>sudo apt-get install python-scipy</code>
* Make sure all .py and .cfg files are executable, using
    * For .py files use
        ```bash
        chmod +x <file_name>.cfg
        ```
    * For .cfg files use
        ```bash
        chmod +x <file_name>.cfg
        ```
* rosrun or roslaunch can't find package or file:
    * Make sure the project is built (see [Building the project](#building_the_project))
    * Make sure you have sourced the files (see [Running the project locally](#running_the_project_locally))
---
