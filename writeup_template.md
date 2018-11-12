## Project: Kinematics Pick & Place


[//]: # (Image References)

[image1]: ./misc_images/sketch.jpg
[image2]: ./misc_images/eq2.png
[image3]: ./misc_images/image-3.png
[image4]: ./misc_images/T0_1.jpg
[image5]: ./misc_images/T1_2.jpg
[image6]: ./misc_images/T2_3.jpg
[image7]: ./misc_images/T3_4.jpg
[image8]: ./misc_images/T4_5.jpg
[image9]: ./misc_images/T5_6.jpg
[image10]: ./misc_images/T6_EE.jpg
[image11]: ./misc_images/T0_EE.jpg

### Writeup / README  

### About the project:
This project consists of performing the forward and inverse kinematic analysis of the KUKA KR210 robotic manipulator, and then implementing this to perform a pick and place operation in a simulated environment. The simulation is built in Gazebo, and the control of the robotic arm is implemented in ROS. Most of the code used in this project was already provided by Udacity, and I worked on implementing the IK_server ROS node, which was responsible for providing a service to the environment to request joint trajectories, given the pose trajectories obtained by the planner.

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To perform the FK analysis, I first sketched the diagram of the Kuka KR210 robot in its zero configuration, i.e. the configuration it is in when I first launch the simulation. The KUKA KR210 arm has 6 revolute joints, which control the position of the gripper, and two prismatic joints to open and close the gripper. Following the Udacity classroom video, I labeled all the joints (1 to 6), links (1 to 6), and marked the link lengths (a) and link offsets (d). The diagram is as shown below:
![alt text][image1]

The next step was to create the DH parameter table using the modified DH parameters as described in the lesson. To get the numerical values of each of the “a” and “d” values, I referred to the URDF file for the Kuka KR210 robot. The relevant lines are lines 317 to 363 in the URDF file, which gives the specifications of each joint. Using the information in these lines, I constructed the DH table as shown below:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | qi
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  - pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

For each joint, I used the values from the DH parameter table above to create the corresponding transformation matrix. This was based on the fact that the homogenous transformation matrix going from frame i-1 to frame i is given by the equation:
![alt text][image2]  
![alt text][image3]

So, going link by link from link 0 to the end effector, we get the following homogenous transformation matrixes:  
(Note: these matrixes were generated using [this online latex editor](https://www.codecogs.com/latex/eqneditor.php))  
![alt text][image4]  
![alt text][image5]  
![alt text][image6]  
![alt text][image7]  
![alt text][image8]  
![alt text][image9]  

For the transformation from the 6th joint to the gripper, we need to account for the difference between the orientation of the gripper link as defined in the URDF, and its DH convention, which means we include a body fixed rotation about the Z axis, and then the Y axis. This gives us:
![alt text][image10]

The complete transformation from the base link to the end effector is then given by:
![alt text][image11]


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


