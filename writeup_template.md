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
[image12]: ./misc_images/codeblock.png
[image13]: ./misc_images/theta123.jpg
[image14]: ./misc_images/R3_6.png
[image15]: ./misc_images/R3_6_from_transforms.png
[image16]: ./misc_images/theta456.jpg
[image17]: ./misc_images/kinematics_complete.png

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

And to implement this in Python code, I used the following snippet of code, taken directly from the Udacity classroom video:
![alt text][image12]  


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.
The procedure is as follows:  
First we calculate the location of the wrist center (WC), using the position and orientation of the gripper, and the length of the last link. Here, the wrist center can be found by performing a translation of 0.303 to go from the gripper to the WC. In the x-y plane, the wrist center is located at (WCx, WCy). With the location of the wrist center, we calculate theta1 as the inverse tangent of (Wcy / WCx).  
To calculate theta2 and theta3, we construct the traingle made by joint 2, joint 3, and the wrist center. The sides of this triangle are calculated from the values in the URDF and Pythagoras theorem. The angles of this traingle are calculated from the sides, using the cosine rule. Then, theta2 and theta3 can be found using simple geometry. My calculations are included in the following image:
![alt text][image13]  

For angles theta4, theta5, theta6, i.e. for the inverse orientation, we can peform the following calculation. We have the matrix R0_6 from the individual transformation matrices, and we have R0_3 from our calculation of theta1, theta2, theta3. This can give us R3_6 as follows (screenshot captured from the Udacity classroom lesson):  
![alt text][image14]  

Now since the inverse of R0_3 is also the transpose of R0_3, we simply calculate R0_3 transposed * Rrpy = R3_6. We can calculate analytically the expressions for each term in R3_6 by multiplying the transformation matrices to go from joint 3 to the end effector. Let's write this out in full:  

```
    print("This is R3_6 from the homogenous transformation matrices\n")
    R3_6m = simplify(T3_4[0:3, 0:3] * T4_5[0:3, 0:3] * T5_6[0:3, 0:3] * T6_EE[0:3, 0:3])
    print(R3_6m)
```

Here's what this gives us:  
![alt text][image15] 

Looking at this matrix, we can use certain terms in it to solve for theta4, theta5, and theta6 separately. Let's write those terms out, with respect to the matrix R3_6.  
![alt text][image16]  

The calculations show two possible solutions for theta5, as there are two roots to the equation to solve for atan2(theta5). Let's expand what this means for theta4 and theta6. Suppose we pick the negative root, then sin(theta4) must be less than zero, in which case, theta4 and theta6 take on the first set of expressions in the diagram above. If we pick the positive root, then sin(theta5) is positive, which means theta4 and theta6 must take on the second set of expressions. I selected the positive root for this simulation and ran the IK_debug script with the corresponding for theta4, theta5, and theta6. This gave me good results, so I used this solution in the IK_server.py code.

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

The code that I implemented was based on the calculations I showed above, and heavily inspired by the project walkthrough. The one change I made was to use the transpose of R0_3 instead of the inverse, because based on discussions in the Slack forum, I learned that this resulted in better performance by the robot. To improve this project further, I would start by making more precise calculations with higher decimal point accuracy. I would also make use of [Lambdify](http://docs.sympy.org/latest/modules/utilities/lambdify.html) to try and speed up the performance, rather than using subs and evalf.
A screenshot of the completed pick and place operation using the IK_server.py code is attached below. As can be seen, the pick and place operation succeeds at least 8 times out of 10:  
![alt text][image17] 

