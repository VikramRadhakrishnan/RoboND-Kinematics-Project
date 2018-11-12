## Project: Kinematics Pick & Place


[//]: # (Image References)

[image1]: ./misc_images/sketch.jpg
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

To perform the FK analysis, I first sketched the diagram of the Kuka KR210 robot, and labeled all the joints (1 to 6), links (1 to 6), and marked the link lengths (a) and link offsets (d). The diagram is as shown below:
![alt text][image1]

The next step was to create the DH parameter table using the modified DH parameters as described in the lesson. To get the numerical values of each of the “a” and “d” values, I referred to the URDF file for the Kuka KR210 robot. The relevant lines are lines 317 to 363 in the URDF file, which gives the specifications of each joint. Using the information in these lines, I constructed the DH table as shown below:

\begin{table}[]
\begin{tabular}{lllll}
\begin{tabular}[c]{@{}l@{}}Links\\ 			i\end{tabular} & αi-1  & ai-1   & di    & qi                                                     \\
0 --\textgreater 1                                   & 0     & 0      & 0.75  & q1                                                     \\
1 --\textgreater 2                                   & -pi/2 & 0.35   & 0     & \begin{tabular}[c]{@{}l@{}}q2\\ 			– pi/2\end{tabular} \\
2 --\textgreater 3                                   & 0     & 1.25   & 0     & q3                                                     \\
3 --\textgreater 4                                   & -pi/2 & -0.054 & 1.5   & q4                                                     \\
4 --\textgreater 5                                   & pi/2  & 0      & 0     & q5                                                     \\
5 --\textgreater 6                                   & -pi/2 & 0      & 0     & q6                                                     \\
7                                                    & 0     & 0      & 0.303 & 0                                                     
\end{tabular}
\end{table}

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]


