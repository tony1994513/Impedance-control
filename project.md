#summary
Impedance control applies a **mass-spring-damper** between the target position and the actual position is the air. The node has been tested on **ROS Indigo** powered by **Ubuntu 14.04** only. If you meet any problem with the code, please contact us.


####code location
[impedcontrol](https://github.com/birlrobotics/birl_baxter_controllers/tree/master/imped_control) 


###Quickstart:
Frist, Please configure your computer with the wiki [birl_baxter wiki](https://github.com/birlrobotics/birl_baxter/wiki).  and aslo install baxter simulator first [baxter_simulator](http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)  We recommend using Gazebo first.

   Run the baxter shell script with sim specified:

`$ ./baxter.sh sim`

   Start simulation with controllers:

`$ roslaunch baxter_gazebo baxter_world.launch`

You should wait for the following three lines before the simulator is truly running:

	[ INFO] [1400513321.531488283, 34.216000000]: Simulator is loaded and started successfully
	[ INFO] [1400513321.535040726, 34.219000000]: Robot is disabled
	[ INFO] [1400513321.535125386, 34.220000000]: Gravity compensation was turned off


Also make sure you connect to Force sensor, if you cannot receive the force information then you can not get in the callback function!

`$ rosrun ft_wacoh ft_wacoh_pub` 

You can check by `$ rostopic echo /wrench/filter` to see if you can receive the data.
Finally run the impedance code 
`$ rosrun birl_baxter_controllers   impedcontrol.py -l right`

**Reference**
See the following link [A brief introduction to impdecontrol](https://ocw.mit.edu/courses/mechanical-engineering/2-141-modeling-and-simulation-of-dynamic-systems-fall-2006/lecture-notes/interaction_cont.pdf) , p. 4-9.


>** schematic diagram**
![ ](/home/tony/impedcontol/imped.png  "impedcontrol ")

##Stucture:##
This code mainly refer to `baxter_example/scripts/joint_torque_spring.py` you can find the code [here](http://sdk.rethinkrobotics.com/wiki/Joint_Torque_Springs_-_Code_Walkthrough) . because it has already made the Spring-Damping circle. And we can see the picture that* Kd* can represent *Spring gain*, and Bd represent *Damping gain*. Here **e* , e_dot, e_double_dot* means position error, velocity error and accelerate error respectively. Xr,Xr_dot,Xr_double_dot means the desired position and velocity and acceleration respectively. And we can see that the output of first cicle is *Fr* and there is also a *Fe*, which I can get from Force sensor. So after the combination of the two force information, it can be sent to the robot as  command torque, which baxter robot has built-in function that can help us finish this.

##Video

