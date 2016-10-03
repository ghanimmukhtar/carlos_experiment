# carlos_experiment
This is a package that makes the left arm of BAXTER robot touches a cube from four sides in a loop where the starting position of the arm changes each iteration. It is done in simulation with GAZEBO

So among things you need are GAZEBO and a BAXTER simulator with SDK v1.2. 

The node to be used is called (carlos_experiment_diff_init) it is already configured in the launch file

To be able to use this package you need to install (descartes) library, which is a library that produce cartesian trajectories for any serial arm if they are attainable. There is a public one in ros-industrial website but we have made some changes to adapt the library to our experiment so try to install the repository https://github.com/ghanimmukhtar/descartes.git

After you have everything you need simply to bring BAXTER on GAZEBO by running ```roslaunch baxter_gazebo baxter_world.launch``` 

Then enable the robot by executing ```rosrun baxter_tools tuck_arm -u```, you might need to wait sometime before being able to execute this command

Then run the experiment by running ```roslaunch carlos_experiment carlos_experiment.launch```
