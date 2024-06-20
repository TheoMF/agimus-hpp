1. First start the ros master with `roscore`

2. Start the HPP Corba Server `hppcorbaserver`

3. Start the HPP node `roslaunch start_hpp.launch`

4. Generate a path with HPP. With agimus-demo for franka, you can use script_hpp.py. You can start the script with :
```
python -i script_hpp.py
```
And generate the path : 
```
q_init, p = GrabAndDrop(robot, ps, binPicking, <your pose acquisition method>)
```
Verify that the path is generated. The console should show : 
```
[INFO] Object found with no collision
Solving ...
Path generated.
```

5. Give the joint you want the acceleration from : 
```
rosservice call /hpp/target/set_joint_names "names:
- 'pandas/panda2_joint1'
- 'pandas/panda2_joint2'
- 'pandas/panda2_joint3'
- 'pandas/panda2_joint4'
- 'pandas/panda2_joint5'
- 'pandas/panda2_joint6'"
```

6. Read the path genereated `rostopic pub /hpp/target/read_path std_msgs/UInt32 0`

7. Display the acceleration topic from the path : `rostopic echo /hpp/target/acceleration`

8. Publish the inforamtion from the path : `rostopic pub /hpp/target/publish std_msgs/Empty`
The acceleration should appear in the terminal where you display the acceleration topic.