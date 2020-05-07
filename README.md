# hilsys_new
new hilsys package to generate the hardware in the loop system

##bulid<br>
do NOT forget to build the parallel_car and iiwa_agv package

##plan trajectory
```
roslaunch sim_sys start_sim_sys.launch
```
```
rosrun sim_sys sys_planner.py
```
the planned trajectory will be placed at ../sim_sys/data/*<br>
this path is `ABSOLUTE` Path. before you use it, plz change the path to your absolute path

##control the robot
```
roslaunch sim_sys whole_system.launch
```
```
rosrun sim_sys start_sim.py
```
the start_sim.py node read trajectory from data in txt file.<br>
change the origin path to your absolute path

if you want to see the figure of the poles, run<br>
```
rosrun parallel_car parallel_monitor.py
```
or launch the launch file that merge the second the third command<br>

```
roslaunch sim_sys start_movement.launch 
```

