# 3D Laser Scan with Hokuyo LIDAR & Dynamixel motor

This pacakage makes the servo to which the laser scanner is attached, spin. It 
takes the 2D laser scans and combines them into a 3D point cloud


### Install Dependencies

NOTE: If you use another version of ROS rather than `indigo`, make the proper substitutions

1. urg_node 
```
sudo apt-get install ros-indigo-urg-node
```

2. dynamixel_motor 
```
sudo apt-get install ros-indigo-dynamixel-motor
```

3. laser_assembler 
```
sudo apt-get install ros-indigo-laser-assembler
```


### Run a 3D Scan

```
roslaunch lidar_3d_scan lidar_3d_scan.launch
```

#### Configuration

##### Arguments

- `scan_in`: output topic for the laser scan from `urg_node` and input topic for 
`laser_assembler`. Default is `/3d_laser_scan`

- `robot_base`: the frame into which the output point cloud is published. 
Default is `base_link`

- `cloud_out`: name of the topic for the output point cloud. The type of the 
message is `sensor_msgs/PointCloud2`

- `lidar_ip`: IP of the lidar to use. Default is `192.168.2.10` - orange laser
scanner. IP for the dark blue laser scanner should be `192.168.2.11` (provided
you configured it properly)

##### Tf transforms and coordinate frames

- laser scanner to the center of the servo gear - modify the static transform
in the node called `lidar_static_frame_br` in the `lidar_motor_control.launch`

- servo orientation to robot base - automatically done by 
`lidar_3d_tf_broadcaster.py`


##### urg_node
- Default output topic will be called `/3d_laser_scan`. If you want to change
it, provide the argument `scan_in` to the launchfile. This will automatically 
change the input topic name for `laser_assembler automatically`

##### laser_assembler
NOTE: `laser_assembler` uses `tf` for getting the position of the laser scanner.
This means that you don't need to publish the position of the servo at the 
exact same time that a laser scan was made as `tf` will perform interpolation 
automatically.
- Documentation: http://wiki.ros.org/laser_assembler


### Documentation

#### `urg_node`
- Documentation: http://wiki.ros.org/urg_node
- Package that runs the laser scanner

#### `laser_assembler`
- Documentation: http://wiki.ros.org/laser_assembler
- Package that combines the scans into point cloud
- NOTE: `laser_assembler` uses `tf` for getting the position of the laser scanner.
This means that you don't need to publish the position of the servo at the 
exact same time that a laser scan was made as `tf` will perform interpolation 
automatically.

#### `dynamixel_controllers`
- Documentation: http://wiki.ros.org/dynamixel_controllers
- Package that controls the dynamixel servo
- Tutorials: http://wiki.ros.org/dynamixel_controllers/Tutorials


##### Existing Dynamixel Type in ROBIN Lab

| Motor         | Interface  | USB Controller                                                |
| ------------- | ---------- | ------------------------------------------------------------- |
| AX-12A        | TTL        | USB2DYNAMIXEL (ROBOTIS)  & USB2AX v3.2b (xevelabs.com)        |
| MX-12W        | TTL        | USB2DYNAMIXEL (ROBOTIS)  & USB2AX v3.2b (xevelabs.com)        |
| MX-28R        | RS485      | USB2DYNAMIXEL (ROBOTIS)  & USB to Dynamixel R (RoboSavvy)     |


##### Default address for USB dynamixel controller

| Controller                     | Default port address     | 
| ------------------------------ | ------------------------ | 
| USB2DYNAMIXEL (ROBOTIS)        | /dev/ttyUSB0             |
| USB2AX v3.2b (xevelabs.com)    | /dev/ttyACM0             | 
| USB to Dynamixel R (RoboSavvy) | /dev/ttyUSB0             | 

##### Motor Setting

| Motor         | ID     | Angle Limit           | Resolution  | Baud Rate   |
| ------------- | ------ | --------------------- | ----------- | ----------- |
| AX-12A        | 1      | 0~1023 ; 0~300 degree | 0.35 deg    | 1000000     |
| MX-12W        | 2      | 0~4095 ; 0~360 degree | 0.088 deg   | 1000000     |
| MX-28R        | 3      | 0~4095 ; 0~360 degree | 0.088 deg   | 57600       |


##### Initialise motor ID
Using Windows based GUI: Dynamixel Wizard

Required software: Roboplus
    
Source: http://en.robotis.com/BlueAD/board.php?bbs_id=downloads&mode=view&bbs_no=1132559&page=1&key=bbs_subject&keyword=Roboplus&sort=&scate=
