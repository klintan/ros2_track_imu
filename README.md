# ROS2 TrackIMU Node

Ros2 Node for TrackIMU

### Installation
Source your ROS2 workspace and run

`colcon build`


### Usage

`ros2 run ros2_track_imu imu_node`


Configure
---------
In its default configuration, ``ros2_track_imu`` expects a yaml config file ``ros2_track_imu.yaml`` with:
* USB port to use
* Calibration parameters

An example``ros2_track_imu.yaml`` file is provided.
Copy that file to ``ros2_track_imu.yaml`` as follows:


Then, edit ``ros2_track_imu.yaml`` as needed

### Ros2 Launch

*Not implemented yet.*

Publisher and 3D visualization:

	$ ros2 launch ros2_track_imu track-imu-pub-and-display.py

Publisher only:

	$ ros2 launch ros2_track_imu track-imu-pub.py

Publisher only with diagnostics:

	$ ros2 launch ros2_track_imu track-imu-pub-diags.py

3D visualization only:

	$ ros2 launch ros2_track_imu track-imu-display.py


### Calibrate

For best accuracy, follow the tutorial to calibrate the sensors:

http://wiki.ros.org/razor_imu_9dof

An updated version of Peter Bartz's magnetometer calibration scripts from https://github.com/ptrbrtz/razor-9dof-ahrs is provided in the ``magnetometer_calibration`` directory.

Update ``my_razor.yaml`` with the new calibration parameters.
