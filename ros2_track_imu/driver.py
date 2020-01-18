import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import math
import time

from transforms3d.euler import euler2quat as quaternion_from_euler

degrees2rad = math.pi / 180.0
imu_yaw_calibration = 0.0


class Ros2TrackIMUDriver(Node):
    def __init__(self):
        super().__init__('track_imu_driver')

        # We only care about the most recent measurement, i.e. queue_size=1
        self.pub = self.create_publisher(Imu, 'imu', 10)

        self.diag_pub = self.create_publisher(DiagnosticArray, 'diagnostics', 10)

        self.diag_pub_time = self.get_clock().now()

        self.imu_msg = Imu()

        self.frame_id = self.declare_parameter('frame_id', "base_imu_link").value

        # Orientation covariance estimation:
        # Observed orientation noise: 0.3 degrees in x, y, 0.6 degrees in z
        # Magnetometer linearity: 0.1% of full scale (+/- 2 gauss) => 4 milligauss
        # Earth's magnetic field strength is ~0.5 gauss, so magnetometer nonlinearity could
        # cause ~0.8% yaw error (4mgauss/0.5 gauss = 0.008) => 2.8 degrees, or 0.050 radians
        # i.e. variance in yaw: 0.0025
        # Accelerometer non-linearity: 0.2% of 4G => 0.008G. This could cause
        # static roll/pitch error of 0.8%, owing to gravity orientation sensing
        # error => 2.8 degrees, or 0.05 radians. i.e. variance in roll/pitch: 0.0025
        # so set all covariances the same.
        self.imu_msg.orientation_covariance = [
            0.0025, 0.0, 0.0,
            0.0, 0.0025, 0.0,
            0.0, 0.0, 0.0025
        ]

        # Angular velocity covariance estimation:
        # Observed gyro noise: 4 counts => 0.28 degrees/sec
        # nonlinearity spec: 0.2% of full scale => 8 degrees/sec = 0.14 rad/sec
        # Choosing the larger (0.14) as std dev, variance = 0.14^2 ~= 0.02
        self.imu_msg.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.02
        ]

        # linear acceleration covariance estimation:
        # observed acceleration noise: 5 counts => 20milli-G's ~= 0.2m/s^2
        # nonliniarity spec: 0.5% of full scale => 0.2m/s^2
        # Choosing 0.2 as std dev, variance = 0.2^2 = 0.04
        self.imu_msg.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04
        ]

        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.seq = 0
        self.accel_factor = 9.806 / 256.0  # sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.

        # read calibration parameters
        # accelerometer
        # self.declare_parameter('time_ref_source').value
        self.accel_x_min = self.declare_parameter('accel_x_min', -250.0).value 
        self.accel_x_max = self.declare_parameter('accel_x_max', 250.0).value
        self.accel_y_min = self.declare_parameter('accel_y_min', -250.0).value
        self.accel_y_max = self.declare_parameter('accel_y_max', 250.0).value
        self.accel_z_min = self.declare_parameter('accel_z_min', -250.0).value
        self.accel_z_max = self.declare_parameter('accel_z_max', 250.0).value

        # magnetometer
        self.magn_x_min = self.declare_parameter('magn_x_min', -600.0).value 
        self.magn_x_max = self.declare_parameter('magn_x_max', 600.0).value 
        self.magn_y_min = self.declare_parameter('magn_y_min', -600.0).value 
        self.magn_y_max = self.declare_parameter('magn_y_max', 600.0).value
        self.magn_z_min = self.declare_parameter('magn_z_min', -600.0).value 
        self.magn_z_max = self.declare_parameter('magn_z_max', 600.0).value
        self.calibration_magn_use_extended = self.declare_parameter('calibration_magn_use_extended', False).value 
        self.magn_ellipsoid_center = self.declare_parameter('magn_ellipsoid_center', [0, 0, 0]).value
        
        # Array of arrays parameters not supported.
        #self.magn_ellipsoid_transform = self.declare_parameter('magn_ellipsoid_transform', [[0, 0, 0], [0, 0, 0], [0, 0, 0]]).value 
        self.magn_ellipsoid_transform = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
        self.imu_yaw_calibration = self.declare_parameter('imu_yaw_calibration', 0.0).value 

        # gyroscope
        self.gyro_average_offset_x = self.declare_parameter('gyro_average_offset_x', 0.0).value
        self.gyro_average_offset_y = self.declare_parameter('gyro_average_offset_y', 0.0).value
        self.gyro_average_offset_z = self.declare_parameter('gyro_average_offset_z', 0.0).value

    def configure_imu(self, ser):
        ### configure board ###
        # stop datastream
        ser.write('#o0' + chr(13))

        # discard old input
        # automatic flush - NOT WORKING
        # ser.flushInput()  #discard old input, still in invalid format
        # flush manually, as above command is not working
        discard = ser.readlines()

        # set output mode
        ser.write('#ox' + chr(13))  # To start display angle and sensor reading in text

        self.get_logger().info("Writing calibration values to razor IMU board...")
        # set calibration values
        ser.write('#caxm' + str(self.accel_x_min) + chr(13))
        ser.write('#caxM' + str(self.accel_x_max) + chr(13))
        ser.write('#caym' + str(self.accel_y_min) + chr(13))
        ser.write('#cayM' + str(self.accel_y_max) + chr(13))
        ser.write('#cazm' + str(self.accel_z_min) + chr(13))
        ser.write('#cazM' + str(self.accel_z_max) + chr(13))

        if (not self.calibration_magn_use_extended):
            ser.write('#cmxm' + str(self.magn_x_min) + chr(13))
            ser.write('#cmxM' + str(self.magn_x_max) + chr(13))
            ser.write('#cmym' + str(self.magn_y_min) + chr(13))
            ser.write('#cmyM' + str(self.magn_y_max) + chr(13))
            ser.write('#cmzm' + str(self.magn_z_min) + chr(13))
            ser.write('#cmzM' + str(self.magn_z_max) + chr(13))
        else:
            ser.write('#ccx' + str(self.magn_ellipsoid_center[0]) + chr(13))
            ser.write('#ccy' + str(self.magn_ellipsoid_center[1]) + chr(13))
            ser.write('#ccz' + str(self.magn_ellipsoid_center[2]) + chr(13))
            ser.write('#ctxX' + str(self.magn_ellipsoid_transform[0][0]) + chr(13))
            ser.write('#ctxY' + str(self.magn_ellipsoid_transform[0][1]) + chr(13))
            ser.write('#ctxZ' + str(self.magn_ellipsoid_transform[0][2]) + chr(13))
            ser.write('#ctyX' + str(self.magn_ellipsoid_transform[1][0]) + chr(13))
            ser.write('#ctyY' + str(self.magn_ellipsoid_transform[1][1]) + chr(13))
            ser.write('#ctyZ' + str(self.magn_ellipsoid_transform[1][2]) + chr(13))
            ser.write('#ctzX' + str(self.magn_ellipsoid_transform[2][0]) + chr(13))
            ser.write('#ctzY' + str(self.magn_ellipsoid_transform[2][1]) + chr(13))
            ser.write('#ctzZ' + str(self.magn_ellipsoid_transform[2][2]) + chr(13))

        ser.write('#cgx' + str(self.gyro_average_offset_x) + chr(13))
        ser.write('#cgy' + str(self.gyro_average_offset_y) + chr(13))
        ser.write('#cgz' + str(self.gyro_average_offset_z) + chr(13))

        # print calibration values for verification by user
        ser.flushInput()
        ser.write('#p' + chr(13))
        calib_data = ser.readlines()
        calib_data_print = "Printing set calibration values:\r\n"
        for line in calib_data:
            calib_data_print += line
        self.get_logger().info(calib_data_print)

    def publish_imu_data(self, line):
        line = line.decode("utf-8").replace("ypr,", "").strip().replace("\t", "")  # Delete "#YPRAG="
        # f.write(line)                     # Write to the output log file
        words = line.split(",")  # Fields split
        if len(words) > 2:
            # in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
            yaw_deg = -float(words[0])
            yaw_deg = yaw_deg + self.imu_yaw_calibration
            if yaw_deg > 180.0:
                yaw_deg = yaw_deg - 360.0
            if yaw_deg < -180.0:
                yaw_deg = yaw_deg + 360.0
            yaw = yaw_deg * degrees2rad
            # in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
            pitch = -float(words[1]) * degrees2rad
            roll = float(words[2]) * degrees2rad

            # Publish message
            # AHRS firmware accelerations are negated
            # This means y and z are correct for ROS, but x needs reversing
            # self.imu_msg.linear_acceleration.x = -float(words[3]) * self.accel_factor
            # self.imu_msg.linear_acceleration.y = float(words[4]) * self.accel_factor
            # self.imu_msg.linear_acceleration.z = float(words[5]) * self.accel_factor

            # self.imu_msg.angular_velocity.x = float(words[6])
            # in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
            # self.imu_msg.angular_velocity.y = -float(words[7])
            # in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
            # self.imu_msg.angular_velocity.z = -float(words[8])

        q = quaternion_from_euler(roll, pitch, yaw)
        self.imu_msg.orientation.x = q[0]
        self.imu_msg.orientation.y = q[1]
        self.imu_msg.orientation.z = q[2]
        self.imu_msg.orientation.w = q[3]

        #self.imu_msg.header.stamp = rclpy.time.Time(seconds=time.time()).to_msg()
        self.imu_msg.header.stamp = self.get_clock().now().to_msg()
        self.imu_msg.header.frame_id = self.frame_id
        self.seq = self.seq + 1
        self.pub.publish(self.imu_msg)

        if self.diag_pub_time < self.get_clock().now():
            diag_arr = DiagnosticArray()
            diag_arr.header.stamp = self.get_clock().now().to_msg()
            diag_arr.header.frame_id = '1'
            diag_msg = DiagnosticStatus()
            diag_msg.name = 'Track_IMU'
            diag_msg.level = DiagnosticStatus.OK
            diag_msg.message = 'Received AHRS measurement'

            roll_key_val = KeyValue()
            yaw_key_val = KeyValue()
            pitch_key_val = KeyValue()
            seq_key_val = KeyValue()

            roll_key_val.key = 'roll (deg)'
            roll_key_val.value = str(roll * (180.0 / math.pi))

            yaw_key_val.key = 'yaw (deg)'
            yaw_key_val.value = str(yaw * (180.0 / math.pi))

            pitch_key_val.key = 'pitch (deg)'
            pitch_key_val.value = str(pitch * (180.0 / math.pi))

            diag_msg.values.append(roll_key_val)
            diag_msg.values.append(yaw_key_val)
            diag_msg.values.append(pitch_key_val)

            seq_key_val.key= 'sequence number'
            seq_key_val.value = str(self.seq)

            diag_msg.values.append(seq_key_val)
            diag_arr.status.append(diag_msg)
            self.diag_pub.publish(diag_arr)

    # Callback for dynamic reconfigure requests
    def reconfig_callback(self, config, level):
        global imu_yaw_calibration
        self.get_logger().info("""Reconfigure request for yaw_calibration: %d""" % (config['yaw_calibration']))
        # if imu_yaw_calibration != config('yaw_calibration'):
        imu_yaw_calibration = config['yaw_calibration']
        self.get_logger().info("Set imu_yaw_calibration to %d" % (imu_yaw_calibration))
        return config