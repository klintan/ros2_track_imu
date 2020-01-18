#!/usr/bin/env python

# Copyright (c) 2012, Tang Tiong Yew
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rclpy
import serial
import sys
import time

from ros2_track_imu.driver import Ros2TrackIMUDriver


def main(args=None):
    rclpy.init(args=args)

    driver = Ros2TrackIMUDriver()

    serial_port = driver.declare_parameter('port', '/dev/tty.usbmodemHIDFH1').value
    serial_baud = driver.declare_parameter('baud', 115200).value

    try:
        ser = serial.Serial(port=serial_port, baudrate=serial_baud, timeout=1)
    except serial.serialutil.SerialException:
        driver.get_logger().error("IMU not found at port " + serial_port + ". Did you specify the correct port in the launch file?")
        # exit
        sys.exit(0)

    driver.get_logger().info("Giving the TrackIMU board 5 seconds to boot...")
    time.sleep(0)

    # calibrate and configure IMU
    # driver.configure_imu(ser)

    # start datastream
    # ser.write('#o1' + chr(13))

    # automatic flush - NOT WORKING
    # ser.flushInput()  #discard old input, still in invalid format
    # flush manually, as above command is not working - it breaks the serial connection
    driver.get_logger().info("Flushing first 200 IMU entries...")
    for x in range(0, 200):
        _ = ser.readline()

    driver.get_logger().info("Publishing IMU data...")
    while rclpy.ok():
        line = ser.readline()

        driver.publish_imu_data(line)

    ser.close


if __name__ == '__main__':
    main()
