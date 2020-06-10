#!/usr/bin/env python

import time
import icm20948
from math import pi
from sensor_msgs.msg import Imu, MagneticField
import rospy

print("""icm20948_publisher.py

Reads from the icm20948 IMU.

Publishes accelerometer and gyroscope data to imu/data_raw topic

Publishes magnetometer data to imu/mag topic

""")

imu = icm20948.ICM20948(icm20948.I2C_ADDR_ALT)
imu.set_accelerometer_full_scale(2)

def g_to_ms2(values):
    return [value*9.80665 for value in values]

def dps_to_radps(values):
    return [value*pi/180 for value in values]

def publish_imu_data(freq):
    pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    pub_mag = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
    rospy.init_node('icm20948_publisher', anonymous=True)
    rate = rospy.Rate(freq)
    seq = 0
    while not rospy.is_shutdown():
        ax, ay, az, gx, gy, gz = imu.read_accelerometer_gyro_data()
	[ax, ay, az] = g_to_ms2([ax, ay, az])
	[gx, gy, gz] = dps_to_radps([gx, gy, gz])

	mag_x, mag_y, mag_z = imu.read_magnetometer_data()
	mag_msg = MagneticField()
	mag_msg.magnetic_field.x = mag_x
	mag_msg.magnetic_field.y = mag_y
	mag_msg.magnetic_field.z = mag_z
	mag_msg.header.stamp = rospy.Time.now()
	mag_msg.header.seq = seq
	# TODO: Check if this frame_id is correct
	mag_msg.header.frame_id = ("mag")
	
	pub_mag.publish(mag_msg)
	
	imu_msg = Imu()
	# TODO: Input covariances from IMU datasheet 
	imu_msg.orientation_covariance[0] = -1
	imu_msg.angular_velocity_covariance[0] = -1
	imu_msg.linear_acceleration_covariance[0] = -1

	imu_msg.linear_acceleration.x = ax
	imu_msg.linear_acceleration.y = ay
	imu_msg.linear_acceleration.z = az
	imu_msg.angular_velocity.x = gx
	imu_msg.angular_velocity.y = gy
	imu_msg.angular_velocity.z = gz
	imu_msg.header.stamp = rospy.Time.now()
	imu_msg.header.frame_id = ("imu")
	imu_msg.header.seq = seq
        
	pub.publish(imu_msg)
	seq += 1
	rate.sleep()

if __name__ == '__main__':
    rospy.loginfo("Starting Imu publisher node")

    try:
	publish_imu_data(50)
    except rospy.ROSInterruptException:
	pass
 
