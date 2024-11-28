import argparse

import rosbag
import numpy as np
from sensor_msgs.msg import Imu
from scipy.interpolate import interp1d


parser = argparse.ArgumentParser()
parser.add_argument("input_bag", help="rosbag filename captured by RealSense T265")
parser.add_argument("output_bag", help="rosbag filename with merged imu")
args = parser.parse_args()


# Helper function to interpolate accelerometer data
def interpolate_accel(accel_data, accel_times, target_times):
    interpolator = interp1d(accel_times, accel_data, axis=0, kind='linear', fill_value="extrapolate")
    return interpolator(target_times)


# Open the original bag file and a new one for writing
bag_in = rosbag.Bag(args.input_bag, 'r')
bag_out = rosbag.Bag(args.output_bag, 'w')


# Extract messages from the Accel_0 and Gyro_0 topics
accel_msgs = []
gyro_msgs = []

# Read all messages from the bag and process accordingly
for topic, msg, t in bag_in.read_messages():
    # Copy all other topics to the new bag without changes
    if topic != '/device_0/sensor_0/Accel_0/imu/data' and topic != '/device_0/sensor_0/Gyro_0/imu/data':
        bag_out.write(topic, msg, t)  # Write all other topics to the new bag

    # Process the Accel_0 topic
    if topic == '/device_0/sensor_0/Accel_0/imu/data':
        accel_msgs.append((msg, t))  # Store message with timestamp

    # Process the Gyro_0 topic
    elif topic == '/device_0/sensor_0/Gyro_0/imu/data':
        gyro_msgs.append((msg, t))  # Store message with timestamp

# Initialize lists for IMU data
accel_data = []
gyro_data = []
accel_times = []
gyro_times = []

# Extract accelerometer data and gyro data
for msg, t in accel_msgs:
    accel_data.append([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
    accel_times.append(t.to_sec())  # Store time in seconds

for msg, t in gyro_msgs:
    gyro_data.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
    gyro_times.append(t.to_sec())  # Store time in seconds

# Convert to numpy arrays for easier manipulation
accel_data = np.array(accel_data)
gyro_data = np.array(gyro_data)

# Interpolate accelerometer data to match gyro timestamps (200Hz)
interpolated_accel_data = []
for gyro_time in gyro_times:
    # Interpolate accel data for each gyro timestamp
    interpolated_accel = interpolate_accel(accel_data, accel_times, gyro_time)
    interpolated_accel_data.append(interpolated_accel)

interpolated_accel_data = np.array(interpolated_accel_data)

# Write the new IMU data to the new bag file
for i in range(len(gyro_msgs)):
    gyro_msg, gyro_time = gyro_msgs[i]
    imu_msg = Imu()
    imu_msg.header.stamp = gyro_time  # Use gyro timestamp (200Hz)

    # Copy gyroscope data
    imu_msg.angular_velocity.x = gyro_data[i][0]
    imu_msg.angular_velocity.y = gyro_data[i][1]
    imu_msg.angular_velocity.z = gyro_data[i][2]

    # Interpolated accelerometer data
    imu_msg.linear_acceleration.x = interpolated_accel_data[i][0]
    imu_msg.linear_acceleration.y = interpolated_accel_data[i][1]
    imu_msg.linear_acceleration.z = interpolated_accel_data[i][2]

    # Write the IMU message to the new bag
    bag_out.write('/merged/imu/data', imu_msg, gyro_time)

# Close the bag files
bag_in.close()
bag_out.close()
