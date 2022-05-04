#!usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle Carla imu sensor
"""

import numpy as np

from transforms3d.euler import euler2quat

import carla_common.transforms as trans

from carla_ros_bridge.sensor import Sensor

from sensor_msgs.msg import Imu


class ImuSensor(Sensor):

    """
    Actor implementation details for imu sensor
    """

    def __init__(self, uid, name, parent, relative_spawn_pose, node, carla_actor, synchronous_mode):
        """
        Constructor

        :param uid: unique identifier for this object
        :type uid: int
        :param name: name identiying this object
        :type name: string
        :param parent: the parent of this
        :type parent: carla_ros_bridge.Parent
        :param relative_spawn_pose: the relative spawn pose of this
        :type relative_spawn_pose: geometry_msgs.Pose
        :param node: node-handle
        :type node: CompatibleNode
        :param carla_actor : carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(ImuSensor, self).__init__(uid=uid,
                                        name=name,
                                        parent=parent,
                                        relative_spawn_pose=relative_spawn_pose,
                                        node=node,
                                        carla_actor=carla_actor,
                                        synchronous_mode=synchronous_mode)

        self.imu_publisher = node.new_publisher(Imu, self.get_topic_prefix(), qos_profile=10)
        self.listen()
        self.previous_time = None
        self.previous_att = None
        self.sensor_attributes = carla_actor.attributes
        self.rs = np.random.RandomState(int(self.sensor_attributes['noise_seed']))

    def destroy(self):
        super(ImuSensor, self).destroy()
        self.node.destroy_publisher(self.imu_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, carla_imu_measurement):
        """
        Function to transform a received imu measurement into a ROS Imu message

        :param carla_imu_measurement: carla imu measurement object
        :type carla_imu_measurement: carla.IMUMeasurement
        """
        gyro_noise_x = self.rs.normal(loc=float(self.sensor_attributes['noise_gyro_bias_x']), 
                                      scale=float(self.sensor_attributes['noise_gyro_stddev_x']))
        gyro_noise_y = self.rs.normal(loc=float(self.sensor_attributes['noise_gyro_bias_y']), 
                                      scale=float(self.sensor_attributes['noise_gyro_stddev_y']))
        gyro_noise_z = self.rs.normal(loc=float(self.sensor_attributes['noise_gyro_bias_z']), 
                                      scale=float(self.sensor_attributes['noise_gyro_stddev_z']))
        imu_msg = Imu()
        imu_msg.header = self.get_msg_header(timestamp=carla_imu_measurement.timestamp)

        # Carla uses a left-handed coordinate convention (X forward, Y right, Z up).
        # Here, these measurements are converted to the right-handed ROS convention
        #  (X forward, Y left, Z up).
        roll, pitch, yaw = trans.carla_rotation_to_RPY(carla_imu_measurement.transform.rotation)
        quat = euler2quat(roll, pitch, yaw)
        imu_msg.orientation.w = quat[0]
        imu_msg.orientation.x = quat[1]
        imu_msg.orientation.y = quat[2]
        imu_msg.orientation.z = quat[3]

        if not self.previous_time:
            imu_msg.linear_acceleration.x = 0.0
            imu_msg.linear_acceleration.y = 0.0
            imu_msg.linear_acceleration.z = 0.0
            imu_msg.angular_velocity.x = 0.0
            imu_msg.angular_velocity.y = 0.0
            imu_msg.angular_velocity.z = 0.0
            self.previous_time = carla_imu_measurement.timestamp
            self.previous_att = roll, pitch, yaw
            # self.previous_pos = trans.carla_location_to_ros_point(carla_imu_measurement.transform.location)
        else:
            dt = carla_imu_measurement.timestamp - self.previous_time
            imu_msg.linear_acceleration.x = carla_imu_measurement.accelerometer.x
            imu_msg.linear_acceleration.y = -carla_imu_measurement.accelerometer.y
            imu_msg.linear_acceleration.z = carla_imu_measurement.accelerometer.z
            imu_msg.linear_acceleration_covariance = [float(self.sensor_attributes['noise_accel_stddev_x']), 0.0, 0.0, 
                                                      0.0, float(self.sensor_attributes['noise_accel_stddev_y']), 0.0, 
                                                      0.0, 0.0, float(self.sensor_attributes['noise_accel_stddev_z'])]
            imu_msg.angular_velocity.x = gyro_noise_x + (self.previous_att[0] - roll)/dt #-carla_imu_measurement.gyroscope.x
            imu_msg.angular_velocity.y = gyro_noise_y + (self.previous_att[1] - pitch)/dt #carla_imu_measurement.gyroscope.y
            imu_msg.angular_velocity.z = gyro_noise_z + (self.previous_att[2] - yaw)/dt #-carla_imu_measurement.gyroscope.z
            imu_msg.angular_velocity_covariance = [float(self.sensor_attributes['noise_gyro_stddev_x']), 0.0, 0.0, 
                                                   0.0, float(self.sensor_attributes['noise_gyro_stddev_y']), 0.0, 
                                                   0.0, 0.0, float(self.sensor_attributes['noise_gyro_stddev_z'])]
            self.previous_time = carla_imu_measurement.timestamp
            self.previous_att = roll, pitch, yaw

        self.imu_publisher.publish(imu_msg)