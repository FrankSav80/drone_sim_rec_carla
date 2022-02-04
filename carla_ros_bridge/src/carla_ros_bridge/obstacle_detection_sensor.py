#!/usr/bin/env python

#
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.
#
"""
Classes to handle obstacle_detection events
"""

from carla_ros_bridge.sensor import Sensor

from carla_msgs.msg import CarlaObstacleDetectionEvent


class ObstacleDetection(Sensor):

    """
    Actor implementation details for a obstacle detection
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
        :param carla_actor: carla actor object
        :type carla_actor: carla.Actor
        :param synchronous_mode: use in synchronous mode?
        :type synchronous_mode: bool
        """
        super(ObstacleDetection, self).__init__(uid=uid,
                                              name=name,
                                              parent=parent,
                                              relative_spawn_pose=relative_spawn_pose,
                                              node=node,
                                              carla_actor=carla_actor,
                                              synchronous_mode=synchronous_mode,
                                              is_event_sensor=True)

        self.obstacle_detection_publisher = node.new_publisher(CarlaObstacleDetectionEvent,
                                                      self.get_topic_prefix(),
                                                      qos_profile=10)
        self.listen()

    def destroy(self):
        super(ObstacleDetection, self).destroy()
        self.node.destroy_publisher(self.obstacle_detection_publisher)

    # pylint: disable=arguments-differ
    def sensor_data_updated(self, obstacle_detection_event):
        """
        Function to wrap the obstacle_detection event into a ros messsage

        :param obstacle_detection_event: carla obstacle_detection event object
        :type obstacle_detection_event: carla.ObstacleDetectionEvent
        """
        obstacle_detection_msg = CarlaObstacleDetectionEvent()
        obstacle_detection_msg.header = self.get_msg_header(timestamp=obstacle_detection_event.timestamp)
        obstacle_detection_msg.actor_id = obstacle_detection_event.actor.id
        obstacle_detection_msg.other_actor_id = obstacle_detection_event.other_actor.id
        obstacle_detection_msg.distance = obstacle_detection_event.distance
        
        self.obstacle_detection_publisher.publish(obstacle_detection_msg)
