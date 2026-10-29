#!/usr/bin/env python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState


def model_joint_states(source: JointState) -> JointState:
    positions = dict(zip(source.name, source.position, strict=True))
    names = [f'joint{index}' for index in range(1, 7)]
    try:
        arm = [positions[name] for name in names]
        width = positions['gripper']
    except KeyError as error:
        raise ValueError(f'Missing driver joint: {error.args[0]}') from error
    if not all(math.isfinite(value) for value in arm + [width]):
        raise ValueError('Joint positions must be finite')
    result = JointState()
    result.header = source.header
    result.name = names + ['joint7', 'joint8']
    result.position = arm + [width / 2.0, -width / 2.0]
    return result


class ModelJointStateRelay(Node):
    def __init__(self, **kwargs):
        super().__init__('joint_state_relay', **kwargs)
        self.publisher = self.create_publisher(JointState, 'joint_states_model', 10)
        self.create_subscription(JointState, 'joint_states_feedback', self.receive, 10)

    def receive(self, source: JointState):
        try:
            result = model_joint_states(source)
        except ValueError as error:
            self.get_logger().error(f'Invalid driver joint feedback: {error}')
            return
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = ModelJointStateRelay()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
