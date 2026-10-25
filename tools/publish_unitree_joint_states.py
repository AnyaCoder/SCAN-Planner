#!/usr/bin/env python3
"""Publish Unitree Go2 motor positions as sensor_msgs/JointState."""

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from unitree_go.msg import LowState


JOINT_NAMES = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]


class UnitreeJointStateBridge(Node):
    def __init__(self, lowstate_topic, output_topic, rate):
        super().__init__("unitree_joint_state_bridge")
        self.positions = [0.0] * len(JOINT_NAMES)
        self.velocities = [0.0] * len(JOINT_NAMES)
        lowstate_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        output_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.publisher = self.create_publisher(JointState, output_topic, output_qos)
        self.create_subscription(
            LowState,
            lowstate_topic,
            self.on_low_state,
            lowstate_qos,
        )
        self.create_timer(1.0 / max(1.0, rate), self.publish)
        self.get_logger().info(
            f"Unitree {lowstate_topic} -> {output_topic} at {rate:.1f} Hz"
        )

    def on_low_state(self, msg):
        for index in range(min(len(JOINT_NAMES), len(msg.motor_state))):
            state = msg.motor_state[index]
            self.positions[index] = float(state.q)
            self.velocities[index] = float(state.dq)

    def publish(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = self.positions
        message.velocity = self.velocities
        self.publisher.publish(message)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--lowstate-topic", default="/lowstate")
    parser.add_argument("--output-topic", default="/joint_states")
    parser.add_argument("--rate", type=float, default=30.0)
    args = parser.parse_args()

    rclpy.init()
    node = UnitreeJointStateBridge(
        args.lowstate_topic,
        args.output_topic,
        args.rate,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
