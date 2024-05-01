import pytest
import rclpy
from rclpy.time import Time
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from interfaces.msg import Heartbeat
from common_package_py.common_node import CommonNode
import time


def test_heartbeat_rate():
    common_node = CommonNode("heartbeat")
    test_node = rclpy.create_node("test")

    heartbeat_period = 0.5
    executor = SingleThreadedExecutor()

    last_msg = Heartbeat()
    last_msg.tick = 0
    last_msg.time_stamp = test_node.get_clock().now().to_msg()

    def heartbeat_callback(msg):
        nonlocal last_msg

        time_diff = test_node.get_clock().now() - Time.from_msg(last_msg.time_stamp)
        assert time_diff <= rclpy.duration.Duration(
            seconds=heartbeat_period + 0.01)
        assert msg.tick == last_msg.tick + 1
        assert not msg.active
        assert msg.sender_id == "/heartbeat"
        last_msg = msg

    heartbeat_sub = test_node.create_subscription(
        Heartbeat, "heartbeat", heartbeat_callback, 1)

    def end_timer_callback():
        executor.shutdown(0)

    end_timer = test_node.create_timer(
        heartbeat_period * 10, end_timer_callback)

    executor.add_node(common_node)
    executor.add_node(test_node)

    executor.spin()
    del executor
    assert last_msg.tick == 10


def test_heartbeat_activate_deactivate():
    test_node = rclpy.create_node("test")

    heartbeat_period = 0.5
    executor = SingleThreadedExecutor()

    heartbeat_node = CommonNode("heartbeat")
    executor.add_node(heartbeat_node)

    last_msg = Heartbeat()
    last_msg.tick = 0
    last_msg.time_stamp = test_node.get_clock().now().to_msg()
    last_msg.active = True

    def heartbeat_callback(msg):
        nonlocal last_msg

        time_diff = test_node.get_clock().now() - Time.from_msg(last_msg.time_stamp)
        assert time_diff <= rclpy.duration.Duration(
            seconds=heartbeat_period + 0.01)
        assert msg.tick == last_msg.tick + 1
        assert msg.sender_id == "/heartbeat"
        assert msg.active == heartbeat_node.active

        if msg.active:
            assert not last_msg.active
            heartbeat_node._deactivate_()
        else:
            assert last_msg.active
            heartbeat_node._activate_()

        last_msg = msg

    heartbeat_sub = test_node.create_subscription(
        Heartbeat, "heartbeat", heartbeat_callback, 1)

    def end_timer_callback():
        executor.shutdown(0)

    end_timer = test_node.create_timer(
        heartbeat_period * 10.1, end_timer_callback)

    executor.add_node(test_node)

    executor.spin()
    del executor
    assert last_msg.tick == 10


if __name__ == "__main__":
    pytest.main()
