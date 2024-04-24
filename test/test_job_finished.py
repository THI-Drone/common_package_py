import pytest
import time
import json
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node

from common_package_py.common_node import CommonNode

from interfaces.msg import JobFinished


def test_job_finished_successfull():
    executor = SingleThreadedExecutor()

    common_node = CommonNode("common_node")
    assert not common_node.active
    common_node._activate_()
    assert common_node.active

    test_node = Node("test")

    def job_finished_callback(msg):
        nonlocal common_node
        nonlocal test_node
        nonlocal executor

        test_node.get_logger().debug("Got job_finished message")
        assert msg.sender_id == "/common_node"
        assert msg.error_code == common_node.EXIT_SUCCESS
        assert json.loads(msg.payload) == {}
        assert not common_node.active
        executor.shutdown(0)

    job_finished_sub = test_node.create_subscription(
        JobFinished, "job_finished", job_finished_callback, 10)

    def timer_callback():
        nonlocal common_node
        test_node.get_logger().debug("Sending job_finished successfull message")
        common_node._job_finished_successfull_()

    job_finished_timer = common_node.create_timer(
        0.1, timer_callback)

    executor.add_node(common_node)
    executor.add_node(test_node)

    executor.spin()


def test_job_finished_custom_payload():
    executor = SingleThreadedExecutor()

    common_node = CommonNode("common_node")
    assert not common_node.active
    common_node._activate_()
    assert common_node.active

    test_node = Node("test")

    def job_finished_callback(msg):
        nonlocal common_node
        nonlocal test_node
        nonlocal executor

        test_node.get_logger().debug("Got job_finished message")
        assert msg.sender_id == "/common_node"
        assert msg.error_code == 5

        payload_check = {}
        payload_check["coord"] = "These are my coordinates"
        payload_check["height_cm"] = 500
        assert json.loads(msg.payload) == payload_check

        assert not common_node.active
        executor.shutdown(0)

    job_finished_sub = test_node.create_subscription(
        JobFinished, "job_finished", job_finished_callback, 10)

    def timer_callback():
        nonlocal common_node
        test_node.get_logger().debug("Sending job_finished successfull message")

        payload = {}
        payload["coord"] = "These are my coordinates"
        payload["height_cm"] = 500
        common_node._job_finished_custom_(5, payload)

    job_finished_timer = common_node.create_timer(
        0.1, timer_callback)

    executor.add_node(common_node)
    executor.add_node(test_node)

    executor.spin()


def test_job_finished_error_message():
    executor = SingleThreadedExecutor()

    common_node = CommonNode("common_node")
    assert not common_node.active
    common_node._activate_()
    assert common_node.active

    test_node = Node("test")

    def job_finished_callback(msg):
        nonlocal common_node
        nonlocal test_node
        nonlocal executor

        test_node.get_logger().debug("Got job_finished message")
        assert msg.sender_id == "/common_node"
        assert msg.error_code == common_node.EXIT_FAILURE
        assert json.loads(msg.payload) == {
            "error_msg": "This is my error message"}
        assert not common_node.active
        executor.shutdown(0)

    job_finished_sub = test_node.create_subscription(
        JobFinished, "job_finished", job_finished_callback, 10)

    def timer_callback():
        nonlocal common_node
        test_node.get_logger().debug("Sending job_finished successfull message")
        common_node._job_finished_error_msg_("This is my error message")

    job_finished_timer = common_node.create_timer(
        0.1, timer_callback)

    executor.add_node(common_node)
    executor.add_node(test_node)

    executor.spin()


if __name__ == "__main__":
    pytest.main()
