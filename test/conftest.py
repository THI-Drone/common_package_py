import pytest
import rclpy

def pytest_sessionstart(session):
    rclpy.init()

def pytest_sessionfinish(session, exitstatus):
    rclpy.shutdown()