import json
import rclpy
from rclpy.node import Node

from interfaces.msg import Heartbeat, JobFinished
from common_package_py.topic_names import TopicNames


class CommonNode(Node):
    """A class representing a common node.

        This class inherits from rclpy.node.Node and provides functionality for creating a node that sends heartbeat messages.
    """

    # Constants
    EXIT_SUCCESS = 0
    EXIT_FAILURE = 1

    # Maximum allowed flight height in cm
    MAX_FLIGHT_HEIGHT_CM = 120 * 100  # [m] * 100

    # Minimum required cruise height in cm
    MIN_CRUISE_HEIGHT_CM = 5 * 100  # [m] * 100

    # Maximum allowed horizontal speed in m/s
    MAX_HORIZONTAL_SPEED_MPS = 12.0  # [m/s]

    # Maximum allowed vertical speed in m/s
    MAX_VERTICAL_SPEED_MPS = 3.0  # [m/s]

    # Minimum SOC after which a RTH is triggered
    MIN_SOC_PERCENT = 20.0  # [%]

    def __init__(self, id: str) -> None:
        """Constructor to create a new node.

        :param id: Node id
        """

        super().__init__(id)

        # Indicating if node is currently active and sending commands to
        # interface node
        self.__node_active__ = False

        # Create a publisher for the "heartbeat" topic
        self.__heartbeat_publisher__ = self.create_publisher(
            Heartbeat, TopicNames.Heartbeat, 1)

        # Create a timer that sends a heartbeat message every 0.5s
        self.__heartbeat_period__ = 0.5
        self.__heartbeat_timer__ = self.create_timer(
            self.__heartbeat_period__, self.__heartbeat_timer_callback__)

        # Tick counting upwards with every heartbeat
        self.__tick__ = 0

        # Create a publisher for the "job_finished" topic
        self.__job_finished_publisher__ = self.create_publisher(
            JobFinished, TopicNames.JobFinished, 10)

    def _activate_(self) -> None:
        """
        Activates the node.

        This method sets the `__node_active__` attribute to True, indicating that the node is active.
        """
        self.__node_active__ = True
        self.get_logger().debug('CommonNode::_activate_: Activated node')

    def _deactivate_(self) -> None:
        """
        Deactivates the node.

        Sets the `__node_active__` attribute to False, indicating that the node is no longer active.
        """
        self.__node_active__ = False
        self.get_logger().debug('CommonNode::_deactivate_: Deactivated node')

    @property
    def active(self) -> bool:
        """
        Returns the active status of the node.

        :return: True if the node is active, False otherwise.
        """
        return self.__node_active__

    def __heartbeat_timer_callback__(self) -> None:
        """
        This method is called by the timer to publish a heartbeat message.

        It creates a Heartbeat message, sets the sender ID, active status, and tick count,
        and publishes the message using the publisher.
        """

        msg = Heartbeat()

        msg.sender_id = self.get_name()
        msg.active = self.__node_active__
        self.__tick__ += 1
        msg.tick = self.__tick__
        msg.time_stamp = self.get_clock().now().to_msg()

        self.__heartbeat_publisher__.publish(msg)

        self.get_logger().debug(
            f"CommonNode::__heartbeat_timer_callback__: Published heartbeat message with sender_id: {msg.sender_id}, tick: {msg.tick}, active: {msg.active}")

    def _job_finished_custom_(self, error_code: int, payload: dict) -> None:
        """
        Handles the completion of a job.

        This function sends a job_finished message with the given error code and payload.
        Additionally, deactivates the node.

        :param error_code: The error code associated with the job completion (self.EXIT_SUCCESS == 0 if no error).
        :param payload: The payload data associated with the job completion.
        """

        msg = JobFinished()

        msg.sender_id = self.get_name()
        msg.error_code = error_code

        try:
            msg.payload = json.dumps(payload)
        except BaseException:
            self.get_logger().fatal(
                "CommonNode::_job_finished_custom_: Payload is not a valid JSON. Stopping node.")
            exit(CommonNode.EXIT_FAILURE)

        self.__job_finished_publisher__.publish(msg)
        self.get_logger().debug(
            f"CommonNode::_job_finished_custom_: Sent job_finished message with error_code {error_code} and payload")

        # Deactivate the node
        self._deactivate_()

    def _job_finished_error_msg_(self, error_message: str) -> None:
        """
        Handles the completion of a job with the given error message.
        Can be used if your job fails and you just want to return an error message.

        This function formats the error message to JSON and sends a job_finished message with the error message.
        Additionally, deactivates the node.
        This function should only be used if your job failed.

        :param error_message: The error message associated with the job completion.
        """

        msg = JobFinished()

        msg.sender_id = self.get_name()
        # set error code to EXIT_FAILURE to indicate success
        msg.error_code = CommonNode.EXIT_FAILURE

        try:
            msg.payload = json.dumps({"error_msg": error_message})
        except BaseException:
            self.get_logger().fatal(
                "CommonNode::_job_finished_error_msg_: Payload is not a valid JSON. Stopping node.")
            exit(CommonNode.EXIT_FAILURE)

        self.__job_finished_publisher__.publish(msg)
        self.get_logger().debug(
            f"CommonNode::_job_finished_error_msg_: Sent job_finished message with error message '{error_message}' and EXIT_FAILURE error code")

        # Deactivate the node
        self._deactivate_()

    def _job_finished_successfull_(self) -> None:
        """
        Sends a job_finished message with error code 0 and no error message.

        This function should be executed only if your job was completed successfully.
        Additionally, deactivates the node.
        """

        msg = JobFinished()

        msg.sender_id = self.get_name()
        # set error code to EXIT_SUCCESS to indicate success
        msg.error_code = CommonNode.EXIT_SUCCESS
        msg.payload = "{}"  # payload is empty JSON

        self.__job_finished_publisher__.publish(msg)
        self.get_logger().debug(
            "CommonNode::_job_finished_successfull_: Sent job_finished message indicating success")

        # Deactivate the node
        self._deactivate_()
