import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time

from interfaces.msg import Heartbeat


class CommonNode(Node):
    """A class representing a common node.
    
        This class inherits from rclpy.node.Node and provides functionality for creating a node that sends heartbeat messages.
    """

    def __init__(self, id: str):
        """Constructor to create a new node.

        Args:
            id (str): Node id
        """
        
        super().__init__(id)
        
        # Indicating if node is currently active and sending commands to interface node
        self.__node_active__ = False
        
        # Create a publisher for the "heartbeat" topic
        self.__heartbeat_publisher__ = self.create_publisher(Heartbeat, 'heartbeat', 10)
        
        # Create a timer that sends a heartbeat message every 0.5s
        self.__heartbeat_period__  = 0.5
        self.__heartbeat_timer__ = self.create_timer(self.__heartbeat_period__, self.__heartbeat_timer_callback__)
        
        # Tick counting upwards with every heartbeat
        self.__tick__ = 0
    
    def activate(self) -> None:
        """
        Activates the node.

        This method sets the `__node_active__` attribute to True, indicating that the node is active.
        """
        self.__node_active__ = True
        self.get_logger().debug('CommonNode::activate: Activated node')
    
    def deactivate(self) -> None:
        """
        Deactivates the node.

        Sets the `__node_active__` attribute to False, indicating that the node is no longer active.
        """
        self.__node_active__ = False
        self.get_logger().debug('CommonNode::deactivate: Deactivated node')
    
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
        msg.sender_id = self.get_fully_qualified_name()
        msg.active = self.__node_active__
        self.__tick__ += 1
        msg.tick = self.__tick__
        now = self.get_clock().now()
        msg.time_stamp = Time(sec=now.seconds_nanoseconds()[0], nanosec=now.seconds_nanoseconds()[1])
        self.__heartbeat_publisher__.publish(msg)
        self.get_logger().debug(f"CommonNode::__heartbeat_timer_callback__: Published heartbeat message with sender_id: {msg.sender_id}, tick: {msg.tick}, active: {msg.active}")
