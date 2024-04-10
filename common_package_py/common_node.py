import rclpy
from rclpy.node import Node

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
        self._active_ = False
        
        # Create a publisher for the "heartbeat" topic
        self.__publisher__ = self.create_publisher(Heartbeat, 'heartbeat', 10)
        
        # Create a timer that sends a heartbeat message every 0.5s
        self.__timer__ = self.create_timer(0.5, self.__timer_callback__)
        
        # Tick counting upwards with every heartbeat
        self.__tick__ = 0

    def __timer_callback__(self):
        """
        This method is called by the timer to publish a heartbeat message.

        It creates a Heartbeat message, sets the sender ID, active status, and tick count,
        and publishes the message using the publisher.
        """
        
        msg = Heartbeat()
        msg.sender_id = self.get_fully_qualified_name()
        msg.active = self._active_
        self.__tick__ += 1
        msg.tick = self.__tick__
        self.__publisher__.publish(msg)
