#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class ControlManager(Node):
  def __init__(self):
    super().__init__("control_manager")
    # Recibir estados del robot
    self.joint_state_subscriber = self.create_subscription(
      JointState, "/joint_states",self.joint_state_callback, 10
    )
    #Recibir comandos del controlador
    self.joint_goals_subscriber = self.create_subscription(
      JointState, "/joint_goals",self.joint_goal_callback, 10
    )
    # Enviar información a la interfaz de hardware
    self.hardware_command_publisher = self.create_publisher(
      JointState, "/joint_hardware_objectives", 10
    )
  def joint_state_callback(self, msg:JointState):
    """self.get_logger().info("Robot current position: \n{}: {} \n{}: {} \n{}: {}".format(
      msg.name[0], msg.position[0],
      msg.name[1], msg.position[1],
      msg.name[2], msg.position[2]
    ))"""
    pass

  def joint_goal_callback(self, msg:JointState):
    # En esta sección, se envían comandos a a la interfaz de hardware 
    # En este ejemplo sencillo, envía el mensaje tal cual lo recibe
    self.hardware_command_publisher.publish(msg)
    pass

def main(args=None):
  try:
    rclpy.init(args=args)
    node = ControlManager()
    rclpy.spin(node)
  except KeyboardInterrupt as e:
    print("Node stopped")
  finally: 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()