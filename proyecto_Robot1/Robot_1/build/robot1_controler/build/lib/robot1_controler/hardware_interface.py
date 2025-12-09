#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class HardwareInterface(Node):
  def __init__(self):
    super().__init__("hardware_interface")
    # Inicializar las comunicaciones
    # --- 
    # Recibir información de posiciones deseadas
    self.joint_hardware_objectives_subscriber = self.create_subscription(
      JointState, "/joint_hardware_objectives", self.hardware_obj_callback, 10
    )
    # Retroalimentación de las posiciones actuales del robot
    self.joint_states_publisher = self.create_publisher(
      JointState, "/joint_states", 10
    )
    # Mensaje que contiene la posición actual del robot
    self.current_joint_state = JointState()
    self.current_joint_state.header.stamp = self.get_clock().now().to_msg()
    self.current_joint_state.name = ["shoulder_joint", "arm_joint", "forearm_joint"]
    self.current_joint_state.position = [0.1, 0.1, 0.1]
    self.create_timer(0.1, self.joint_states_timer_callback)

  def hardware_obj_callback(self, msg:JointState):
    # En esta parte se mandaría a través de alguna comunicación 
    # la información al hardware
    # Aquí estamos asignando los comandos como si fueran la posición real del robot
    self.current_joint_state = msg
    pass

  def joint_states_timer_callback(self):
    msg = JointState()
    # En esta parte recibiría la información del estado del robot
    # ---
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = ["shoulder_joint", "arm_joint", "forearm_joint"]
    msg.position = [0.1, 0.1, 0.1]
    # self.joint_states_publisher.publish(msg)
    self.current_joint_state.header.stamp = self.get_clock().now().to_msg()
    self.joint_states_publisher.publish(self.current_joint_state)

def main(args=None):
  try:
    rclpy.init(args=args)
    node = HardwareInterface()
    rclpy.spin(node)
  except KeyboardInterrupt as e:
    print("Node stopped")
  finally: 
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()
      
