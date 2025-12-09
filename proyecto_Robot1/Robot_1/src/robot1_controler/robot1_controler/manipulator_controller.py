#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import JointState

from .kinematics import RobotKinematics
from .dynamics import RobotDynamics


class ManipulatorController(Node):
    """
    Controlador para el Robot 1 (RRR en plano XY).

    Usa:
      - Robot1Kinematics  -> espacio de trabajo [x, y, alpha]
      - Robot1Dynamics    -> pares articulares τ(q, q_dot, q_ddot)

    Topics:
      Sub:
        /end_effector_goal (Twist)   -> objetivo en espacio de trabajo
        /clicked_point     (PointStamped) -> objetivo usando Publish Point de RViz
        /joint_states      (JointState)   -> estado actual de las juntas
      Pub:
        /joint_goals       (JointState)   -> trayectoria de juntas para el robot
    """

    def __init__(self):
        super().__init__("manipulator_controller")

        # ---------------- CINEMÁTICA ----------------
        self.kinematics = RobotKinematics()
        # Redirigir print de la cinemática al logger (si existe redirect_print)
        try:
            self.kinematics.redirect_print(self.get_logger().info)
        except AttributeError:
            pass

        self.kinematics.direct_kinematics()

        # ---------------- DINÁMICA ----------------
        self.dynamics = RobotDynamics()
        try:
            self.dynamics.redirect_print(self.get_logger().info)
        except AttributeError:
            pass

        # Enlazar dinámica con la cinemática y definir parámetros
        self.dynamics.define_kinematics(self.kinematics)
        self.dynamics.define_dynamics()

        # Bandera para no solapar trayectorias
        self.moving = False

        # Suscripciones
        self.end_effector_goal_subscriber = self.create_subscription(
            Twist,
            "/end_effector_goal",
            self.end_effector_callback,
            10
        )

        self.clicked_point_subscriber = self.create_subscription(
            PointStamped,
            "/clicked_point",
            self.clicked_point_callback,
            10
        )

        self.joint_states_subscriber = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_callback,
            10
        )

        # Publicador de objetivos de juntas
        self.joint_goals_publisher = self.create_publisher(
            JointState,
            "/joint_goals",
            10
        )

        self.get_logger().info("ManipulatorController para Robot 1 inicializado")

    # ------------------------------------------------------------------
    #  CALLBACKS DE ENTRADA
    # ------------------------------------------------------------------

    def end_effector_callback(self, msg: Twist):
        """
        Objetivo en espacio de trabajo vía /end_effector_goal.

        Interpretación:
          x     = msg.linear.x
          y     = msg.linear.y
          alpha = msg.angular.z
        """
        if self.moving:
            self.get_logger().warning("Ya hay una trayectoria en ejecución, mensaje ignorado")
            return

        if not hasattr(self, "current_joint_states"):
            self.get_logger().warning("Aún no se recibe /joint_states, mensaje ignorado")
            return

        self.moving = True
        self.get_logger().info(
            f"Objetivo Twist recibido: x={msg.linear.x:.3f}, y={msg.linear.y:.3f}, alpha={msg.angular.z:.3f}"
        )

        # Posición articular actual como condición inicial
        q_in = list(self.current_joint_states.position)
        # Objetivo en espacio de trabajo
        xi_fn = [msg.linear.x, msg.linear.y, msg.angular.z]

        # 1) Trayectoria en espacio de trabajo
        self.kinematics.trajectory_generator(
            q_in=q_in,
            xi_fn=xi_fn,
            duration=4.0
        )

        # 2) Cinemática inversa diferencial
        self.kinematics.inverse_kinematics()

        # (Opcional) Gráficas de cinemática
        try:
            self.kinematics.ws_graph()
            self.kinematics.q_graph()
        except Exception as e:
            self.get_logger().warning(f"No se pudieron mostrar gráficas de cinemática: {e}")

        # 3) Preparar publicación de trayectoria articular
        self._prepare_trajectory_timer()

    def clicked_point_callback(self, msg: PointStamped):
        """
        Objetivo usando Publish Point de RViz en /clicked_point.

        Interpretación:
          x     = msg.point.x
          y     = msg.point.y
          alpha = 0.0 (orientación fija)
        """
        if self.moving:
            self.get_logger().warning("Ya hay una trayectoria en ejecución, mensaje ignorado")
            return

        if not hasattr(self, "current_joint_states"):
            self.get_logger().warning("Aún no se recibe /joint_states, mensaje ignorado")
            return

        self.moving = True
        self.get_logger().info(
            f"Punto clickeado: x={msg.point.x:.3f}, y={msg.point.y:.3f}"
        )

        q_in = list(self.current_joint_states.position)
        xi_fn = [msg.point.x, msg.point.y, 0.0]

        # 1) Trayectoria en espacio de trabajo
        self.kinematics.trajectory_generator(
            q_in=q_in,
            xi_fn=xi_fn,
            duration=4.0
        )

        # 2) Cinemática inversa
        self.kinematics.inverse_kinematics()

        # 3) Dinámica (pares articulares)
        try:
            self.dynamics.lagrange_effort_generator()
        except Exception as e:
            self.get_logger().warning(f"Error generando dinámica de Lagrange: {e}")

        # 4) Gráficas de trabajo, juntas y esfuerzos
        try:
            self.kinematics.ws_graph()
            self.kinematics.q_graph()
            self.dynamics.effort_graph()
        except Exception as e:
            self.get_logger().warning(f"No se pudieron mostrar algunas gráficas: {e}")

        # 5) Preparar publicación de trayectoria articular
        self._prepare_trajectory_timer()

    def joint_states_callback(self, msg: JointState):
        """Guarda el estado actual de las juntas."""
        self.current_joint_states = msg

    # ------------------------------------------------------------------
    #  PUBLICACIÓN DE TRAYECTORIA A /joint_goals
    # ------------------------------------------------------------------

    def _prepare_trajectory_timer(self):
        """Inicializa el timer para publicar q_m(t) en /joint_goals."""
        self.count = 0
        self.joint_goals = JointState()

        # Nombres de juntas (debes usarlos igual que en el URDF / hardware_interface)
        self.joint_goals.name = [
            "shoulder_joint",
            "arm_joint",
            "forearm_joint"
        ]

        self.get_logger().info("Iniciando publicación de trayectoria articular para Robot 1")

        self.position_publisher_timer = self.create_timer(
            float(self.kinematics.dt),
            self.trajectory_publisher_callback
        )

    def trajectory_publisher_callback(self):
        """Publica una muestra de la trayectoria articular en /joint_goals."""
        # Time stamp
        self.joint_goals.header.stamp = self.get_clock().now().to_msg()

        try:
            th1 = float(self.kinematics.q_m[0, self.count])
            th2 = float(self.kinematics.q_m[1, self.count])
            th3 = float(self.kinematics.q_m[2, self.count])
        except IndexError:
            self.get_logger().warning("Índice fuera de rango al publicar trayectoria, cancelando")
            self.position_publisher_timer.cancel()
            self.moving = False
            return

        self.joint_goals.position = [th1, th2, th3]
        self.joint_goals_publisher.publish(self.joint_goals)

        self.count += 1

        if self.count >= self.kinematics.samples:
            self.get_logger().info("Trayectoria articular finalizada")
            self.position_publisher_timer.cancel()
            self.moving = False


def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()