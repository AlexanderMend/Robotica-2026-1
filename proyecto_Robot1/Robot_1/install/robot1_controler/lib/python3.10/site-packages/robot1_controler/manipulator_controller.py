#!/usr/bin/env python3
import math
import traceback
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import JointState

from .kinematics import RobotKinematics
from .dynamics import RobotDynamics


class ManipulatorController(Node):
    def __init__(self):
        super().__init__("manipulator_controller")

        # ---------------- CINEMÁTICA ----------------
        try:
            self.kinematics = RobotKinematics()
        except Exception as e:
            self.get_logger().fatal(f"Error inicializando RobotKinematics: {e}")
            self.get_logger().debug(traceback.format_exc())
            raise

        try:
            # Redirigir prints de la cinemática (si existe)
            try:
                self.kinematics.redirect_print(self.get_logger().info)
            except AttributeError:
                pass
            # Inicializaciones internas de la cinemática
            self.kinematics.direct_kinematics()
        except Exception as e:
            self.get_logger().fatal(f"Error en inicialización de cinemática: {e}")
            self.get_logger().debug(traceback.format_exc())
            raise

        # ---------------- DINÁMICA ----------------
        try:
            self.dynamics = RobotDynamics()
            try:
                self.dynamics.redirect_print(self.get_logger().info)
            except AttributeError:
                pass
            self.dynamics.define_kinematics(self.kinematics)
            self.dynamics.define_dynamics()
        except Exception as e:
            self.get_logger().fatal(f"Error inicializando Dinámica: {e}")
            self.get_logger().debug(traceback.format_exc())
            raise

        # Bandera para no solapar trayectorias
        self.moving = False

        # Guard for active timer
        self.position_publisher_timer = None

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

    # ------------------ HELPERS ------------------

    def _is_finite(self, *values):
        """Verifica que todos los valores sean finitos (no NaN/Inf)."""
        return all((isinstance(v, (int, float)) and math.isfinite(v)) for v in values)

    def _cancel_timer_if_exists(self):
        if self.position_publisher_timer is not None:
            try:
                self.position_publisher_timer.cancel()
            except Exception:
                self.get_logger().warning("Error cancelando timer", exc_info=True)
            self.position_publisher_timer = None

    # ------------------------------------------------------------------
    #  CALLBACKS DE ENTRADA
    # ------------------------------------------------------------------

    def end_effector_callback(self, msg: Twist):
        """Objetivo en espacio de trabajo vía /end_effector_goal."""
        try:
            if self.moving:
                self.get_logger().warning("Ya hay una trayectoria en ejecución, mensaje ignorado")
                return

            if not hasattr(self, "current_joint_states"):
                self.get_logger().warning("Aún no se recibe /joint_states, mensaje ignorado")
                return

            x = msg.linear.x
            y = msg.linear.y
            alpha = msg.angular.z

            if not self._is_finite(x, y, alpha):
                self.get_logger().error("Objetivo no válido (NaN/Inf). Ignorado.")
                return

            self.moving = True
            self.get_logger().info(
                f"Objetivo Twist recibido: x={x:.3f}, y={y:.3f}, alpha={alpha:.3f}"
            )

            q_in = list(self.current_joint_states.position)
            xi_fn = [x, y, alpha]

            # 1) Trayectoria en espacio de trabajo
            try:
                self.kinematics.trajectory_generator(q_in=q_in, xi_fn=xi_fn, duration=4.0)
            except Exception as e:
                self.get_logger().error(f"Error en trajectory_generator: {e}")
                self.get_logger().debug(traceback.format_exc())
                self.moving = False
                return

            # 2) Cinemática inversa diferencial
            try:
                self.kinematics.inverse_kinematics()
            except Exception as e:
                self.get_logger().error(f"Error en inverse_kinematics: {e}")
                self.get_logger().debug(traceback.format_exc())
                self.moving = False
                return

            # (Opcional) Gráficas
            try:
                self.kinematics.ws_graph()
                self.kinematics.q_graph()
            except Exception as e:
                self.get_logger().warning(f"No se pudieron mostrar gráficas: {e}")

            # 3) Preparar publicación de trayectoria articular
            try:
                self._prepare_trajectory_timer()
            except Exception as e:
                self.get_logger().error(f"Error preparando timer de trayectoria: {e}")
                self.get_logger().debug(traceback.format_exc())
                self.moving = False
                return

        except Exception as e:
            self.get_logger().error(f"Excepción inesperada en end_effector_callback: {e}")
            self.get_logger().debug(traceback.format_exc())
            self.moving = False

    def clicked_point_callback(self, msg: PointStamped):
        """Objetivo usando Publish Point de RViz en /clicked_point."""
        try:
            if self.moving:
                self.get_logger().warning("Ya hay una trayectoria en ejecución, mensaje ignorado")
                return

            if not hasattr(self, "current_joint_states"):
                self.get_logger().warning("Aún no se recibe /joint_states, mensaje ignorado")
                return

            x = msg.point.x
            y = msg.point.y

            if not self._is_finite(x, y):
                self.get_logger().error("Punto no válido (NaN/Inf). Ignorado.")
                return

            # (Opcional) comprobación de alcance simple si kinematics tiene link_lengths
            max_reach = None
            try:
                max_reach = sum(self.kinematics.link_lengths)
            except Exception:
                max_reach = None

            dist = math.hypot(x, y)
            if max_reach is not None and dist > max_reach + 1e-6:
                self.get_logger().warning(f"Punto fuera de alcance (r={dist:.3f} > {max_reach:.3f}). Ignorado.")
                return

            self.moving = True
            self.get_logger().info(f"Punto clickeado: x={x:.3f}, y={y:.3f}")

            q_in = list(self.current_joint_states.position)
            xi_fn = [x, y, 0.0]

            # 1) Trayectoria en espacio de trabajo
            try:
                self.kinematics.trajectory_generator(q_in=q_in, xi_fn=xi_fn, duration=4.0)
            except Exception as e:
                self.get_logger().error(f"Error en trajectory_generator: {e}")
                self.get_logger().debug(traceback.format_exc())
                self.moving = False
                return

            # 2) Cinemática inversa
            try:
                self.kinematics.inverse_kinematics()
            except Exception as e:
                self.get_logger().error(f"Error en inverse_kinematics: {e}")
                self.get_logger().debug(traceback.format_exc())
                self.moving = False
                return

            # 3) Dinámica (pares articulares)
            try:
                self.dynamics.lagrange_effort_generator()
            except Exception as e:
                self.get_logger().warning(f"Error generando dinámica de Lagrange: {e}")
                self.get_logger().debug(traceback.format_exc())

            # 4) Gráficas (opcionales)
            try:
                self.kinematics.ws_graph()
                self.kinematics.q_graph()
                self.dynamics.effort_graph()
            except Exception as e:
                self.get_logger().warning(f"No se pudieron mostrar algunas gráficas: {e}")

            # 5) Preparar publicación de trayectoria articular
            try:
                self._prepare_trajectory_timer()
            except Exception as e:
                self.get_logger().error(f"Error preparando timer de trayectoria: {e}")
                self.get_logger().debug(traceback.format_exc())
                self.moving = False
                return

        except Exception as e:
            self.get_logger().error(f"Excepción inesperada en clicked_point_callback: {e}")
            self.get_logger().debug(traceback.format_exc())
            self.moving = False

    def joint_states_callback(self, msg: JointState):
        """Guarda el estado actual de las juntas."""
        # Validar estructura mínima del mensaje
        if not isinstance(msg, JointState) or len(msg.position) == 0:
            self.get_logger().warning("JointState recibido inválido o vacío, ignorado")
            return
        self.current_joint_states = msg

    # ------------------------------------------------------------------
    #  PUBLICACIÓN DE TRAYECTORIA A /joint_goals
    # ------------------------------------------------------------------

    def _prepare_trajectory_timer(self):
        """Inicializa el timer para publicar q_m(t) en /joint_goals."""
        # Cancelar timer anterior si existe
        self._cancel_timer_if_exists()

        # Validar que kinematics haya generado q_m
        if not hasattr(self.kinematics, "q_m") or self.kinematics.q_m is None:
            raise RuntimeError("q_m no está definido por la cinemática (trajectory_generator/inverse_kinematics falló).")

        q_m = getattr(self.kinematics, "q_m")
        # Esperamos una estructura (n_joints x samples)
        if not hasattr(q_m, "shape") or len(q_m.shape) != 2:
            raise RuntimeError("q_m tiene formato inesperado. Debe ser un array 2D (n_joints x samples).")

        # Determinar número de muestras
        try:
            samples = int(getattr(self.kinematics, "samples"))
        except Exception:
            samples = int(q_m.shape[1])

        if samples <= 0:
            raise RuntimeError("Número de muestras inválido para la trayectoria.")

        # Guardar muestras y dt en kinematics si hace falta
        self.kinematics.samples = samples
        if not hasattr(self.kinematics, "dt") or self.kinematics.dt is None:
            self.kinematics.dt = 0.02  # valor por defecto seguro (50Hz)

        self.count = 0
        self.joint_goals = JointState()

        # Nombres de juntas (debes usarlos igual que en el URDF / hardware_interface)
        self.joint_goals.name = [
            "shoulder_joint",
            "arm_joint",
            "forearm_joint"
        ]

        self.get_logger().info("Iniciando publicación de trayectoria articular para Robot 1")

        # Crear el timer y guardar referencia
        self.position_publisher_timer = self.create_timer(
            float(self.kinematics.dt),
            self.trajectory_publisher_callback
        )

    def trajectory_publisher_callback(self):
        """Publica una muestra de la trayectoria articular en /joint_goals."""
        if not hasattr(self.kinematics, "q_m") or self.kinematics.q_m is None:
            self.get_logger().warning("q_m no disponible al publicar, cancelando timer.")
            self._cancel_timer_if_exists()
            self.moving = False
            return

        # Time stamp
        try:
            self.joint_goals.header.stamp = self.get_clock().now().to_msg()
        except Exception:
            pass

        try:
            th1 = float(self.kinematics.q_m[0, self.count])
            th2 = float(self.kinematics.q_m[1, self.count])
            th3 = float(self.kinematics.q_m[2, self.count])
        except IndexError:
            self.get_logger().warning("Índice fuera de rango al publicar trayectoria, cancelando")
            self._cancel_timer_if_exists()
            self.moving = False
            return
        except Exception as e:
            self.get_logger().error(f"Error leyendo q_m: {e}")
            self.get_logger().debug(traceback.format_exc())
            self._cancel_timer_if_exists()
            self.moving = False
            return

        # Validar ángulos (finito)
        if not self._is_finite(th1, th2, th3):
            self.get_logger().error("Ángulos de trayectoria no finitos, cancelando")
            self._cancel_timer_if_exists()
            self.moving = False
            return

        self.joint_goals.position = [th1, th2, th3]
        try:
            self.joint_goals_publisher.publish(self.joint_goals)
        except Exception as e:
            self.get_logger().warning(f"Fallo publicando joint_goals: {e}")
            self.get_logger().debug(traceback.format_exc())

        self.count += 1

        if self.count >= self.kinematics.samples:
            self.get_logger().info("Trayectoria articular finalizada")
            self._cancel_timer_if_exists()
            self.moving = False

    def destroy_node(self):
        # Asegurar cancelación de timers antes de destruir el nodo
        try:
            self._cancel_timer_if_exists()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().fatal(f"Nodo finalizado por excepción: {e}")
        node.get_logger().debug(traceback.format_exc())
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
