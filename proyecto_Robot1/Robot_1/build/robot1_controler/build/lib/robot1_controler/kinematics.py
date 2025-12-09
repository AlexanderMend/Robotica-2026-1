#!/usr/bin/env python3
from sympy import *

import matplotlib.pyplot as plt

class RobotKinematics():
  def __init__(self):
    pass

  def direct_kinematics(self):
    print("Definiendo variables del modelo en sympy (RRR plano en XY)")

    # Variables articulares
    self.theta_0_1, self.theta_1_2, self.theta_2_3 = symbols("theta_0_1, theta_1_2, theta_2_3")

    # Longitudes de los eslabones (coinciden con el URDF)
    self.l1 = 0.3
    self.l2 = 0.3
    self.l3 = 0.3

    # Robot RRR plano en XY: todas las juntas giran alrededor de Z
    # y cada eslabón se extiende a lo largo del eje X de su sistema
    #
    # Estructura equivalente al URDF:
    # base_link --(theta_0_1)--> shoulder_link (l1)
    # shoulder_link --(theta_1_2)--> arm_link (l2)
    # arm_link --(theta_2_3)--> forearm_link (l3)

    # Matrices homogéneas
    # Solo usamos rotación alrededor de Z (alpha) y traslación en X, Y
    self.T_0_1 = self.trans_homo(0,        0, 0, 0, 0, self.theta_0_1)
    self.T_1_2 = self.trans_homo(self.l1,  0, 0, 0, 0, self.theta_1_2)
    self.T_2_3 = self.trans_homo(self.l2,  0, 0, 0, 0, self.theta_2_3)
    self.T_3_p = self.trans_homo(self.l3,  0, 0, 0, 0, 0)

    # Transformación total base -> efector final
    T_0_p = simplify(self.T_0_1 * self.T_1_2 * self.T_2_3 * self.T_3_p)

    # Posición del efector final en el plano XY
    x_0_p = T_0_p[0, 3]
    y_0_p = T_0_p[1, 3]

    # Orientación (yaw) respecto a la base
    th_0_p = self.theta_0_1 + self.theta_1_2 + self.theta_2_3

    # Vector de postura en el espacio de trabajo: [x, y, theta]
    self.xi_0_p = Matrix([
      [x_0_p],
      [y_0_p],
      [th_0_p]
    ])

    # Jacobiano geométrico 3x3
    self.J = Matrix.hstack(
      diff(self.xi_0_p, self.theta_0_1),
      diff(self.xi_0_p, self.theta_1_2),
      diff(self.xi_0_p, self.theta_2_3)
    )
    self.J_inv = self.J.inv()

    # Vector de velocidades del espacio de trabajo
    self.x_dot, self.y_dot, self.th_dot = symbols("x_dot, y_dot, th_dot")
    self.xi_dot = Matrix([[self.x_dot],
                          [self.y_dot],
                          [self.th_dot]])

    # Cinemática inversa diferencial: q_dot = J^{-1} * xi_dot
    self.q_dot = self.J_inv * self.xi_dot

    print("Definidas todas las variables para el robot RRR plano en XY")

  def trajectory_generator(self, q_in = [0.1, 0.1, 0.1], xi_fn = [0.8, 0.1, 0], duration = 4):
    self.freq = 30
    print("Definiendo trayectoria en el espacio de trabajo")

    # Polinomio de interpolación lambda (quintico)
    self.t, a0, a1, a2, a3, a4, a5 = symbols("t, a0, a1, a2, a3, a4, a5")
    self.lam = a0 + a1*self.t + a2*(self.t)**2 + a3*(self.t)**3 + a4*(self.t)**4 + a5*(self.t)**5

    # Derivadas de lambda
    self.lam_dot = diff(self.lam, self.t)
    self.lam_dot_dot = diff(self.lam_dot, self.t)

    # Condiciones de frontera para movimiento suave
    ec1 = self.lam.subs(self.t, 0)
    ec2 = self.lam.subs(self.t, duration) - 1
    ec3 = self.lam_dot.subs(self.t, 0)
    ec4 = self.lam_dot.subs(self.t, duration)
    ec5 = self.lam_dot_dot.subs(self.t, 0)
    ec6 = self.lam_dot_dot.subs(self.t, duration)

    terms = solve([ec1, ec2, ec3, ec4, ec5, ec6],
                  [a0, a1, a2, a3, a4, a5], dict = True)

    self.lam_s         = self.lam.subs(terms[0])
    self.lam_dot_s     = self.lam_dot.subs(terms[0])
    self.lam_dot_dot_s = self.lam_dot_dot.subs(terms[0])

    # Posición inicial del efector final en XYθ para las juntas iniciales q_in
    xi_in = self.xi_0_p.subs({
      self.theta_0_1: q_in[0],
      self.theta_1_2: q_in[1],
      self.theta_2_3: q_in[2]
    })

    # Trayectoria del efector final (posición)
    xi = xi_in + Matrix([
      [self.lam_s * (xi_fn[0] - xi_in[0])],
      [self.lam_s * (xi_fn[1] - xi_in[1])],
      [self.lam_s * (xi_fn[2] - xi_in[2])]
    ])

    # Trayectoria del efector final (velocidad)
    xi_dot = Matrix([
      [self.lam_dot_s * (xi_fn[0] - xi_in[0])],
      [self.lam_dot_s * (xi_fn[1] - xi_in[1])],
      [self.lam_dot_s * (xi_fn[2] - xi_in[2])]
    ])

    # Trayectoria del efector final (aceleración)
    xi_dot_dot = Matrix([
      [self.lam_dot_dot_s * (xi_fn[0] - xi_in[0])],
      [self.lam_dot_dot_s * (xi_fn[1] - xi_in[1])],
      [self.lam_dot_dot_s * (xi_fn[2] - xi_in[2])]
    ])

    # Muestreo temporal
    self.samples = int(self.freq * duration + 1)
    self.dt = 1/self.freq

    self.xi_m         = Matrix.zeros(3, self.samples)
    self.xi_dot_m     = Matrix.zeros(3, self.samples)
    self.xi_dot_dot_m = Matrix.zeros(3, self.samples)
    self.t_m          = Matrix.zeros(1, self.samples)

    self.t_m[0, 0] = 0
    for a in range(self.samples - 1):
      self.t_m[0, a+1] = self.t_m[0, a] + self.dt

    # Funciones lambda para evaluar xi, xi_dot, xi_dot_dot
    xi_lam         = lambdify([self.t], xi,         modules='numpy')
    xi_dot_lam     = lambdify([self.t], xi_dot,     modules='numpy')
    xi_dot_dot_lam = lambdify([self.t], xi_dot_dot, modules='numpy')

    for a in range(self.samples):
      self.xi_m[:, a]         = xi_lam(float(self.t_m[0, a]))
      self.xi_dot_m[:, a]     = xi_dot_lam(float(self.t_m[0, a]))
      self.xi_dot_dot_m[:, a] = xi_dot_dot_lam(float(self.t_m[0, a]))

    self.q_in = q_in

  def inverse_kinematics(self):
    print("Modelando cinemática inversa (diferencial)")

    # Arreglos de juntas
    self.q_m         = Matrix.zeros(3, self.samples)
    self.q_dot_m     = Matrix.zeros(3, self.samples)
    self.q_dot_dot_m = Matrix.zeros(3, self.samples)

    # Condiciones iniciales
    self.q_m[:, 0]         = Matrix([[self.q_in[0]], [self.q_in[1]], [self.q_in[2]]])
    self.q_dot_m[:, 0]     = Matrix.zeros(3, 1)
    self.q_dot_dot_m[:, 0] = Matrix.zeros(3, 1)

    # Función lambda para q_dot = J^{-1} * xi_dot
    self.q_dot_lam = lambdify(
      [self.x_dot, self.y_dot, self.th_dot,
       self.theta_0_1, self.theta_1_2, self.theta_2_3],
      self.q_dot,
      modules='numpy'
    )

    for a in range(self.samples - 1):
      # Integración de posiciones articulares
      self.q_m[:, a+1] = self.q_m[:, a] + self.q_dot_m[:, a] * self.dt

      # Velocidades articulares siguientes
      self.q_dot_m[:, a+1] = self.q_dot_lam(
        float(self.xi_dot_m[0, a]),
        float(self.xi_dot_m[1, a]),
        float(self.xi_dot_m[2, a]),
        float(self.q_m[0, a]),
        float(self.q_m[1, a]),
        float(self.q_m[2, a])
      )

      # Aceleraciones articulares
      self.q_dot_dot_m[:, a+1] = (self.q_dot_m[:, a+1] - self.q_dot_m[:, a]) / self.dt

    print("Trayectoria de las juntas generada")

  def ws_graph(self):
    fig, (p_g, v_g, a_g) = plt.subplots(nrows=1, ncols=3)
    fig.suptitle("Espacio de trabajo (x, y, theta)")

    p_g.set_title("Posiciones")
    p_g.plot(self.t_m.T, self.xi_m[0, :].T, color="RED",   label="x")
    p_g.plot(self.t_m.T, self.xi_m[1, :].T, color=(0, 1, 0), label="y")
    p_g.plot(self.t_m.T, self.xi_m[2, :].T, color="blue", label="theta")
    p_g.legend()

    v_g.set_title("Velocidades")
    v_g.plot(self.t_m.T, self.xi_dot_m[0, :].T, color="RED")
    v_g.plot(self.t_m.T, self.xi_dot_m[1, :].T, color=(0, 1, 0))
    v_g.plot(self.t_m.T, self.xi_dot_m[2, :].T, color="blue")

    a_g.set_title("Aceleraciones")
    a_g.plot(self.t_m.T, self.xi_dot_dot_m[0, :].T, color="RED")
    a_g.plot(self.t_m.T, self.xi_dot_dot_m[1, :].T, color=(0, 1, 0))
    a_g.plot(self.t_m.T, self.xi_dot_dot_m[2, :].T, color="blue")

    plt.show()

  def q_graph(self):
    fig, (p_g, v_g, a_g) = plt.subplots(nrows=1, ncols=3)
    fig.suptitle("Espacio de las juntas")

    p_g.set_title("Posiciones")
    p_g.plot(self.t_m.T, self.q_m[0, :].T, color="RED",   label="q1")
    p_g.plot(self.t_m.T, self.q_m[1, :].T, color=(0, 1, 0), label="q2")
    p_g.plot(self.t_m.T, self.q_m[2, :].T, color="blue", label="q3")
    p_g.legend()

    v_g.set_title("Velocidades")
    v_g.plot(self.t_m.T, self.q_dot_m[0, :].T, color="RED")
    v_g.plot(self.t_m.T, self.q_dot_m[1, :].T, color=(0, 1, 0))
    v_g.plot(self.t_m.T, self.q_dot_m[2, :].T, color="blue")

    a_g.set_title("Aceleraciones")
    a_g.plot(self.t_m.T, self.q_dot_dot_m[0, :].T, color="RED")
    a_g.plot(self.t_m.T, self.q_dot_dot_m[1, :].T, color=(0, 1, 0))
    a_g.plot(self.t_m.T, self.q_dot_dot_m[2, :].T, color="blue")

    plt.show()

  def simple_graph(self, val_m, t_m):
    plt.plot(t_m.T, val_m[0, :].T)
    plt.show()

  def trans_homo_xz(self, x=0, z=0, gamma=0, alpha=0)->Matrix:
    # Se deja por compatibilidad, pero ya no se usa en el modelo plano XY
    R_z = Matrix([[cos(alpha), -sin(alpha), 0],
                  [sin(alpha),  cos(alpha), 0],
                  [0,           0,          1]])
    R_x = Matrix([[1, 0, 0],
                  [0, cos(gamma), -sin(gamma)],
                  [0, sin(gamma),  cos(gamma)]])

    p_x = Matrix([[x],[0],[0]])
    p_z = Matrix([[0],[0],[z]])

    T_x = Matrix.vstack(Matrix.hstack(R_x, p_x), Matrix([[0,0,0,1]]))
    T_z = Matrix.vstack(Matrix.hstack(R_z, p_z), Matrix([[0,0,0,1]]))
    return T_x * T_z

  def trans_homo(self, x, y, z, gamma, beta, alpha):
    # Rotaciones Rz(alpha)*Ry(beta)*Rx(gamma) con traslaciones en x, y, z
    R_z = Matrix([[cos(alpha), -sin(alpha), 0],
                  [sin(alpha),  cos(alpha), 0],
                  [0,           0,          1]])
    R_y = Matrix([[cos(beta), 0, sin(beta)],
                  [0,         1, 0],
                  [-sin(beta),0, cos(beta)]])
    R_x = Matrix([[1, 0, 0],
                  [0, cos(gamma), -sin(gamma)],
                  [0, sin(gamma),  cos(gamma)]])

    p_x = Matrix([[x],[0],[0]])
    p_y = Matrix([[0],[y],[0]])
    p_z = Matrix([[0],[0],[z]])

    T_x = Matrix.vstack(Matrix.hstack(R_x, p_x), Matrix([[0,0,0,1]]))
    T_y = Matrix.vstack(Matrix.hstack(R_y, p_y), Matrix([[0,0,0,1]]))
    T_z = Matrix.vstack(Matrix.hstack(R_z, p_z), Matrix([[0,0,0,1]]))
    return T_x * T_y * T_z

  def redirect_print(self, new_print):
    global print
    print = new_print

def main():
  robot = RobotKinematics()
  robot.direct_kinematics()
  robot.trajectory_generator()
  robot.inverse_kinematics()
  robot.ws_graph()
  robot.q_graph()

if __name__ == "__main__":
  main()