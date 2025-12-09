#!/usr/bin/env python3
from sympy import *
from .kinematics import RobotKinematics

import matplotlib.pyplot as plt

class RobotDynamics():
  def __init__(self):
    pass
  def define_kinematics(self, kinematics:RobotKinematics):
    self.kinematics = kinematics
  def define_dynamics(self, mass = [0.25, 0.25, 0.25]):
    print("Definiendo variables en sympy para dinámica")
    # Transformaciones de centros de masa
    self.kinematics.T_0_2 = self.kinematics.T_0_1 * self.kinematics.T_1_2
    self.kinematics.T_0_3 = self.kinematics.T_0_2 * self.kinematics.T_2_3
    self.kinematics.T_1_C1 = self.kinematics.trans_homo(self.kinematics.l1 / 2, 0, 0, 0, 0, 0)
    self.kinematics.T_2_C2 = self.kinematics.trans_homo(self.kinematics.l2 / 2, 0, 0, 0, 0, 0)
    self.kinematics.T_3_C3 = self.kinematics.trans_homo(self.kinematics.l3 / 2, 0, 0, 0, 0, 0)
    self.kinematics.T_0_C1 = simplify(self.kinematics.T_0_1 * self.kinematics.T_1_C1)
    self.kinematics.T_0_C2 = simplify(self.kinematics.T_0_2 * self.kinematics.T_2_C2)
    self.kinematics.T_0_C3 = simplify(self.kinematics.T_0_3 * self.kinematics.T_3_C3)
    #Rotaciones
    self.kinematics.R_0_1 = self.kinematics.T_0_1[:3, :3]
    self.kinematics.R_1_2 = self.kinematics.T_1_2[:3, :3]
    self.kinematics.R_2_3 = self.kinematics.T_2_3[:3, :3]
    #Vectores de posición de sistemas de referencia
    self.p_0_1 = Matrix([[self.kinematics.l1], [0], [0]]) 
    self.p_1_2 = Matrix([[self.kinematics.l2], [0], [0]]) 
    self.p_2_3 = Matrix([[self.kinematics.l3], [0], [0]]) 
    #Vectores de posición de centros de masa
    self.p_1_C1 = Matrix([[self.kinematics.l1 / 2], [0], [0]]) 
    self.p_2_C2 = Matrix([[self.kinematics.l2 / 2], [0], [0]]) 
    self.p_3_C3 = Matrix([[self.kinematics.l3 / 2], [0], [0]]) 
    #Vectores de posición absoluta de centros de masa (para energía potencial)
    self.p_0_C1 = self.kinematics.T_0_C1[:3, 3]
    self.p_0_C2 = self.kinematics.T_0_C2[:3, 3]
    self.p_0_C3 = self.kinematics.T_0_C3[:3, 3]
    
    # Variables de velocidad angular
    self.theta_0_1_dot = Symbol('theta_0_1_dot')
    self.theta_1_2_dot = Symbol('theta_1_2_dot')
    self.theta_2_3_dot = Symbol('theta_2_3_dot')
    # Variables de aceleración angular
    self.theta_0_1_dot_dot = Symbol('theta_0_1_dot_dot')
    self.theta_1_2_dot_dot = Symbol('theta_1_2_dot_dot')
    self.theta_2_3_dot_dot = Symbol('theta_2_3_dot_dot')
    # Masas
    self.m1 = mass[0]
    self.m2 = mass[1]
    self.m3 = mass[2]
    # Matrices de inercia
    self.Ic1 = self.inertia_tensor(self.kinematics.l1, 0.03, 0.03, self.m1)
    self.Ic2 = self.inertia_tensor(self.kinematics.l2, 0.03, 0.03, self.m2)
    self.Ic3 = self.inertia_tensor(self.kinematics.l3, 0.03, 0.03, self.m3)
    #Gravedad
    self.g = -9.81

  def lagrange_effort_generator(self):
    print("Generando propagación de velocidad")
    #Velocidades angulares de sistemas
    omega_0_0 = Matrix([[0], [0], [0]])
    omega_1_1 = self.kinematics.R_0_1.transpose() * (omega_0_0 + Matrix([[0], [0], [self.theta_0_1_dot]]))
    omega_2_2 = self.kinematics.R_1_2.transpose() * (omega_1_1 + Matrix([[0], [0], [self.theta_1_2_dot]]))
    omega_3_3 = self.kinematics.R_2_3.transpose() * (omega_2_2 + Matrix([[0], [0], [self.theta_2_3_dot]]))
    #Velocidades angulares de centros de masa
    omega_1_C1 = omega_1_1
    omega_2_C2 = omega_2_2
    omega_3_C3 = omega_3_3
    #Velocidades lineales de sistemas
    v_1_1 = Matrix([[0], [0], [0]])
    v_2_2 = self.kinematics.R_1_2.transpose() * (v_1_1 + omega_1_1.cross(self.p_1_2))
    v_3_3 = self.kinematics.R_2_3.transpose() * (v_2_2 + omega_2_2.cross(self.p_2_3))
    #Velocidades lineales de centros de masa
    v_1_C1 = v_1_1 + omega_1_C1.cross(self.p_1_C1)
    v_2_C2 = v_2_2 + omega_2_C2.cross(self.p_2_C2)
    v_3_C3 = v_3_3 + omega_3_C3.cross(self.p_3_C3)
    print("Generando ecuaciones de lagrange")
    #Euler-Lagrange
    #Energía cinética
    k1 = 0.5 * self.m1 * v_1_C1.dot(v_1_C1) + 0.5 * omega_1_C1.dot(self.Ic1*omega_1_C1)
    k2 = 0.5 * self.m2 * v_2_C2.dot(v_2_C2) + 0.5 * omega_2_C2.dot(self.Ic2*omega_2_C2)
    k3 = 0.5 * self.m3 * v_3_C3.dot(v_3_C3) + 0.5 * omega_3_C3.dot(self.Ic3*omega_3_C3)
    k = k1 + k2 + k3
    #Energía potencial
    u1 = - self.m1 * Matrix([0, 0, self.g]).dot(self.p_0_C1)
    u2 = - self.m2 * Matrix([0, 0, self.g]).dot(self.p_0_C2)
    u3 = - self.m3 * Matrix([0, 0, self.g]).dot(self.p_0_C3)
    u = u1 + u2 + u3
    #Lagrangiano
    La = k - u
    #Reasignando variables de las juntas a la clase actual
    self.theta_0_1 = self.kinematics.theta_0_1
    self.theta_1_2 = self.kinematics.theta_1_2
    self.theta_2_3 = self.kinematics.theta_2_3
    #Derivadas respecto al espacio de trabajo
    La_dot_q = Matrix([[diff(La, self.theta_0_1)], 
                       [diff(La, self.theta_1_2)], 
                       [diff(La, self.theta_2_3)]])
    #Derivadas respecto a la derivada del espacio de trabajo
    La_dot_q_dot = Matrix([[diff(La, self.theta_0_1_dot)], 
                            [diff(La, self.theta_1_2_dot)], 
                            [diff(La, self.theta_2_3_dot)]])
    #Derivada total
    La_dot_q_dot_dt = (diff(La_dot_q_dot, self.theta_0_1) * self.theta_0_1_dot + 
                       diff(La_dot_q_dot, self.theta_1_2) * self.theta_1_2_dot + 
                       diff(La_dot_q_dot, self.theta_2_3) * self.theta_2_3_dot + 
                       diff(La_dot_q_dot, self.theta_0_1_dot) * self.theta_0_1_dot_dot + 
                       diff(La_dot_q_dot, self.theta_1_2_dot) * self.theta_1_2_dot_dot + 
                       diff(La_dot_q_dot, self.theta_2_3_dot) * self.theta_2_3_dot_dot)
    #Pares en las juntas
    tau = La_dot_q_dot_dt - La_dot_q
    tau_f = lambdify([self.theta_0_1,         self.theta_1_2,         self.theta_2_3, 
                      self.theta_0_1_dot,     self.theta_1_2_dot,     self.theta_2_3_dot, 
                      self.theta_0_1_dot_dot, self.theta_1_2_dot_dot, self.theta_2_3_dot_dot], tau)
    #Generar valores numéricos
    self.tau_m = Matrix.zeros(3, self.kinematics.samples)
    #Ciclo para todas las muestras
    print("Muestreando las ec. de lagrange")
    for i in range(self.kinematics.samples):
      self.tau_m[:, i] = tau_f( float(self.kinematics.q_m[0, i]),         float(self.kinematics.q_m[1, i]),          float(self.kinematics.q_m[2, i]),
                                float(self.kinematics.q_dot_m[0, i]),     float(self.kinematics.q_dot_m[1, i]),      float(self.kinematics.q_dot_m[2, i]), 
                                float(self.kinematics.q_dot_dot_m[0, i]), float(self.kinematics.q_dot_dot_m[1, i]),  float(self.kinematics.q_dot_dot_m[2, i]))
      """self.tau_val[:, i] = tau.subs({self.theta_0_1 : self.q_m[0, i], self.theta_1_2: self.q_m[1, i], self.theta_2_3 : self.q_m[2, i],
                            self.theta_0_1_dot:self.q_dot_m[0, i], self.theta_1_2_dot:self.q_dot_m[1, i], self.theta_2_3_dot:self.q_dot_m[2, i], 
                            self.theta_0_1_dot_dot:self.q_dot_dot_m[0, i], self.theta_1_2_dot_dot:self.q_dot_dot_m[1, i], self.theta_2_3_dot_dot:self.q_dot_dot_m[2, i]})"""
      #print(i)  

  def effort_graph(self):
    fig, ((tau_1_g, tau_2_g, tau_3_g)) = plt.subplots(nrows=1, ncols = 3)
    fig.suptitle("Pares en las juntas")
    # Posiciones ws
    tau_1_g.set_title("Esfuerzo junta 1")
    tau_1_g.plot(self.kinematics.t_m.T, self.tau_m[0, :].T, color = "RED")

    # Velocidades ws
    tau_2_g.set_title("Esfuerzo junta 2")
    tau_2_g.plot(self.kinematics.t_m.T, self.tau_m[1, :].T, color = "GREEN")

    # Aceleraciones ws
    tau_3_g.set_title("Esfuerzo junta 3")
    tau_3_g.plot(self.kinematics.t_m.T, self.tau_m[2, :].T, color = "BLUE")
    plt.show()

  def redefine_kinematics(self):
    self.kinematics = RobotKinematics()
    self.kinematics.direct_kinematics()
    self.kinematics.trajectory_generator(q_in=[0.59, 2.6, 0.25])
    self.kinematics.inverse_kinematics()
  
  def redirect_print(self, new_print):
    global print
    print = new_print

  def inertia_tensor(self, lx, ly, lz, mass):
    return Matrix([[(mass/12.0)*(ly**2 + lz**2), 0, 0], 
                  [0, (mass/12.0)*(lx**2 + lz**2), 0], 
                  [0, 0, (mass/12.0)*(lx**2 + ly**2)]])
  
def main():
  robot = RobotDynamics()
  robot.redefine_kinematics()
  robot.kinematics.ws_graph()
  robot.kinematics.q_graph()
  robot.define_dynamics()
  robot.lagrange_effort_generator()
  robot.effort_graph()

if __name__ == "__main__":
  main()