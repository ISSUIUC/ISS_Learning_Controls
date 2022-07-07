import numpy as np
import sympy as sym
import json
import matplotlib.pyplot as plt
from scipy import linalg
from scipy.interpolate import interp1d
from sympy import Matrix

def lqr(A, B, Q, R):
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = linalg.inv(R) @  B.T @ P
    return K

def ft_to_m(measurement):
    return (measurement / 3.2808) 

# defining components of position 
h, y, z = sym.symbols('h, y, z')
# defining roll, pitch and yaw
psi, theta, phi  = sym.symbols('psi, theta, phi')
# defining the components of the linear velocity 
u, v, w = sym.symbols('u, v, w')
# defining compoenents for angular velocity 
wx, wy, wz = sym.symbols('wx, wy, wz')
# defining inputs
l1, l2 = sym.symbols('l1, l2')
# defining the physical parameters on the rocket 
g, W_f, Ixx, Sref_a, rho, m, D = sym.symbols('g, W_f, Ixx, Sref_a, rho, m, D')
# defining the coefficients 
Cd_a, Cd_f, Cl_a, Cl_f = sym.symbols('Cd_a, Cd_f, Cl_a, Cl_f')

#* EOMS
hdot = u
hddot = -((rho * u**2 * Sref_a * Cd_a) / (2*m)) - ((rho * u**2 * Cd_f * W_f * (l1+l2))/m) - g
omegadot = (rho * u**2 * Cl_f * W_f * (l1*D + l1**2 - l2*D - l2**2)) / (2*Ixx)
f_sym = Matrix.vstack(Matrix([[hdot],[hddot],[omegadot]]))

#* State Space Model
s = [h, hdot, wx]
i = [l1, l2]
p = [Cd_f, Sref_a,Cd_a,rho, W_f, m, g, Cl_f, Ixx, D]
f = sym.lambdify(s + i + p, f_sym)

#* Plug in Constants
# Mass of the rocket
m = 50 #kg
# moment of inertia
Ixx = 0.5*m*0.1524**2
# acceleration of gravity
g = 9.81 #m/s^2
#  lift coefficient of the rocket 
Cl_a = 0.063
# lift coefficient of the flaps 
Cl_f = 2*np.pi*np.sin(45)
# drag coefficient of the rocket 
Cd_a = 0.58
# drag coefficient of the flaps
Cd_f = 2*np.pi*np.sin(45)
# Width of the flaps 
W_f = 0.0254 # meters
# Diameter of the rocket 
D = 0.1524 # meters
# reference area of the rocket 
Sref_a = np.pi*(D/2)**2
# Density of air 
rho = 1.225 # kg/m^3

#* Define and Plug in Equilibrium Values
h_e = ft_to_m(3000)
hdot_e = .1
omega_e = .1
l1_e = 0.5
l2_e = 0.5

s_eq = [h_e,hdot_e,omega_e]
i_eq = [l1_e,l2_e]
p_eq = [Cd_f,Sref_a,Cd_a,rho,W_f,m,g,Cl_f,Ixx,D]

feq = f(*s_eq,*i_eq,*p_eq)

#* Compute A and B
A_sym = f_sym.jacobian(s)
A_num = sym.lambdify(s + i + p,A_sym)
A = A_num(*s_eq, *i_eq, *p_eq)

B_sym = f_sym.jacobian(i)
B_num = sym.lambdify(s + i + p,B_sym)
B = B_num(*s_eq, *i_eq, *p_eq)

print(B)

#* Compute Controllability
n = len(B)

# Initialize the Controllability Matrix (W)
W = B

# Create the Controllability Matrix (W)
for i in range(1,n):
    new_mat = np.linalg.matrix_power(A, i) @ B
    W = np.block([W,new_mat])

# Make sure that the rank of the matrix is equal to the number of states
# if (np.linalg.matrix_rank(W) == n):
#     print("Full rank")
# else:
#     print("Rank deficient") 

#* LQR Design
## Designing a Q matrix 
Q_diag = np.array([100,10,.1]) #h, hdot, omega
Q = np.diag(Q_diag)

## Designing a R matrix
R_diag = np.array([0.01,0.01]) #l1, l2
R = np.diag(R_diag)

K = lqr(A,B,Q,R)

## testing to see if the A - BK has all negative eigenvalues 
F = A - B@K
eig = np.linalg.eigvals(F)

# print(eig.real < 0)

#* Sim
start_alt = ft_to_m(727.65)
start_vel = ft_to_m(491.29)

v_t = start_vel
h_t = start_alt
a_t = -((rho*(v_t**2)*Sref_a*Cd_a) / (2*m)) - g
omega_t = 0

h_vals = []
v_vals = []
a_vals = []
omega_vals = []

l_max = 0.5

time = np.linspace(0,20,1000)
dt = 20/1000

# print(h_t)
# print(v_t)
# print(omega_t)

# for t in time:

#     ##*With Control
#     x = np.array([[h_t],[v_t],[omega_t]],dtype=object)
#     u = -K@x

#     # if (u[0] > l_max):
#     #     u[0] = l_max
#     # if (u[1] > l_max):
#     #     u[1] = l_max

#     # print(u)

#     xdot = A@x - B@u

#     v_t1 = float(xdot[0])
#     a_t1 = float(xdot[1])
#     omegadot_t1 = float(xdot[2])

#     # print(float(v_t1[0]))

#     # a_t1 = -((rho*(v_t**2)*Sref_a*Cd_a) / (2*m)) - ((rho*(v_t**2)*Cd_f*W_f*(l1+l2))/m) - g
#     # v_t1 = v_t + (a_t*dt)
#     h_t1 = h_t + (v_t*dt) + (0.5*a_t*(dt**2)) 
#     omega_t1 = omega_t + (omegadot_t1*dt)

#     a_vals.append(a_t)
#     v_vals.append(v_t)
#     h_vals.append(h_t)
#     omega_vals.append(omega_t)

#     a_t = a_t1
#     h_t = h_t1
#     v_t = v_t1
#     omega_t = omega_t1

# plt.plot(time,h_vals)
# plt.plot(time,v_vals)
# plt.show()
# plt.savefig("t_vs_h.png")


#State Space Equation for Controller
#xdot = Ax + Bu, u = -Kx
#xdot = (A - BK)x

from scipy import integrate

tspan = np.linspace(0,25,num=100)
xinit = [727.65,491.29,0.25] #altitude, velcity at burnout and random omega

def state_space_func(t,x,Amat,Bmat,Kmat):
    xdot = (Amat - Bmat@Kmat) @ x
    return xdot

sol = integrate.solve_ivp(state_space_func,[tspan[0], tspan[-1]], xinit, t_eval=tspan, rtol = 1e-5,args=(A,B,K))
# sol

plt.plot(sol.t,sol.y[0]) #alt
plt.plot(sol.t,sol.y[1]) #vertical velocity
plt.show()
plt.plot(sol.t,sol.y[2]) #roll rate
plt.show()

# 3721.2


