import casadi as ca
import numpy as np
import matplotlib.pyplot as plt

# Problem set up
## Pendulum parameters
m = 1 # kg
l = 1 # m
g = 9.81 # m/s^2
b = 0.1 
dt = 0.05 # time step, s
N = 20 # prediction horizon, steps
T = 10 # total simulation time
steps = int(T / dt) # int to prevent float error in loop

theta = ca.MX.sym('theta')
theta_dot = ca.MX.sym('theta_dot')
x = ca.vertcat(theta, theta_dot) # combining states into a column vector
u = ca.MX.sym('u')

## External force definition
force_positions = [1, 0.5] # locations where force is applied; distance (m) from pivot (here used as tip and halfway of mass). More locations can be applied.
num_forces = len(force_positions)
F_ext = ca.MX.sym('F_ext', num_forces) # external forces 

torques = [] # Net external torque, flexible as a loop so more positions can be added
for i, distance in enumerate(force_positions):
    torques.append(distance * F_ext[i])
tau_ext_total = sum(torques)

## Dynamics
theta_ddot = (u + tau_ext_total - b * theta_dot - m * g * l * ca.sin(theta)) / (m * l**2)
x_dots = ca.vertcat(theta_dot, theta_ddot)
dynamics_func = ca.Function('dynamics_func', [x, u, F_ext], [x_dots])

## Integration
def rk4_step(x_val, u_val, F_val):
    k1 = dynamics_func(x_val, u_val, F_val)
    k2 = dynamics_func(x_val + dt/2 * k1, u_val, F_val)
    k3 = dynamics_func(x_val + dt/2 * k2, u_val, F_val)
    k4 = dynamics_func(x_val + dt * k3, u_val, F_val)
    return x_val + dt/6 * (k1 + 2*k2 + 2*k3 + k4)

f_discrete = ca.Function('f_discrete', [x, u, F_ext], [rk4_step(x, u, F_ext)])


# MPC setup
Q = np.diag([10, 5])
R = np.array([[0.1]])
x_ref = np.array([[np.pi], [0]]) 

opti = ca.Opti()
X = opti.variable(2, N+1)
U = opti.variable(1, N)
F_ext_seq = opti.parameter(num_forces, N)
X0 = opti.parameter(2, 1)

## Cost function
obj = 0
for k in range(N):
    x_k = X[:, k]
    u_k = U[:, k]
    x_err = x_k - x_ref.flatten()
    obj += ca.mtimes([x_err.T, Q, x_err]) + ca.mtimes([u_k.T, R, u_k])
opti.minimize(obj)

## Constraints
opti.subject_to(X[:, 0] == X0)
for k in range(N):
    f_k = F_ext_seq[:, k]
    opti.subject_to(X[:, k+1] == f_discrete(X[:, k], U[:, k], f_k))
opti.subject_to(opti.bounded(-5.0, U, 5.0))
opti.solver('ipopt', {'ipopt.print_level': 0, 'print_time': 0})

## Simulation
x_mpc = np.array([np.pi, 0.0])
x_uncontrolled = x_mpc.copy()
X_log_mpc = [x_mpc.copy()]
X_log_uncontrolled = [x_uncontrolled.copy()]
U_log = []

for t in range(steps):
    # Random forces applied at different points
    f_ext_t = 10.0 * (np.random.rand(num_forces) - 0.5)
    f_ext_seq = np.tile(f_ext_t.reshape(-1, 1), (1, N))  # repeat same force across horizon

    opti.set_value(X0, x_mpc)
    opti.set_value(F_ext_seq, f_ext_seq)
    sol = opti.solve()
    u_mpc = sol.value(U[:, 0])

    # Applying first force
    x_mpc = f_discrete(x_mpc, u_mpc, f_ext_t).full().flatten()
    x_uncontrolled = f_discrete(x_uncontrolled, 0.0, f_ext_t).full().flatten()

    X_log_mpc.append(x_mpc.copy())
    X_log_uncontrolled.append(x_uncontrolled.copy())
    U_log.append(u_mpc)

## Converting
X_log_mpc = np.array(X_log_mpc)
X_log_uncontrolled = np.array(X_log_uncontrolled)
U_log = np.array(U_log)


# Plotting
plt.figure()
plt.plot(np.arange(steps+1)*dt, X_log_mpc[:, 0], label='theta (controlled)')
plt.plot(np.arange(steps+1)*dt, X_log_uncontrolled[:, 0], label='theta (uncontrolled)', linestyle='--')
plt.axhline(y=np.pi, color='gray', linestyle=':', label='target θ=π')
plt.xlabel('Time (s)')
plt.ylabel('Angle (rad)')
plt.title('Pendulum Angle: Multiple External Forces')
plt.legend()
plt.grid()

plt.figure()
plt.plot(np.arange(steps+1)*dt, X_log_mpc[:, 1], label='theta_dot (controlled)')
plt.plot(np.arange(steps+1)*dt, X_log_uncontrolled[:, 1], label='theta_dot (uncontrolled)', linestyle='--')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Angular Velocity: Multiple External Forces')
plt.legend()
plt.grid()

plt.figure()
plt.plot(np.arange(steps)*dt, U_log)
plt.xlabel('Time (s)')
plt.ylabel('Control Torque (u)')
plt.title('Control Input from NMPC')
plt.grid()

plt.show()
