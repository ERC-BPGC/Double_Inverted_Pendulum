import numpy as np
import sys
from casadi import *
import do_mpc
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Setup the system model (same as the initial code provided)

model_type = 'continuous'
model = do_mpc.model.Model(model_type)

# System parameters
m0 = 0.6  # kg, mass of the cart
m1 = 0.2  # kg, mass of the first rod
m2 = 0.2  # kg, mass of the second rod
L1 = 0.5  # m, length of the first rod
L2 = 0.5  # m, length of the second rod
g = 9.80665  # m/s^2, Gravity
l1 = L1 / 2  # m
l2 = L2 / 2  # m
J1 = (m1 * l1**2) / 3   # Inertia
J2 = (m2 * l2**2) / 3   # Inertia

# Define system equations (same as the initial code provided)

h1 = m0 + m1 + m2
h2 = m1 * l1 + m2 * L1
h3 = m2 * l2
h4 = m1 * l1**2 + m2 * L1**2 + J1
h5 = m2 * l2 * L1
h6 = m2 * l2**2 + J2
h7 = (m1 * l1 + m2 * L1) * g
h8 = m2 * l2 * g

# Define the state variables and input
pos = model.set_variable('_x', 'pos')  # position of the cart
theta = model.set_variable('_x', 'theta', (2, 1))  # theta[0] = rod1, theta[1] = rod2
dpos = model.set_variable('_x', 'dpos')  # velocity of cart
dtheta = model.set_variable('_x', 'dtheta', (2, 1))  # angular velocity of rod0, rod1

u = model.set_variable('_u', 'force')  # input force
ddpos = model.set_variable('_z', 'ddpos')  # acceleration of cart
ddtheta = model.set_variable('_z', 'ddtheta', (2, 1))  # angular acceleration

# Define equations of motion (same as the initial code provided)

model.set_rhs('pos', dpos)
model.set_rhs('theta', dtheta)
model.set_rhs('dpos', ddpos)
model.set_rhs('dtheta', ddtheta)

# Euler-Lagrange equations for the system
euler_lagrange = vertcat(
    # Equation 1
    h1 * ddpos + h2 * ddtheta[0] * cos(theta[0]) + h3 * ddtheta[1] * cos(theta[1])
    - (h2 * dtheta[0]**2 * sin(theta[0]) + h3 * dtheta[1]**2 * sin(theta[1]) + u),

    # Equation 2
    h2 * cos(theta[0]) * ddpos + h4 * ddtheta[0] + h5 * cos(theta[0] - theta[1]) * ddtheta[1]
    - (h7 * sin(theta[0]) - h5 * dtheta[1]**2 * sin(theta[0] - theta[1])),

    # Equation 3
    h3 * cos(theta[1]) * ddpos + h5 * cos(theta[0] - theta[1]) * ddtheta[0] + h6 * ddtheta[1]
    - (h5 * dtheta[0]**2 * sin(theta[0] - theta[1]) + h8 * sin(theta[1]))
)

# Add the Euler-Lagrange equations to the model
model.set_alg('euler_lagrange', euler_lagrange)

# Define kinetic and potential energy (same as the initial code provided)
E_kin_cart = 1 / 2 * m0 * dpos**2
E_kin_p1 = 1 / 2 * m1 * ((dpos + l1 * dtheta[0] * cos(theta[0]))**2 + (l1 * dtheta[0] * sin(theta[0]))**2) + 1 / 2 * J1 * dtheta[0]**2
E_kin_p2 = 1 / 2 * m2 * ((dpos + L1 * dtheta[0] * cos(theta[0]) + l2 * dtheta[1] * cos(theta[1]))**2 + (L1 * dtheta[0] * sin(theta[0]) + l2 * dtheta[1] * sin(theta[1]))**2) + 1 / 2 * J2 * dtheta[0]**2

E_kin = E_kin_cart + E_kin_p1 + E_kin_p2
E_pot = m1 * g * l1 * cos(theta[0]) + m2 * g * (L1 * cos(theta[0]) + l2 * cos(theta[1]))

# Set expressions for energy
model.set_expression('E_kin', E_kin)
model.set_expression('E_pot', E_pot)

# Build the model
model.setup()

# Set up the MPC controller (same as before)
mpc = do_mpc.controller.MPC(model)
setup_mpc = {
    'n_horizon': 100,  # Prediction horizon
    'n_robust': 1,  # Robust optimization
    'open_loop': 0,  # Closed-loop optimization
    't_step': 0.04,  # Time-step
    'state_discretization': 'collocation',
    'collocation_type': 'radau',
    'collocation_deg': 3,
    'collocation_ni': 1,
    'store_full_solution': True,
    'nlpsol_opts': {'ipopt.linear_solver': 'mumps'}
}
mpc.set_param(**setup_mpc)
mterm = model.aux['E_kin'] - model.aux['E_pot']
lterm = model.aux['E_kin'] - model.aux['E_pot']
mpc.set_objective(mterm=mterm, lterm=lterm)
mpc.set_rterm(force=0.1)
mpc.bounds['lower', '_u', 'force'] = -4
mpc.bounds['upper', '_u', 'force'] = 4
mpc.setup()

# State feedback estimator
estimator = do_mpc.estimator.StateFeedback(model)

# Simulator setup
simulator = do_mpc.simulator.Simulator(model)
params_simulator = {
    'integration_tool': 'idas',
    'abstol': 1e-10,
    'reltol': 1e-10,
    't_step': 0.04
}
simulator.set_param(**params_simulator)
simulator.setup()

# Initial conditions
simulator.x0['theta'] = 0.99 * np.pi
x0 = simulator.x0.cat.full()
mpc.x0 = x0
estimator.x0 = x0
mpc.set_initial_guess()

# Set up real-time plotting
plt.ion()
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-1.8, 1.8)
ax.set_ylim(-1.2, 1.2)
ax.set_axis_off()

# Initialize pendulum bars
bar1, = ax.plot([], [], '-o', linewidth=5, markersize=10)
bar2, = ax.plot([], [], '-o', linewidth=5, markersize=10)

# Key press event handler
def on_key_press(event):
    global u0
    if event.key == 'left':
        u0 = -1  # Apply force to the left
    elif event.key == 'right':
        u0 = 1  # Apply force to the right
    elif event.key == 'q':
        plt.close()  # Close the plot and stop the simulation

fig.canvas.mpl_connect('key_press_event', on_key_press)

# Pendulum bar update function
def pendulum_bars(x):
    x = x.flatten()
    line_1_x = np.array([x[0], x[0] + L1 * np.sin(x[1])])
    line_1_y = np.array([0, L1 * np.cos(x[1])])

    line_2_x = np.array([line_1_x[1], line_1_x[1] + L2 * np.sin(x[2])])
    line_2_y = np.array([line_1_y[1], line_1_y[1] + L2 * np.cos(x[2])])

    return line_1_x, line_1_y, line_2_x, line_2_y

# Simulation loop
while True:
    u0 = mpc.make_step(x0)
    y_next = simulator.make_step(u0)
    x0 = estimator.make_step(y_next)

    # Update the pendulum plot
    line1_x, line1_y, line2_x, line2_y = pendulum_bars(x0)
    bar1.set_data(line1_x, line1_y)
    bar2.set_data(line2_x, line2_y)

    # Redraw the plot
    plt.draw()
    plt.pause(0.04)  # Adjust based on your time step

    # Check for the quit condition
    if plt.fignum_exists(fig.number) == False:
        break
