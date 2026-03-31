from casadi import *
import numpy as np

# 1. Setup Parameters
N = 10          # Horizon length (past steps)
M = 10          # Prediction length (future steps)
dt = 0.05       # 20Hz
L = 0.33        # F1TENTH wheelbase

# 2. Define Symbolic Variables
x = MX.sym('x')
y = MX.sym('y')
theta = MX.sym('theta')
v = MX.sym('v')
states = vertcat(x, y, theta, v)
n_states = states.size1()

delta = MX.sym('delta')
a = MX.sym('a')
controls = vertcat(delta, a)
n_controls = controls.size1()

# 3. ODE and Discretization (RK4)
rhs = vertcat(v * cos(theta), v * sin(theta), (v/L) * tan(delta), a)
f = Function('f', [states, controls], [rhs])

X_next = MX.sym('X_next', n_states)
k1 = f(states, controls)
k2 = f(states + dt/2 * k1, controls)
k3 = f(states + dt/2 * k2, controls)
k4 = f(states + dt * k3, controls)
x_prop = states + dt/6 * (k1 + 2*k2 + 2*k3 + k4)
f_discrete = Function('f_discrete', [states, controls], [x_prop])

# 4. Formulate MHE Optimization
X_vars = MX.sym('X_vars', n_states, N+1)
U_vars = MX.sym('U_vars', n_controls, N)
Y_meas = MX.sym('Y_meas', 2, N+1) # We only measure [X, Y]
X_prior = MX.sym('X_prior', n_states)

obj = 0
g = []

# Weighting Matrices (Tuning)
Q_noise = diag(vertcat(0.1, 0.1)) # Process noise penalty
R_meas = diag(vertcat(10, 10))   # Measurement trust
P_arrival = diag(vertcat(1, 1, 1, 1)) # Arrival cost trust

# Arrival Cost
obj += (X_vars[:,0] - X_prior).T @ P_arrival @ (X_vars[:,0] - X_prior)

for k in range(N):
    # Measurement Cost (X and Y only)
    err = X_vars[0:2, k] - Y_meas[:, k]
    obj += err.T @ R_meas @ err
    
    # Smoothness Penalty (Prevents jittery steering/accel)
    obj += U_vars[:, k].T @ Q_noise @ U_vars[:, k]
    
    # Dynamics Constraint
    g.append(X_vars[:, k+1] - f_discrete(X_vars[:, k], U_vars[:, k]))

# 5. Create Solver
opt_vars = vertcat(reshape(X_vars, -1, 1), reshape(U_vars, -1, 1))
opt_params = vertcat(X_prior, reshape(Y_meas, -1, 1))
nlp = {'x': opt_vars, 'f': obj, 'g': vertcat(*g), 'p': opt_params}

opts = {'ipopt.print_level': 0, 'print_time': 0}
solver = nlpsol('S', 'ipopt', nlp, opts)

# --- 6. Execution Loop (Simplified Simulation) ---
current_prior = np.array([0, 0, 0, 1.0]) # Initial guess [X, Y, theta, v]
meas_history = []

print("Starting MHE Tracking...")

# Bounds for U (Physical limits of F1TENTH)
lbu = [-0.41, -5.0] * N
ubu = [ 0.41,  5.0] * N
lbx = [-inf] * (n_states * (N+1)) + lbu
ubx = [ inf] * (n_states * (N+1)) + ubu

for t in range(30):
    # Simulate a "True" circular path with some noise
    true_x = t * dt * 1.5 * np.cos(t * dt)
    true_y = t * dt * 1.5 * np.sin(t * dt)
    meas_history.append([true_x + np.random.normal(0, 0.05), 
                         true_y + np.random.normal(0, 0.05)])
    
    if len(meas_history) > N:
        y_window = np.array(meas_history[-N-1:]).T
        params = vertcat(current_prior, y_window.flatten())
        
        # Solve MHE
        sol = solver(x0=0, p=params, lbx=lbx, ubx=ubx, lbg=0, ubg=0)
        res = sol['x'].full()
        
        # Extract Current State and Inputs
        est_states = res[:n_states*(N+1)].reshape((n_states, N+1), order='F')
        est_inputs = res[n_states*(N+1):].reshape((n_controls, N), order='F')
        
        x_now = est_states[:, -1]
        u_now = est_inputs[:, -1]
        current_prior = x_now # For next iteration