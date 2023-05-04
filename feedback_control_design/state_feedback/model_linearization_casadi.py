import casadi as cs

# Setup nonlinear ode
# -------------------
# States
px1 = cs.MX.sym('px1')
py1 = cs.MX.sym('py1')
th1 = cs.MX.sym('th1')
th0 = cs.MX.sym('th0')
x = cs.vertcat(px1, py1, th1, th0)

# Controls
v0 = cs.MX.sym('v0')
om0 = cs.MX.sym('om0')
u = cs.vertcat(v0, om0)

# Parameters
M0 = cs.MX.sym('M0')
L0 = cs.MX.sym('L0')
L1 = cs.MX.sym('L1')
p = cs.vertcat(M0, L0, L1)

# Expressions
b01 = th0 - th1
om1 = v0/L1*cs.sin(b01) - M0/L1*cs.cos(b01)*om0
v1 = v0*cs.cos(b01) + M0*cs.sin(b01)*om0

# ode
rhs = cs.vertcat(v1*cs.cos(th1),
                 v1*cs.sin(th1),
                 v0/L1*cs.sin(b01) - M0/L1*cs.cos(b01)*om0,
                 om0)

f = cs.Function('f', [x, u, p], [rhs])

# Linearize ode
# -------------
dfdx = cs.Function('dfdx', [x, u, p], [cs.jacobian(f(x, u, p), x)])
dfdu = cs.Function('dfdu', [x, u, p], [cs.jacobian(f(x, u, p), u)])
print(dfdx)
print(dfdu)

# Evaluate jacobians
# ------------------
xstar = cs.vertcat(0, 0, 0.1, 0.1)
ustar = cs.vertcat(-0.1, 0)
p = cs.vertcat(0.1, 0.3375, 0.3)

print('Evaluated in xstar = ' + str(xstar) + ' and ustar = ' + str(ustar))
print(dfdx(xstar, ustar, p))
print(dfdu(xstar, ustar, p))
