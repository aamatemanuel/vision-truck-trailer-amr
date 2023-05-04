import sympy as sp

# Setup nonlinear ode
# -------------------
# States
px1 = sp.Symbol('px1')
py1 = sp.Symbol('py1')
th1 = sp.Symbol('th1')
th0 = sp.Symbol('th0')
x = sp.Matrix([px1, py1, th1, th0])

# Controls
v0 = sp.Symbol('v0')
om0 = sp.Symbol('om0')
u = sp.Matrix([v0, om0])

# Parameters
M0 = sp.Symbol('M0')
L0 = sp.Symbol('L0')
L1 = sp.Symbol('L1')
p = sp.Matrix([M0, L0, L1])

# Expressions
b01 = th0 - th1
om1 = v0/L1*sp.sin(b01) - M0/L1*sp.cos(b01)*om0
v1 = v0*sp.cos(b01) + M0*sp.sin(b01)*om0

# ode
f = sp.Matrix([[v1*sp.cos(th1)],
               [v1*sp.sin(th1)],
               [v0/L1*sp.sin(b01) - M0/L1*sp.cos(b01)*om0],
               [om0]])

# Linearize ode
# -------------
dfdx = f.jacobian(x)
dfdu = f.jacobian(u)
print(dfdx)
print(dfdu)

# Result:
# Matrix([[0, 0, -(M0*om0*sin(th0 - th1) + v0*cos(th0 - th1))*sin(th1) + (-M0*om0*cos(th0 - th1) + v0*sin(th0 - th1))*cos(th1),
#          (M0*om0*cos(th0 - th1) - v0*sin(th0 - th1))*cos(th1)],
#         [0, 0, (M0*om0*sin(th0 - th1) + v0*cos(th0 - th1))*cos(th1) + (-M0*om0*cos(th0 - th1) + v0*sin(th0 - th1))*sin(th1),
#          (M0*om0*cos(th0 - th1) - v0*sin(th0 - th1))*sin(th1)],
#         [0, 0, -M0*om0*sin(th0 - th1)/L1 - v0*cos(th0 - th1)/L1, M0*om0*sin(th0 - th1)/L1 + v0*cos(th0 - th1)/L1],
#         [0, 0, 0, 0]])
# Matrix([[cos(th1)*cos(th0 - th1),
#          M0*sin(th0 - th1)*cos(th1)],
#         [sin(th1)*cos(th0 - th1),
#          M0*sin(th1)*sin(th0 - th1)],
#         [sin(th0 - th1)/L1,
#          -M0*cos(th0 - th1)/L1],
#         [0, 1]])


# Evaluate jacobians
# ------------------
xstar = sp.Matrix([0, 0, 0, 0])
ustar = sp.Matrix([0, 0])

dfdx_star = dfdx.subs([(px1, 0),
                       (py1, 0),
                       (th1, 0),
                       (th0, 0),
                       (v0, 0),
                       (om0, 0)])
dfdu_star = dfdu.subs([(px1, 0),
                       (py1, 0),
                       (th1, 0),
                       (th0, 0),
                       (v0, 0),
                       (om0, 0)])

print('Evaluated in xstar = ' + str(xstar) + ' and ustar = ' + str(ustar))
print(dfdx_star)
print(dfdu_star)
