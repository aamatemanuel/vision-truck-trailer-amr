#!/usr/bin/env python3


def dlqr(G, H, Q, R, returnPE=False):
    '''
    From https://github.com/python-control/python-control/issues/359:

    Discrete-time Linear Quadratic Regulator calculation.
    State-feedback control  u[k] = -K*x[k]

    How to apply the function:
      K = dlqr_calculate(G,H,Q,R)
      K, P, E = dlqr_calculate(G,H,Q,R, return_solution_eigs=True)

    Inputs:
    G, H, Q, R  -> all numpy arrays  (simple float number not allowed)
    returnPE: define as True to return Ricatti solution and final eigenvalues

    Returns:
    K: state feedback gain
    P: Ricatti equation solution
    E: eigenvalues of (G-HK)  (closed loop z-domain poles)
    '''
    from scipy.linalg import solve_discrete_are, inv, eig
    P = solve_discrete_are(G, H, Q, R)  # Solução Ricatti
    K = inv(H.T@P@H + R)@H.T@P@G    # K = (B^T P B + R)^-1 B^T P A

    if not returnPE:
        return K

    from numpy.linalg import eigvals
    ei
    gs = np.array([eigvals(G-H@K)]).T
    return K, P, eigs
