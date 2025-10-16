import numpy as np
from xmera.fswAlgorithms import rwMotorTorque
from numpy import linalg as la


def control_axes_3D():
    C = np.array([
        [1.0, 0.0, 0.0]
        , [0.0, 1.0, 0.0]
        , [0.0, 0.0, 1.0]
    ])
    return C
def control_axes_2D():
    C = np.array([
        [1.0, 0.0, 0.0]
        , [0.0, 0.0, 1.0]
    ])
    return C
def control_axes_1D():
    C = np.array([
        [1.0, 0.0, 0.0]
    ])
    return C



def compute_torque_u(C, Gs_B, Lr, avail_msg):

    num_control_axes = (np.linalg.norm(C, axis=1) > 0.0).sum()
    num_wheels = len(avail_msg)
    non_avail_wheels = 0

    # Remove wheels that are deemed unavailable
    for i in range(len(Gs_B[0])): #
        if num_wheels > i:
            if avail_msg[i] is not rwMotorTorque.AVAILABLE:
                Gs_B[:,i] = [0.0, 0.0, 0.0]
                non_avail_wheels += 1
        else:
            Gs_B[:,i] = [0.0, 0.0, 0.0]

    # If fewer wheels than number of control axes, output no torque
    if (num_wheels-non_avail_wheels) < num_control_axes:
        return [0.0]*len(Gs_B[0])


    Lr_C = np.dot(C,Lr) # Project torque onto control axes
    CGs = np.dot(C, Gs_B) # Map the control axes onto the wheels

    # Build minimum norm framework
    M = np.dot(CGs, CGs.T)
    M_rep = np.identity(3) # Need to keep the matrix non-singular for inversion
    for i in range(0,num_control_axes):
        for j in range(0,num_control_axes):
            M_rep[i][j] = M[i][j]
    M_inv = la.inv(M_rep)

    # Remove projection to any non-defined control axes
    for i in range(num_control_axes,3):
        M_inv[i][i] = 0.0

    # Determine the solution
    v3_temp = np.dot(M_inv, Lr_C)

    # Map the solution to the wheels
    u_s = np.dot(CGs.T, v3_temp)

    return -u_s

def example_computation():
    Gs_B = np.array([
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.5773502691896258, 0.5773502691896258, 0.5773502691896258]
    ]).T

    js_list = np.array([0.1, 0.1, 0.1, 0.1])
    num_rw = 4
    rw_config_params = (Gs_B, js_list, num_rw)

    Lr = np.array([1.0, -0.5, 0.7])
    rw_availability = np.array([1, 1, 1, 1])

    print('3D Control')
    u_s = compute_torque_u(control_axes_3D(), Gs_B, Lr)
    print('U_s = ', u_s, '\n')

    print('2D Control')
    u_s = compute_torque_u(control_axes_2D(), Gs_B, Lr)
    print('U_s = ', u_s)

    print('1D Control')
    u_s = compute_torque_u(control_axes_1D(), Gs_B, Lr)
    print('U_s = ', u_s)
