from cProfile import label
import numpy as np
import matplotlib.pyplot as plt
import math as m
from copy import deepcopy

dt = 0.01
cnt = 1000 # Measure times

P = np.diag([0.0009, 0.0009, 0.0009, 0.0009, 0.0009, 0.0009, 0.0009, 0.0009, 0.0009])
             #xc        #yc   #zc     #yaw    #v_xc #v_yc  #v_zc  #w #r
Q = np.diag([0.0002, 0.0002, 0.0001, 0.0002, 0.0012, 0.0002, 0.0002, 0.0002, 0.0002])
#Q = np.diag([0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01])
R = np.diag([0.0006, 0.0006, 0.0009, 0.0009])

F = np.array([[1,   0,   0,   0,   dt, 0,   0,   0,   0],
              [0,   1,   0,   0,   0,  dt,  0,   0,   0],
              [0,   0,   1,   0,   0,  0,   dt,  0,   0], 
              [0,   0,   0,   1,   0,  0,   0,   dt,  0],
              [0,   0,   0,   0,   1,  0,   0,   0,   0],
              [0,   0,   0,   0,   0,  1,   0,   0,   0],
              [0,   0,   0,   0,   0,  0,   1,   0,   0],
              [0,   0,   0,   0,   0,  0,   0,   1,   0],
              [0,   0,   0,   0,   0,  0,   0,   0,   1]])

def h(x):
    z = np.zeros(4)
    xc = x[0]
    zc = x[2]
    yaw = x[3]
    r=x[8] 
    z[0] = xc + r * m.cos(yaw)
    z[1] = x[1]
    z[2] = zc + r * m.sin(yaw)
    z[3] = x[3]
    return z
    

def Jh(x):
    yaw = x[3]
    r = x[8] 
    return np.array([[1,   0,   0,  -r*m.sin(yaw),  0,   0,   0,   0,   m.cos(yaw)],
                     [0,   1,   0,  0,              0,   0,   0,   0,   0],
                     [0,   0,   1,   r*m.cos(yaw),  0,   0,   0,   0,   m.sin(yaw)],
                     [0,   0,   0,   1,             0,   0,   0,   0,   0]])

def EKF(x, Z):
    # predict
    global P
    x = F @ x
    P = F @ P @ F.T + Q

    # mesure
    H = Jh(x)
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x = x +  K @ (Z - h(x))
    P = (np.identity(9) - K @ H) @ P
    return x

def static(x):
    return x

v = np.array([3.0, 0.0, 0.0])
def tranlateMove(x):
    x[:3] += v * dt
    x[4:7] = v
    return x

w = 2.0
def spinMove(x):
    x[3] += w * dt
    x[7] = w
    return x

def tranlateSpinMove(x):
    x = tranlateMove(x)
    x = spinMove(x)
    return x

def generateMeasureData(x_init, move_fun):
    Z_list = []
    real_x_list = []
    x_sigma = 0.02
    y_sigma = 0.02
    z_sigma = 0.03
    yaw_sigma = 0.03
    x_noise = np.random.normal(0, x_sigma, cnt)
    y_noise = np.random.normal(0, y_sigma, cnt)
    z_noise = np.random.normal(0, z_sigma, cnt)
    yaw_noise = np.random.normal(0, yaw_sigma, cnt) 
    noise = np.stack([x_noise, y_noise, z_noise, yaw_noise])
    x = x_init
    for i in range(cnt):
        real_x = move_fun(x)
        real_x_list.append(deepcopy(real_x))
        Z_no_noise = h(real_x)
        Z = Z_no_noise + noise[:, i]
        Z_list.append(deepcopy(Z))
        x = real_x
    return [real_x_list, Z_list]
    
def execEKF(Z_list):
    ekf_list = []
    P_list = []
    yaw = Z_list[0][3]
    r = 0.2
    x = Z_list[0][0] - r * m.cos(yaw)
    y = Z_list[0][1]
    z = Z_list[0][2] - r * m.sin(yaw)

    x_prio = np.array([x, y, z, yaw, 0.0, 0.0, 0.0, 0.0, r])
    p_arr = np.zeros(9)
    for i in range(9):
        p_arr[i] = P[i][i]
    P_list.append(deepcopy(p_arr))
    ekf_list.append(x_prio)
    for i in range(1, cnt):
        x_post = EKF(x_prio, Z_list[i])
        ekf_list.append(deepcopy(x_post))
        x_prio = x_post
        
        p_arr = np.zeros(9)
        for i in range(9):
            p_arr[i] = P[i][i]
        P_list.append(p_arr)

    return [ekf_list, P_list]

x_init = np.array([0.1, -0.1, -3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2])
#real_x_list, Z_list = generateMeasureData(x_init, static)
#real_x_list, Z_list = generateMeasureData(x_init, tranlateMove)
real_x_list, Z_list = generateMeasureData(x_init, spinMove)
ekf_list, P_list = execEKF(Z_list)

t = np.arange(0, cnt * dt, dt)
real_x_arr = np.array(real_x_list)
Z_arr = np.array(Z_list)
ekf_arr = np.array(ekf_list)
P_arr = np.array(P_list)
plt.subplot(421)
plt.plot(t, real_x_arr[:, 0], label = 'real_xa')
plt.plot(t, ekf_arr[:, 0], label = 'ekf_xa')
plt.title('xa')
plt.subplot(422)
plt.plot(t, real_x_arr[:, 4], label = 'real_vx')
plt.plot(t, ekf_arr[:, 4], label = 'ekf_va')
plt.title('vx')
plt.subplot(423)
plt.plot(t, real_x_arr[:, 2], label = 'real_za')
plt.plot(t, ekf_arr[:, 2], label = 'ekf_za')
plt.title('za')
plt.subplot(424)
plt.plot(t, real_x_arr[:, 6], label = 'real_vz')
plt.plot(t, ekf_arr[:, 6], label = 'ekf_vz')
plt.title('vz')
plt.subplot(425)
plt.plot(t, real_x_arr[:, 3], label = 'real_yaw')
plt.plot(t, ekf_arr[:, 3], label = 'ekf_yaw')
plt.title('yaw')
plt.subplot(426)
plt.plot(t, real_x_arr[:, 7], label = 'real_w')
plt.plot(t, ekf_arr[:, 7], label = 'ekf_w')
plt.title('w')
plt.subplot(427)
plt.plot(t, real_x_arr[:, 8], label = 'real_r')
plt.plot(t, ekf_arr[:, 8], label = 'ekf_r')
plt.title('r')
plt.subplot(428)
for i in range(9):
    plt.plot(t, P_arr[:, i], label = 'sigma_' + str(i))
plt.title('P')
plt.show()


