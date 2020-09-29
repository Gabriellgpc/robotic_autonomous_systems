# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import matplotlib.pyplot as plt
import numpy as np
import sys
import ctypes
#################################### Functions definition ############################################## 
# retorna Wd, We
d = 0.331 #wheel axis distance
r_w = 0.09751 #wheel radius
def pioneer_robot_model(v, omega):
    v_r = (v+d*omega)
    v_l = (v-d*omega)
    omega_right = v_r/r_w
    omega_left = v_l/r_w
    return omega_right, omega_left

def send_path_4_drawing(path, sleep_time = 0.07):    
    for i in path[:,0:2]:
        point2send = i
        packedData = sim.simxPackFloats(point2send.flatten())
        raw_bytes  = (ctypes.c_ubyte * len(packedData)).from_buffer_copy(packedData) 
        returnCode = sim.simxWriteStringStream(clientID, "path_coord", raw_bytes, sim.simx_opmode_oneshot)
        if returnCode != 0:
            # print('Error: fail to send the path point to the simulator!')
            pass
        time.sleep(sleep_time)
    
# xi, yi, thi => ponto e orientação inicial
# xf, yf, thf => ponto e orientação final
# coef : {a0,a1,a2,a3,b0,b1,b2,b3} (dicionario com os coeficientes do pol. de grau 3)
# return: coef
def pathComputer(xi, yi, thi, xf, yf, thf):
    delta = 0.001
    dx = xf - xi
    dy = yf - yi
    coef = dict()
    
    thi_test = (np.pi/2.0 - delta) < thi < (np.pi/2.0 + delta)
    thf_test = (np.pi/2.0 - delta) < thf < (np.pi/2.0 + delta)
    if (thi_test) and (thf_test):
        print('Caso Especial #1')
        # caso especial 1
        coef['b1'] = dy    #coef. livre
        coef['b2'] = 0     #coef. livre
        coef['a0'] = xi
        coef['a1'] = 0
        coef['a2'] = 3*dx
        coef['a3'] = -2*dx
        coef['b0'] = yi
        coef['b3'] = dy - coef['b1'] - coef['b2']
    elif thi_test:
        print('Caso Especial #2')
        #caso especial 2
        alpha_f = np.tan(thf)
        coef['a3'] = -dx/2.0  #coef. livre
        coef['b3'] = 0        #coef. livre (qualquer valor aqui)
        coef['a0'] = xi
        coef['a1'] = 0
        coef['a2'] = dx - coef['a3']
        coef['b0'] = yi
        coef['b1'] = 2*(dy - alpha_f*dx) - alpha_f*coef['a3'] + coef['b3']
        coef['b2'] = (2*alpha_f*dx - dy) + alpha_f*coef['a3'] - 2*coef['b3']
    elif thf_test:
        print('Caso Especial #3')
        #caso especial 3
        alpha_i = np.tan(thi)
        coef['a1'] = 3*dx/2.0  #coef. livre
        coef['b2'] = 0         #coef. livre (qualquer valor aqui)
        coef['a0'] = xi
        coef['a2'] = 3*dx - 2*coef['a1']
        coef['a3'] = coef['a1'] - 2*dx
        coef['b0'] = yi
        coef['b1'] = alpha_i*coef['a1']
        coef['b3'] = dy - alpha_i*coef['a1'] - coef['b2']
    else:
        print('Caso Geral')
        #caso geral
        alpha_i = np.tan(thi)
        alpha_f = np.tan(thf)
        coef['a1'] = dx       #coef. livre
        coef['a2'] = 0        #coef. livre
        coef['a0'] = xi
        coef['a3'] = dx - coef['a1'] - coef['a2']
        coef['b0'] = yi
        coef['b1'] = alpha_i*coef['a1']
        coef['b2'] = 3*(dy - alpha_f*dx) + 2*(alpha_f - alpha_i)*coef['a1'] + alpha_f*coef['a2']
        coef['b3'] = 3*alpha_f*dx - 2*dy - (2*alpha_f - alpha_i)*coef['a1'] - alpha_f*coef['a2']
    return coef
def pathGenerator(coef, l):
    x = coef['a0'] + coef['a1']*l + coef['a2']*l**2 + coef['a3']*l**3
    y = coef['b0'] + coef['b1']*l + coef['b2']*l**2 + coef['b3']*l**3
    th= np.arctan2(coef['b1'] + 2*coef['b2']*l + 3*coef['b3']*l**2, 
                   coef['a1'] + 2*coef['a2']*l + 3*coef['a3']*l**2)
    return np.array(list(zip(x,y,th)),dtype=float)
##################################### Connecting to simulator ##################################################
clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)
if (clientID != 0):
    print('Falha na conexão')
    exit()
print('Conectado!')

# handlers of motors and the robot
returnR, robot_handle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_oneshot_wait)
returnT, target_handle = sim.simxGetObjectHandle(clientID,'Target',sim.simx_opmode_oneshot_wait)
if returnR != 0:
    print('ERRO: Falha em GetObjectHandle para o Pioneer')
    exit()
if returnT != 0:
    print('ERRO: Falha em GetObjectHandle para o Target')
    exit()    
######################################## variables ###############################################
time.sleep(0.5)
resA,resP = 1, 1
resT = 1
while (resA != 0) and (resP != 0) and (resT != 0):
    resA,ang = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)    
    resP,pos = sim.simxGetObjectPosition(clientID,robot_handle,-1, sim.simx_opmode_streaming)
    resT,posT= sim.simxGetObjectPosition(clientID,target_handle,-1, sim.simx_opmode_streaming)

pi = [pos[0], pos[1], ang[2]]
pf = [posT[0], posT[1],0.0]
coef = pathComputer(pi[0], pi[1], pi[2], pf[0], pf[1], pf[2])
print('Coeficientes:')
print(coef)

l = np.linspace(0,1,100)
path = pathGenerator(coef, l)

# print(path.shape)
plt.plot(path[0,0], path[0,1], 'or', label='Start')
plt.plot(path[-1,0], path[-1,1], 'ob', label='End')
plt.plot(path[:,0], path[:,1], '-k', label='Path')

send_path_4_drawing(path)

plt.title('Caminho gerado')
plt.ylabel('y[m]')
plt.xlabel('x[m]')
plt.grid(True)
plt.legend()
plt.show()
####################################### graph configuration #######################################

######################################## simulation  #######################################

# Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
sim.simxGetPingTime(clientID)
# Now close the connection to CoppeliaSim:
sim.simxFinish(clientID)