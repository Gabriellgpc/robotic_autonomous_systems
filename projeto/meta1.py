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

#################################### Functions definition ############################################## 
# calcula as velocidades que devem ser aplicadas para realizar um circulo de radio R no tempo t
# retorna Wd, We
def omegas(b, r, R, delta_t):
    w = ((2.0*np.pi*R)/delta_t) * r
    we = ((R - b/2.0)/r) * w
    wd = we * (R+b/2.0)/(R - b/2.0)

    print('Omega = {} | Omega_D = {} | Omega_E = {}'.format(w, wd,we))
    return wd,we
##################################### Connecting to simulator ##################################################

clientID = sim.simxStart('127.0.0.1',19999,True,True,5000,5)
if (clientID != 0):
    print('Falha na conexão')
    exit()
print('Conectado!')

# handlers of motors and the robot
returnLM, LwMotor_handle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_oneshot_wait)
returnRM, RwMotor_handle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_oneshot_wait)
returnR, robot_handle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_oneshot_wait)

if returnR != 0:
    print('ERRO: Falha em GetObjectHandle para o Pioneer')
    exit()

###################################################################################################

t_vec  = list()
we_vec = list()
wd_vec = list()
x_vec  = list()
y_vec  = list()
th_vec = list()

####################################### graph configuration #######################################
fig1 = plt.figure(figsize=(4, 4))
ax_xy = fig1.add_subplot(1, 1, 1, aspect=1)
ax_xy.set_xlim([-2.5,2.5])
ax_xy.set_ylim([-0.5,4.5])
ax_xy.set_xlabel('x[m]')
ax_xy.set_ylabel('y[m]')
ax_xy.grid(False)
ax_xy.plot([],[], '*k')

fig2, (ax_th, ax_w) = plt.subplots(2, 1, constrained_layout=True, sharex=True)

ax_th.set_ylabel(r'$\theta$ [rad]')
ax_th.plot([], [], '-r', label=r'$\theta$')
ax_th.set_ylim([-np.pi, np.pi])
ax_th.legend()
ax_th.grid(True)

ax_w.set_ylabel(r'$\omega$ [rad/s]')
ax_w.set_xlabel('t[s]')
ax_w.grid(True)
ax_w.plot([], [], '-r', label=r'$\omega_d$')
ax_w.plot([], [], '-g', label=r'$\omega_e$')
ax_w.legend()

plt.ion()
######################################## simulation  #######################################
b = 0.331 #wheel axis distance [m]
r = 0.1 #wheel radius [m]
R = 2.0 #[m]
deltaT = 20.0 #[s]
wd,we = omegas(b,r,R, deltaT)

stopTime = 20.0 #[s]
begin = time.time() #[s]
t = 0
while t <= stopTime:
    t = time.time() - begin
    # getting absolute position and orientation(euler) information
    resA,ang = sim.simxGetObjectOrientation(clientID, robot_handle, -1, sim.simx_opmode_streaming)    
    resP,pos = sim.simxGetObjectPosition(clientID,robot_handle,-1, sim.simx_opmode_streaming)
    
    if (resA!=0) or (resP!=0):
        # print('Erro na obtenção da posição ou da orientação')
        continue
    # saving datas
    t_vec.append(t)
    we_vec.append(we)
    wd_vec.append(wd)
    x_vec.append(pos[0])
    y_vec.append(pos[1])
    th_vec.append(ang[2])

    #live plotting
    ax_xy.plot(x_vec,y_vec,'*k')
    ax_th.plot(t_vec, th_vec, '-b')
    ax_w.plot(t_vec, wd_vec, '-r', t_vec, we_vec, '-g')
    plt.pause(0.01)
    
    # sending velocities (rad/s)
    sim.simxSetJointTargetVelocity(clientID,RwMotor_handle,wd,sim.simx_opmode_streaming)
    sim.simxSetJointTargetVelocity(clientID,LwMotor_handle,we,sim.simx_opmode_streaming)

    time.sleep(0.1)
end = time.time()
print('Tempo Decorrido:', end - begin)

sim.simxSetJointTargetVelocity(clientID,RwMotor_handle,0.0,sim.simx_opmode_streaming)
sim.simxSetJointTargetVelocity(clientID,LwMotor_handle,0.0,sim.simx_opmode_streaming)

# Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
sim.simxGetPingTime(clientID)

# Now close the connection to CoppeliaSim:
sim.simxFinish(clientID)

######################################## Static plotting  #######################################
plt.ioff()
ax_xy.plot(x_vec,y_vec,'-ok')
ax_th.plot(t_vec, th_vec, '-b')
ax_w.plot(t_vec, wd_vec, '-r', t_vec, we_vec, '-g')
plt.show()