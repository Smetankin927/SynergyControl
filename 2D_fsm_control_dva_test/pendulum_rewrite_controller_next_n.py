import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os


import matplotlib.pyplot as plt 
import sympy as sym
from sympy import Matrix, init_printing, symbols, solve, eye, Rational, sin, cos, simplify, Eq, evaluate
from sympy import diag, collect, expand, latex
import numpy as np

xml_path = '2D_simple_pendulum.xml' #xml file (assumes this is in the same folder as this file)
simend = 4.0 #simulation time
print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0


from tryafteronecopyoffindparamssim import U, W
from eq_of_moti_and_fem import t_s1_plt1, t_s1_plt2, JA3

#######################################
from collections import deque


#######################################

def init_controller(model,data):
    #initialize the controller here. This function is called once, in the beginning
    pass
    # global FSM
    # FSM = FSM_SWINGUP

def controller(model, data):
    #put the controller here. This function is called inside the simulation.
    pass
    #torque control;
    # set_torque_servo(0, 1)
    #

def set_torque_servo(model, actuator_no, flag):
    if (flag==0):
        model.actuator_gainprm[actuator_no, 0] = 0
    else:
        model.actuator_gainprm[actuator_no, 0] = 1


def keyboard(window, key, scancode, act, mods):
    if act == glfw.PRESS and key == glfw.KEY_BACKSPACE:
        mj.mj_resetData(model, data)
        mj.mj_forward(model, data)

def mouse_button(window, button, act, mods):
    # update button state
    global button_left
    global button_middle
    global button_right

    button_left = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_LEFT) == glfw.PRESS)
    button_middle = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_MIDDLE) == glfw.PRESS)
    button_right = (glfw.get_mouse_button(
        window, glfw.MOUSE_BUTTON_RIGHT) == glfw.PRESS)

    # update mouse position
    glfw.get_cursor_pos(window)

def mouse_move(window, xpos, ypos):
    # compute mouse displacement, save
    global lastx
    global lasty
    global button_left
    global button_middle
    global button_right

    dx = xpos - lastx
    dy = ypos - lasty
    lastx = xpos
    lasty = ypos

    # no buttons down: nothing to do
    if (not button_left) and (not button_middle) and (not button_right):
        return

    # get current window size
    width, height = glfw.get_window_size(window)

    # get shift key state
    PRESS_LEFT_SHIFT = glfw.get_key(
        window, glfw.KEY_LEFT_SHIFT) == glfw.PRESS
    PRESS_RIGHT_SHIFT = glfw.get_key(
        window, glfw.KEY_RIGHT_SHIFT) == glfw.PRESS
    mod_shift = (PRESS_LEFT_SHIFT or PRESS_RIGHT_SHIFT)

    # determine action based on mouse button
    if button_right:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_MOVE_H
        else:
            action = mj.mjtMouse.mjMOUSE_MOVE_V
    elif button_left:
        if mod_shift:
            action = mj.mjtMouse.mjMOUSE_ROTATE_H
        else:
            action = mj.mjtMouse.mjMOUSE_ROTATE_V
    else:
        action = mj.mjtMouse.mjMOUSE_ZOOM

    mj.mjv_moveCamera(model, action, dx/height,
                      dy/height, scene, cam)

def scroll(window, xoffset, yoffset):
    action = mj.mjtMouse.mjMOUSE_ZOOM
    mj.mjv_moveCamera(model, action, 0.0, -0.05 *
                      yoffset, scene, cam)

#get the full path
dirname = os.path.dirname(__file__)
abspath = os.path.join(dirname + "/" + xml_path)
xml_path = abspath

# MuJoCo data structures
model = mj.MjModel.from_xml_path(xml_path)  # MuJoCo model
data = mj.MjData(model)                # MuJoCo data
cam = mj.MjvCamera()                        # Abstract camera
opt = mj.MjvOption()                        # visualization options

# Init GLFW, create window, make OpenGL context current, request v-sync
glfw.init()
window = glfw.create_window(1200, 900, "Demo", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# initialize visualization data structures
mj.mjv_defaultCamera(cam)
mj.mjv_defaultOption(opt)
scene = mj.MjvScene(model, maxgeom=10000)
context = mj.MjrContext(model, mj.mjtFontScale.mjFONTSCALE_150.value)

# install GLFW mouse and keyboard callbacks
glfw.set_key_callback(window, keyboard)
glfw.set_cursor_pos_callback(window, mouse_move)
glfw.set_mouse_button_callback(window, mouse_button)
glfw.set_scroll_callback(window, scroll)

# Example on how to set camera configuration

cam.azimuth = -90.68741727466428 ; cam.elevation = -2.8073894766455036 ; cam.distance =  5.457557373462702
cam.lookat =np.array([ 0.0 , 0.0 , 3.0 ])



print("U, ", U)
print("W, ", W)


set_torque_servo(model, 0, 1)
set_torque_servo(model, 1, 1)

data.qpos[0] = np.pi
data.qpos[1] = 0


lH = 0.0077
SH = 1.042
VH = 0.088

lA = 0.0557
SA = 1.335
VA = 0.305

i = 0
time = 0
dt = 0.001

f = 0.5
A = 0.1
t = symbols("t")

fi_des1 = -0.25*A*sin(2*np.pi*f*t)

fi_des2 = A*sin(2*np.pi*f*t)#-0.349066*sin(2*np.pi*f*t)
fi_des2_2 = 0
fi_des1_1 = 0

taums_1 = 40
taums_2 = 80

Ksi_1 = deque([0] * taums_1)

Ksi_tau1 = deque([0] * taums_1)
dKsi_tau1 =deque([0] * taums_1)
ddKsi_tau1 = deque([0] * taums_1)


Ksi_2 = deque([0] * taums_2)

Ksi_tau2 = deque([0] * taums_2)
dKsi_tau2 = deque([0] * taums_2)
ddKsi_tau2 = deque([0] * taums_2)

ArrayKsi_2 = [0]*taums_2
ArrayKsi_1 = [0]*taums_1

ksides1 = []
ksides2 = []
i=0
while not glfw.window_should_close(window):
    time_prev = data.time

    while (data.time - time_prev < 1.0/60.0):
        
        mj.mj_forward(model,data)
        # we have already fi_des1 and fi_des2 is sympy
        # fi = W @ ksi  ==>  ksi = W.inv() @ fi
        stt = {t:time}
        # print(time)
        # if time < 0.2:
        #     fi_des2_2 = fi_des2.subs(stt)
        #     fi_des1_1 = fi_des1.subs(stt)
        #     Fi_des = Matrix([fi_des1.subs(stt), fi_des2.subs(stt)])  # зависит от t
        # else: 
        #     Fi_des = Matrix([fi_des1_1, fi_des2_2])

        Fi_des = Matrix([fi_des1.subs(stt), fi_des2.subs(stt)])  # зависит от t


            
        Ksi_des = W.inv() @ Fi_des

        ksides1.append(Ksi_des[0])
        ksides2.append(Ksi_des[1])

        # Fi_des = Matrix([fi_des1.subs(stt), fi_des2.subs(stt)])  # зависит от t
        
        # Ksi_des = W.inv() @ Fi_des
        
        # берем значения угловой скорости и ускорения и записываем их в очереди

        joint_name = "joint0"  # Replace with the name of your joint
        joint_id = model.joint(joint_name).id  # Get the joint ID   
        dof_start = model.jnt_dofadr[joint_id]  # Start index of the joint's DOF in qvel/qacc

        # Joint angular acceleration (rad/s²) NOW 
        joint_angular_acceleration_fi1 = data.qacc[dof_start]
        joint_angular_vel_fi1 = data.qvel[dof_start]
        joint_angular_pos_fi1 = data.qpos[dof_start] - np.pi
        # print("current ", data.qpos[dof_start] - np.pi )
        
        #joint 1
        joint_name1 = "joint1"  # Replace with the name of your joint
        joint_id1 = model.joint(joint_name1).id  # Get the joint ID
        dof_start1 = model.jnt_dofadr[joint_id1]  # Start index of the joint's DOF in qvel/qacc

        # Joint angular acceleration (rad/s²) NOW 
        joint_angular_acceleration_fi2 = data.qacc[dof_start1]
        joint_angular_vel_fi2 = data.qvel[dof_start1]
        joint_angular_pos_fi2 = data.qpos[dof_start1]

        # from fi to ksi
        # eta = eta(t-tau) + Seig(ksi_des(t) - ksi(t-tau)) - Veig*dksi(t-tau) - lambda*ddksi(t-tau)

        ksi_1, ksi_2 = W.inv() @ Matrix([joint_angular_pos_fi1, joint_angular_pos_fi2])

        Ksi_tau1.append(ksi_1)
        Ksi_tau2.append(ksi_2)
        ArrayKsi_2.append(ksi_2)
        ArrayKsi_1.append(ksi_1)

        dksi_1, dksi_2 = W.inv() @ Matrix([joint_angular_vel_fi1, joint_angular_vel_fi2])

        dKsi_tau1.append(dksi_1)
        dKsi_tau2.append(dksi_2)
        

        ddksi_1, ddksi_2 = W.inv() @ Matrix([joint_angular_acceleration_fi1, joint_angular_acceleration_fi2])
        
        ddKsi_tau1.append(ddksi_1)
        ddKsi_tau2.append(ddksi_2)

        ###
        # eta = eta(t-tau) + Seig(ksi_des(t) - ksi(t-tau)) - Veig*dksi(t-tau) - lambda*ddksi(t-tau)

        ksi_tau1 = Ksi_tau1.popleft()
        dksi_tau1 = dKsi_tau1.popleft()
        ddksi_tau1 = ddKsi_tau1.popleft()
        

        eta_1 = -SH*(Ksi_des[0] -  ksi_tau1) + VH* dksi_tau1 #+ lH*ddksi_tau1

        ###

        ksi_tau2 = Ksi_tau2.popleft()
        dksi_tau2 = dKsi_tau2.popleft()
        ddksi_tau2 = ddKsi_tau2.popleft()


        eta_2 = -SA*(Ksi_des[1] -  ksi_tau2) + VA* dksi_tau2 #+ lA*ddksi_tau2

        
        Etas = Matrix([eta_1, eta_2])
        Tcltr = U @ Etas

        # eq = -3.0*9.81*0.2*np.sin(np.pi - np.pi/4 - np.pi)

        data.ctrl[0] = t_s1_plt1[i]#+t_s1_plt2[i]#float(Tcltr[0]+Tcltr[1])#eq#Ht_s1_plt1[i]#
        data.ctrl[1] = t_s1_plt2[i]#float(Tcltr[1])#t_s1_plt2[i]#
        time +=dt
        i+=1
        mj.mj_forward(model,data)
        
        mj.mj_step(model, data)

    print("cltr",t_s1_plt1[i] )
    print(i)
    if (data.time>=simend):
        break

    # get framebuffer viewport
    viewport_width, viewport_height = glfw.get_framebuffer_size(
        window)
    viewport = mj.MjrRect(0, 0, viewport_width, viewport_height)

    #print camera configuration (help to initialize the view)
    if (print_camera_config==1):
        print('cam.azimuth =',cam.azimuth,';','cam.elevation =',cam.elevation,';','cam.distance = ',cam.distance)
        print('cam.lookat =np.array([',cam.lookat[0],',',cam.lookat[1],',',cam.lookat[2],'])')

    # Update scene and render
    mj.mjv_updateScene(model, data, opt, None, cam,
                       mj.mjtCatBit.mjCAT_ALL.value, scene)
    mj.mjr_render(viewport, scene, context)

    # swap OpenGL buffers (blocking call due to v-sync)
    glfw.swap_buffers(window)

    # process pending GUI events, call GLFW callbacks
    glfw.poll_events()

glfw.terminate()

# plt.plot(ksides2, label="des2")
plt.plot(ArrayKsi_2, label="real2")
# plt.plot(ksides1, label="des1")
plt.plot(ArrayKsi_1, label="real1")

plt.plot([JA3[0,i] for i in range(len(JA3)//2)],label="des1") # сумма синергий
plt.plot([JA3[1,i] for i in range(len(JA3)//2)], label="des2")
plt.legend()
plt.show()
