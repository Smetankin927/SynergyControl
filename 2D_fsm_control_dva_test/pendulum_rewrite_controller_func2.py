import mujoco as mj
from mujoco.glfw import glfw
import numpy as np
import os


import matplotlib.pyplot as plt 
import sympy as sym
from sympy import Matrix, init_printing, symbols, solve, eye, Rational, sin, cos, tanh, simplify, Eq, evaluate
from sympy import diag, collect, expand, latex
import numpy as np


print_camera_config = 0 #set to 1 to print camera config
                        #this is useful for initializing view of the model)

# For callback functions
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0


from tryafteronecopyoffindparamssim import U, W, Uinv

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

def create_traj(A, f):
    times = np.linspace(0,11, 11000)
    # A*np.tanh(2*np.pi*f*times)
    s0 = 0
    sk = 1
    tv = 5
    return s0 + (sk-s0)/2 * (1+ np.sin(np.pi/tv * (times - tv/2)))

def cumtrapz(y,x):
    res = np.array([np.trapezoid(y[:i], x[:i]) for i in range(len(x))])
    return res

def createTraj2():
    times = np.linspace(0,11, 11000)
    b_upch = -0.19473
    b_dwch = -0.78947
    vel_prof1 = 0.1/(1+(times-2)**2) * float(1/b_dwch)
    vel_prof2 = 0.1/(1+(times-2)**2) * float(1/b_upch)
    

    sinerg_prof1 = cumtrapz(vel_prof1, times)
    sinerg_prof2 = cumtrapz(vel_prof2, times)

    return sinerg_prof1, sinerg_prof2



def main(A_, f_,  S_H, V_H, taums_H,  S_A, V_A, taums_A, folder1, folder2):
    xml_path = '2D_simple_pendulum.xml' #xml file (assumes this is in the same folder as this file)
    simend = 10 #simulation time

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

    data.qpos[0] = np.pi #+ np.pi/4
    data.qpos[1] = 0 #np.pi/4


    lH = 0.0077
    SH = S_H#1.042
    VH = V_H#0.2

    lA = 0.0557
    SA = S_A#1.4
    VA = V_A#0.3

    i = 0
    time = 0
    dt = 0.001

    f = f_ #1 #Hz
    A = A_#0.1
    t = symbols("t")

    fi_des1 = -A*sin(2*np.pi*f*t)

    fi_des2 = A*sin(2*np.pi*f*t)#-0.349066*sin(2*np.pi*f*t)

    fi_des2_2 = 0
    fi_des1_1 = 0

    taums_1 = taums_H#40
    taums_2 = taums_A#80

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
    ksides_test = create_traj(A_,f_)
    i=0

    etas_1 = []
    etas_2 = []

    cltr1 = []
    cltr2 = []
    sinerg_prof1, sinerg_prof2 = createTraj2()
    while not glfw.window_should_close(window):
        time_prev = data.time

        while (data.time - time_prev < 1.0/60.0):
            
            mj.mj_forward(model,data)
            # we have already fi_des1 and fi_des2 is sympy
            # fi = W @ ksi  ==>  ksi = W.inv() @ fi
            stt = {t:time}

            # if time < 2:
            #     fi_des2_2 = fi_des2.subs(stt)
            #     fi_des1_1 = fi_des1.subs(stt)
            #     Ksi_des = Matrix([fi_des2.subs(stt), 0])  # зависит от t
            # else: 
            # Ksi_des = Matrix([A*tanh(2*np.pi*f*t).subs(stt), 0])
            # print("fides", W@Ksi_des)
            

            Ksi_des = Matrix([0, sinerg_prof2[i]])
            # Ksi_des = Matrix([0, 0])
            # Ksi_des = Matrix([ksides_test[i], 0])
            i+=1
            # Fi_des = Matrix([np.pi/4, np.pi/4])#W@Ksi_des #Matrix([fi_des1.subs(stt), fi_des2.subs(stt)])  # зависит от t
            Fi_des = W@Ksi_des #Matrix([fi_des1.subs(stt), fi_des2.subs(stt)])  # зависит от t

            # Ksi_des = Matrix([0, 0])
            # Ksi_des = W.inv() @ Fi_des #[ 0 , 0.05*sin(2*np.pi*f*t).subs(stt)]#

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
            # print("current ", joint_angular_pos_fi1)
            
            #joint 1
            joint_name1 = "joint1"  # Replace with the name of your joint
            joint_id1 = model.joint(joint_name1).id  # Get the joint ID
            dof_start1 = model.jnt_dofadr[joint_id1]  # Start index of the joint's DOF in qvel/qacc

            # Joint angular acceleration (rad/s²) NOW 
            joint_angular_acceleration_fi2 = data.qacc[dof_start1]
            joint_angular_vel_fi2 = data.qvel[dof_start1]
            joint_angular_pos_fi2 = data.qpos[dof_start1]
            # print("current ", joint_angular_pos_fi2)


            # eq = 3.0*9.81*0.2*np.sin(-joint_angular_pos_fi2-joint_angular_pos_fi1) #fi_1 + fi_2
            # eq1 = 3.0*9.81*3/2*0.4*np.sin(-joint_angular_pos_fi1) + eq #fi_i (-(np.pi - fi_1))

            eq = 3.0*9.81*0.2*np.sin(float(-Fi_des[1]-Fi_des[0]))#(-joint_angular_pos_fi2-joint_angular_pos_fi1) #fi_1 + fi_2
            eq1 = 3.0*9.81*3/2*0.4*np.sin(float(-Fi_des[0])) + eq
            EqKsi = Uinv@Matrix([eq1, eq])

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

            # eta = eta(t-tau) + Seig(ksi_des(t) - ksi(t-tau)) - Veig*dksi(t-tau) - lambda*ddksi(t-tau)

            ksi_tau1 = Ksi_tau1.popleft()
            dksi_tau1 = dKsi_tau1.popleft()
            ddksi_tau1 = ddKsi_tau1.popleft()

            eta_1 = EqKsi[0] -SH*(Ksi_des[0] -  ksi_tau1) + VH* dksi_tau1 #EqKsi[0] + lH*ddksi_tau1

            # print("eta_1 ", eta_1)
            ###

            ksi_tau2 = Ksi_tau2.popleft()
            dksi_tau2 = dKsi_tau2.popleft()
            ddksi_tau2 = ddKsi_tau2.popleft()

            subeta = -SA*(Ksi_des[1] -  ksi_tau2) + VA* dksi_tau2
            eta_2 = EqKsi[1] -SA*(Ksi_des[1] -  ksi_tau2) + VA* dksi_tau2 #EqKsi[1]  #+ lA*ddksi_tau2

            print("eta_2 ", eta_2)#Ksi_des[1] -  ksi_tau2)
            print("EqKsi1 ", EqKsi[1])
            print("subeta ", subeta)
            print("ksidiff ", Ksi_des[1] -  ksi_tau2)
            print("dksi_tau2 ", dksi_tau2)


            print("UUNIV", U@Uinv)
            Etas = Matrix([eta_1,eta_2 ])
            etas_1.append(eta_1)
            etas_2.append(eta_2)

            # Tcltr = U@EqKsi #U @ Etas
            Tcltr = U@Etas

            # joint_angular_pos_fi1

            eqp = 3.0*9.81*0.2*np.sin(-np.pi/4 - np.pi/4)#np.sin(-joint_angular_pos_fi2-joint_angular_pos_fi1) #fi_1 + fi_2
            eqp1 = eqp + 3.0*9.81*3/2*0.4*np.sin(-np.pi/4) #np.sin(-joint_angular_pos_fi1) + eq #fi_i (-(np.pi - fi_1))



            data.ctrl[0] = float(Tcltr[0]) #eq1#round(float(Tcltr[0]),3)# + Tcltr[1])#eq#H eq1 + 
            data.ctrl[1] = float(Tcltr[1]) #eq#round(float(Tcltr[1]), 3)# eq +
            cltr1.append( data.ctrl[0])
            cltr2.append( data.ctrl[1])

            print("eq1 ", eqp1 )
            print("data.ctrl[1] ", data.ctrl[0] )
            time +=dt
            mj.mj_forward(model,data)
            
            mj.mj_step(model, data)


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


    # plt.plot(ksides1, label="desired angles 2")
    # plt.plot(ArrayKsi_1, label = "real angles 2")
    # plt.plot(ksides2, label="desired angles 2")
    # plt.plot(ArrayKsi_2, label = "real angles 2")
    # plt.xlabel("time, ms")
    # plt.ylabel("angle, rad")
    # plt.title("Desired and real angles in simulations")
    # plt.figtext(0.5, -0.05, f"A*sin(2pi*f*t) tH={taums_1} SH={SH} VH={VH} tA={taums_2} SA={SA} VA={VA} A={A}, f={f}", ha="center", fontsize=10, style="italic")
    # plt.subplots_adjust(bottom=0.3)
    # plt.legend()
    # plt.tight_layout()
    # # plt.show()
    # plt.savefig(f"{folder}/two_pendulum{SH}_{VH}_{taums_1}_{SA}_{VA}_{taums_2}_{A}_{f}.png", bbox_inches='tight', pad_inches=0.5)
    # plt.close()


    # folder = "pres"

# First plot
    # fig, ax = plt.subplots()
    # ax.set_xlabel("time, ms")
    # ax.set_ylabel("angle, rad")
    # ax.set_title("Desired and real dynamics variables in simulations")
    # ax.plot(ksides1, label="des 1")
    # ax.plot(ArrayKsi_1, label="real 1")
    # ax.legend()
    # fig.tight_layout()
    # fig.savefig(f"{folder1}/two_pendulum{SH}_{VH}_{taums_1}_{SA}_{VA}_{taums_2}_{A}_{f}.png", bbox_inches='tight', pad_inches=0.5)
    # plt.close(fig)

    # # Second plot
    # fig, ax = plt.subplots()
    # ax.set_xlabel("time, ms")
    # ax.set_ylabel("angle, rad")
    # ax.set_title("Desired and real dynamics variables in simulations")
    # ax.plot(ksides2, label="desired 2")
    # ax.plot(ArrayKsi_2, label="real 2")
    # ax.legend()
    # fig.tight_layout()
    # fig.savefig(f"{folder2}/two_pendulum2{SH}_{VH}_{taums_1}_{SA}_{VA}_{taums_2}_{A}_{f}.png", bbox_inches='tight', pad_inches=0.5)
    # plt.close(fig)


###############################
    # First plot

    plt.rcParams['text.usetex'] = False
    plt.figure()  # Create a new figure
    plt.xlabel("time, ms")
    plt.ylabel("angle, rad")
    plt.title("Desired and real dynamics variables in simulations")
    plt.plot(ksides1, label=r"desired $\xi_1$")
    plt.plot(ArrayKsi_1, label=r"real $\xi_1$")
    plt.legend()
    plt.tight_layout()
    # plt.savefig(f"{folder1}/two_pendulum_th.png", bbox_inches='tight', pad_inches=0.5)
    # plt.close()
    # plt.show()

    # Second plot
    plt.figure()  # Create a new figure
    plt.xlabel("time, ms")
    plt.ylabel("angle, rad")
    plt.title("Desired and real dynamics variables in simulations")
    plt.plot(ksides2, label=r"desired $\xi_2$")
    plt.plot(ArrayKsi_2, label=r"real $\xi_2$")
    plt.legend()
    plt.tight_layout()
    plt.savefig(f"{folder2}/two_pendulum2SH_th1.png", bbox_inches='tight', pad_inches=0.5)
    # plt.close()
    plt.show()

    # plt.figure()  # Create a new figure
    # plt.xlabel("time, ms")
    # plt.ylabel("angle, rad")
    # plt.title("Desired and real dynamics variables in simulations")
    # plt.plot(etas_2, label="eta_2")
    # plt.plot(etas_1, label="eta_1")
    # plt.legend()
    # plt.tight_layout()
    # plt.savefig(f"{folder2}/two_pendulum2{SH}_{VH}_{taums_1}_{SA}_{VA}_{taums_2}_{A}_{f}.png", bbox_inches='tight', pad_inches=0.5)
    # # plt.close()
    # # plt.show()

    # plt.figure()  # Create a new figure
    # plt.xlabel("time, ms")
    # plt.ylabel("angle, rad")
    # plt.title("Desired and real dynamics variables in simulations")
    # plt.plot(cltr1, label="ct_1")
    # plt.plot(cltr2, label="ct_2")
    # plt.legend()
    # plt.tight_layout()
    # plt.savefig(f"{folder2}/two_pendulum2S_{VH}_{taums_1}_{SA}_{VA}_{taums_2}_{A}_{f}.png", bbox_inches='tight', pad_inches=0.5)
    # # plt.close()
    # plt.show()
if __name__ == "__main__":
    print("No")
    # main(0.1, 0.5)