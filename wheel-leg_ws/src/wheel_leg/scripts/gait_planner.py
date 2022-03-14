#!/usr/bin/python
import numpy as np
from LIPM_3D import LIPM3D
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3
from wheel_leg.msg import Footprint
from std_msgs.msg import  Float64
import rospy
import math

rospy.init_node('bipedal_planner')
foot_print_pub = rospy.Publisher('wheel_leg/footprint',Footprint,queue_size=1)

# %% ---------------------------------------------------------------- LIPM control
print('\n--------- Program start from here ...')
# stance_leg = list()
COM_pos_x = list()
COM_pos_y = list()
left_foot_pos_x = list()
left_foot_pos_y = list()
left_foot_pos_z = list()
right_foot_pos_x = list()
right_foot_pos_y = list()
right_foot_pos_z = list()

# Initialize the COM position and velocity
COM_pos_0 = [0.0, 0.0, 0.2]
COM_v0 = [0.0, -0.6]

# Initialize the foot positions
left_foot_pos = [-0.0, 0.1, 0]
right_foot_pos = [-0.0, -0.1, 0]

delta_t = 0.02
rat = rospy.Rate(int(1/delta_t))

s_x_table = [0.0 ,0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
s_y_table = [0.2 ,0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
a = 1.0
b = 1.0
theta = 0.0

LIPM_model = LIPM3D(dt=delta_t, T_sup=0.5)
LIPM_model.initializeModel(COM_pos_0, left_foot_pos, right_foot_pos)

LIPM_model.support_leg = 'right_leg' # set the support leg to right leg in next step
if LIPM_model.support_leg is 'left_leg':
    support_foot_pos = LIPM_model.left_foot_pos
    LIPM_model.p_x = LIPM_model.left_foot_pos[0]
    LIPM_model.p_y = LIPM_model.left_foot_pos[1]
else:
    support_foot_pos = LIPM_model.right_foot_pos
    LIPM_model.p_x = LIPM_model.right_foot_pos[0]
    LIPM_model.p_y = LIPM_model.right_foot_pos[1]

LIPM_model.x_0 = LIPM_model.COM_pos[0] - support_foot_pos[0]
LIPM_model.y_0 = LIPM_model.COM_pos[1] - support_foot_pos[1]
LIPM_model.vx_0 = COM_v0[0]
LIPM_model.vy_0 = COM_v0[1]


step_num = 0
total_time = 10 # seconds
global_time = 0

swing_data_len = int(LIPM_model.T_sup/delta_t)
swing_foot_pos = np.zeros((swing_data_len, 3))
j = 0

switch_index = swing_data_len


# calculate the next foot locations, with modification, stable
x_0, vx_0, y_0, vy_0 = LIPM_model.calculateXtVt(LIPM_model.T_sup) # calculate the xt and yt as the initial state for next step

if LIPM_model.support_leg is 'left_leg':
    x_0 = x_0 + LIPM_model.left_foot_pos[0] # need the absolute position for next step
    y_0 = y_0 + LIPM_model.left_foot_pos[1] # need the absolute position for next step
else:
    x_0 = x_0 + LIPM_model.right_foot_pos[0] # need the absolute position for next step
    y_0 = y_0 + LIPM_model.right_foot_pos[1] # need the absolute position for next step

LIPM_model.calculateFootLocationForNextStep(s_x_table[step_num], s_y_table[step_num], a, b, theta, x_0, vx_0, y_0, vy_0)
# print('p_star=', LIPM_model.p_x_star, LIPM_model.p_y_star)

# calculate the foot positions for swing phase
if LIPM_model.support_leg is 'left_leg':
    right_foot_target_pos = [LIPM_model.p_x_star, LIPM_model.p_y_star, 0]

    delta_x = LIPM_model.p_x_star - LIPM_model.right_foot_pos[0]
    delta_y = LIPM_model.p_y_star - LIPM_model.right_foot_pos[1]

    swing_foot_pos[:,0] = LIPM_model.right_foot_pos[0] + np.array([delta_x*(1.0/(swing_data_len-1) * tao - math.sin(2*math.pi*tao/(swing_data_len-1))/(2*math.pi)) for tao in range(swing_data_len)])
    swing_foot_pos[:,1] = LIPM_model.right_foot_pos[1] + np.array([delta_y*(1.0/(swing_data_len-1) * tao - math.sin(2*math.pi*tao/(swing_data_len-1))/(2*math.pi)) for tao in range(swing_data_len)])
    swing_foot_pos[:,2] = np.array([0.05*(0.5-0.5*math.cos(2*math.pi*tao/(swing_data_len-1))) for tao in range(swing_data_len)])
else:
    left_foot_target_pos = [LIPM_model.p_x_star, LIPM_model.p_y_star, 0]
    delta_x = LIPM_model.p_x_star - LIPM_model.left_foot_pos[0]
    delta_y = LIPM_model.p_y_star - LIPM_model.left_foot_pos[1]

    swing_foot_pos[:,0] = LIPM_model.left_foot_pos[0] + np.array([delta_x*(1.0/(swing_data_len-1) * tao - math.sin(2*math.pi*tao/(swing_data_len-1))/(2*math.pi)) for tao in range(swing_data_len)])
    swing_foot_pos[:,1] = LIPM_model.left_foot_pos[1] + np.array([delta_y*(1.0/(swing_data_len-1) * tao - math.sin(2*math.pi*tao/(swing_data_len-1))/(2*math.pi)) for tao in range(swing_data_len)])      
    swing_foot_pos[:,2] = np.array([0.05*(0.5-0.5*math.cos(2*math.pi*tao/(swing_data_len-1))) for tao in range(swing_data_len)])


for i in range(int(total_time/delta_t)):
    if rospy.is_shutdown():
        break
    global_time += delta_t

    LIPM_model.step()

    # if step_num >= 1:
    if LIPM_model.support_leg is 'left_leg':
        LIPM_model.right_foot_pos = [swing_foot_pos[j,0], swing_foot_pos[j,1], swing_foot_pos[j,2]]
    else:
        LIPM_model.left_foot_pos = [swing_foot_pos[j,0], swing_foot_pos[j,1], swing_foot_pos[j,2]]
    j += 1

    # record data
    # stance_leg.append(LIPM_model.support_leg)
    COM_pos_x.append(LIPM_model.x_t + support_foot_pos[0])
    COM_pos_y.append(LIPM_model.y_t + support_foot_pos[1])
    left_foot_pos_x.append(LIPM_model.left_foot_pos[0])
    left_foot_pos_y.append(LIPM_model.left_foot_pos[1])
    left_foot_pos_z.append(LIPM_model.left_foot_pos[2])
    right_foot_pos_x.append(LIPM_model.right_foot_pos[0])
    right_foot_pos_y.append(LIPM_model.right_foot_pos[1])
    right_foot_pos_z.append(LIPM_model.right_foot_pos[2])


    foot_print = Footprint()
    foot_print.COM.x = LIPM_model.x_t + support_foot_pos[0]
    foot_print.COM.y = LIPM_model.y_t + support_foot_pos[1]
    foot_print.COM.z = COM_pos_0[-1]

    foot_print.Left_foot.x = LIPM_model.left_foot_pos[0]
    foot_print.Left_foot.y = LIPM_model.left_foot_pos[1]
    foot_print.Left_foot.z = LIPM_model.left_foot_pos[2]

    foot_print.Right_foot.x = LIPM_model.right_foot_pos[0]
    foot_print.Right_foot.y = LIPM_model.right_foot_pos[1]
    foot_print.Right_foot.z = LIPM_model.right_foot_pos[2]
   

    foot_print_pub.publish(foot_print)


    # switch the support leg
    if (i > 0) and (j % switch_index == 0):
        j = 0

        LIPM_model.switchSupportLeg() # switch the support leg ,change x_0 and y_0
        step_num += 1

        # theta -= 0.04 # set zero for walking forward, set non-zero for turn left and right

        if step_num >= len(s_x_table): # stop forward after 5 steps
            s_x = 0.0
            s_y = 0.2
        else:
            s_x = s_x_table[step_num]
            s_y = s_y_table[step_num]

        if LIPM_model.support_leg is 'left_leg':
            support_foot_pos = LIPM_model.left_foot_pos
            LIPM_model.p_x = LIPM_model.left_foot_pos[0]
            LIPM_model.p_y = LIPM_model.left_foot_pos[1]
        else:
            support_foot_pos = LIPM_model.right_foot_pos
            LIPM_model.p_x = LIPM_model.right_foot_pos[0]
            LIPM_model.p_y = LIPM_model.right_foot_pos[1]

        # calculate the next foot locations, with modification, stable
        x_0, vx_0, y_0, vy_0 = LIPM_model.calculateXtVt(LIPM_model.T_sup) # calculate the xt and yt as the initial state for next step

        if LIPM_model.support_leg is 'left_leg':
            x_0 = x_0 + LIPM_model.left_foot_pos[0] # need the absolute position for next step
            y_0 = y_0 + LIPM_model.left_foot_pos[1] # need the absolute position for next step
        else:
            x_0 = x_0 + LIPM_model.right_foot_pos[0] # need the absolute position for next step
            y_0 = y_0 + LIPM_model.right_foot_pos[1] # need the absolute position for next step

        LIPM_model.calculateFootLocationForNextStep(s_x, s_y, a, b, theta, x_0, vx_0, y_0, vy_0)
        # print('p_star=', LIPM_model.p_x_star, LIPM_model.p_y_star)

        # calculate the foot positions for swing phase
        if LIPM_model.support_leg is 'left_leg':
            right_foot_target_pos = [LIPM_model.p_x_star, LIPM_model.p_y_star, 0]

            delta_x = LIPM_model.p_x_star - LIPM_model.right_foot_pos[0]
            delta_y = LIPM_model.p_y_star - LIPM_model.right_foot_pos[1]

            swing_foot_pos[:,0] = LIPM_model.right_foot_pos[0] + np.array([delta_x*(1.0/(swing_data_len-1) * tao - math.sin(2*math.pi*tao/(swing_data_len-1))/(2*math.pi)) for tao in range(swing_data_len)])
            swing_foot_pos[:,1] = LIPM_model.right_foot_pos[1] + np.array([delta_y*(1.0/(swing_data_len-1) * tao - math.sin(2*math.pi*tao/(swing_data_len-1))/(2*math.pi)) for tao in range(swing_data_len)])
            swing_foot_pos[:,2] = np.array([0.05*(0.5-0.5*math.cos(2*math.pi*tao/(swing_data_len-1))) for tao in range(swing_data_len)])

        else:
            left_foot_target_pos = [LIPM_model.p_x_star, LIPM_model.p_y_star, 0]
            delta_x = LIPM_model.p_x_star - LIPM_model.left_foot_pos[0]
            delta_y = LIPM_model.p_y_star - LIPM_model.left_foot_pos[1]


            swing_foot_pos[:,0] = LIPM_model.left_foot_pos[0] + np.array([delta_x*(1.0/(swing_data_len-1) * tao - math.sin(2*math.pi*tao/(swing_data_len-1))/(2*math.pi)) for tao in range(swing_data_len)])
            swing_foot_pos[:,1] = LIPM_model.left_foot_pos[1] + np.array([delta_y*(1.0/(swing_data_len-1) * tao - math.sin(2*math.pi*tao/(swing_data_len-1))/(2*math.pi)) for tao in range(swing_data_len)])      
            swing_foot_pos[:,2] = np.array([0.05*(0.5-0.5*math.cos(2*math.pi*tao/(swing_data_len-1))) for tao in range(swing_data_len)])

    rat.sleep()



print('---------  Program terminated')
