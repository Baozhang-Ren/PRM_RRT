#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sun Nov 18 10:36:04 2018

@author: baozhang
"""
from gazebo_msgs.msg import ModelState
import rospy
import ast
from gazebo_msgs.msg import ModelStates
pub_set_model_state= rospy.Publisher('/gazebo/set_model_state', ModelState,queue_size=50)
#rospy.Subscriber('/gazebo/model_states', ModelStates,save_robot_state)

rospy.init_node('ass2_piano')
'''file_path = '/home/baozhang/Desktop/result/result/routs/reduce_map_fda_5.txt'
f = open(file_path)
routs = []
temp = []
for line in f:
    # print(line)
    temp.append(line)
#
for i in range(len(temp)):
    routs.append(ast.literal_eval(temp[i]))'''
routs = [((-4.3298282500173, 1.431189956918935, 0.12859062318763692), (0.33548403155574447, 0.22854038878586014, -0.8304355327310744, -0.3815712007507496)), ((-3.839857624059859, 1.0843698754877735, 0.3054554642132299), (0.8390720951174016, 0.24145482890962483, -0.25564234764488275, -0.41509586228018575)), ((-2.8722136454326996, -0.0971508214604011, 1.0387869553024056), (-0.046111964134279496, 0.648389421102053, -0.21554648104053403, 0.7287005968692946)), ((-1.797910248499985, 0.7731528451470489, 0.1167059656411722), (0.004367364239035213, -0.8042639450425592, -0.5900367097091193, 0.07069026828227098)), ((-1.5549436767079534, -0.493088037426233, 0.7109708286743479), (0.5565173289803138, 0.6249962322995903, 0.10036113042808022, 0.5381410741200969)), ((0.12137074501875844, -1.0342391692103234, 0.48474498269025124), (-0.3096249904169215, -0.8129431079087693, -0.15616873388941316, -0.4678324434755781)), ((0.9359368688602405, -2.3824121727504277, 0.9019289341538431), (0.024606336623009286, -0.16648910616937904, 0.7529838015030462, -0.636153519520986)), ((1.1272103635825186, -2.0311156601376945, 1.5478667779951343), (0.05406464675493242, -0.9485778784401823, -0.23964938091929522, 0.19961261666083596)), ((1.8212299958676645, -2.790954894880744, 2.20678903969001), (-0.6788280226139377, 0.58718708842202, 0.44069834169299493, -0.01374083452947424)), ((3.5977302103591775, -3.2403434000396913, 2.6453184245794694), (-0.6081193986673016, 0.2712593355060089, -0.4977853804223263, -0.5557147513816397)), ((4.985305934851759, -4.409166195560319, 1.6737286872079995), (0.9971758102775988, 0.030087659104572898, -0.06466955126438487, -0.023515639606109588)), ((5.456093761795275, -5.972338004431674, 2.5786339393895528), (0.22469742701713355, 0.04323188830580837, 0.4809262984656075, -0.8463757827168518)), ([4.98192963164758, -5.856713641494153, 2.9451312636213642], [0.22252616059627714, -0.10402895583604971, -0.4500787435318473, -0.8585389966790878])]
print routs
for modelstate in routs:
    msg = ModelState()
    msg.model_name = "piano2"
    msg.pose.position.x = modelstate[0][0]
    msg.pose.position.y = modelstate[0][1]
    msg.pose.position.z = modelstate[0][2]
    msg.pose.orientation.x = modelstate[1][0]
    msg.pose.orientation.y = modelstate[1][1]
    msg.pose.orientation.z = modelstate[1][2]
    msg.pose.orientation.w = modelstate[1][3]
    msg.twist.linear.x = 0.0
    msg.twist.linear.y = 0.0
    msg.twist.linear.z = 0.0
    msg.twist.angular.x = 0.0
    msg.twist.angular.y = 0.0
    msg.twist.angular.z = 0.0
    print msg
    pub_set_model_state.publish(msg)
    rospy.sleep(1)