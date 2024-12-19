#! /usr/bin/env python2
# -*- coding: utf-8 -*-
from numpy.core.numeric import moveaxis
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import os
import math
from tf.transformations import quaternion_from_euler
from collections import Counter
from std_srvs.srv import Empty

# 坐标变换，将三元转换为四元
def pose_e(x,y,th):#输入x（前）坐标，y（左）坐标，th（平面朝向0～360度）
    new_pose=Pose()
    new_pose.position.x=x
    new_pose.position.y=y
    new_pose.position.z = 0.0
    #机器朝向，平面朝向弧度转化成四元数空间位姿
    q=quaternion_from_euler(0.0,0.0,th/180.0*math.pi)
    new_pose.orientation.x=q[0]
    new_pose.orientation.y=q[1]
    new_pose.orientation.z=q[2]
    new_pose.orientation.w=q[3]
    return  new_pose



# ---------------------------设定目标-----------------------------------
#E
# target_detectionE = pose_e(2.85,-1.0,300)
target_detectionE = pose_e(2.95,-0.80,323)

#D 
# target_detectionD = Pose(Point(2.78, -4.58, 0), Quaternion(0.000, 0.000, -0.714, -0.7))
target_detectionD1_1 = pose_e(2.9,-3.1,320)
target_detectionD1_2 = pose_e(2.95,-3.1,220)

# target_detectionD2_1 = pose_e(2.80,-4.7,320)
# target_detectionD2_2 = pose_e(2.80,-4.7,90)
# target_detectionD2_3 = pose_e(2.85,-4.75,220)

target_detectionD3 = pose_e(2.85,-4.7,325)
target_detectionD4 = pose_e(2.85,-4.7,230)

# B
# target_detectionB = Pose(Point(4.63, -4.04, 0), Quaternion(0.000, 0.000, -0.695, 0.719))
target_detectionB1 = pose_e(5.30,-3.20,220)

target_detectionB2_1 = pose_e(4.55,-4.8,220)
target_detectionB2_2 = pose_e(4.55,-4.8,90)
target_detectionB2_3 = pose_e(4.55,-4.8,320)

#C
# target_detectionC= Pose(Point(4.73,-0.58 ,0.000), Quaternion(0.000, 0.000, 0.732, 0.682))
target_detectionC1 = pose_e(5.2,-1.7,160)
target_detectionC2_1 = pose_e(4.55,-0.1,160)
target_detectionC2_2 = pose_e(4.55,-0.1,270)
target_detectionC2_3 = pose_e(4.55,-0.1,25)

# F1
# target_detectionF1= Pose(Point(0.948, -0.36, 0.000), Quaternion(0.000, 0.000, -0.63, 0.776))
target_detectionF1 = pose_e(0.95,-0.1,270)
target_detectionF2 = pose_e(0.98,-2.5,270)

target_detectionF3= pose_e(1.90,-2.4,235)
target_detectionF4= pose_e(1.90,-4.6,130)

target_detectionF5 = pose_e(0.5,-4.6,315)
# target_detectionF6 = pose_e(1,-4.6,250)
target_detectionF7 = pose_e(0.15,-4.4,45)
target_detectionF8 = pose_e(0.15,-2.5,300)

target_detectionF9 = pose_e(0.9,-2.4,70)

target_detectionF10 = pose_e(0.85,-2.4,90)
target_detectionF11 = pose_e(0.85,-0.4,90)

# target_detectionF3= pose_e(1.60,-2.5,70)
# target_detectionF4= pose_e(1.80,-2.4,235)

# target_detectionF5= pose_e(1.50,-4.6,315)
# target_detectionF6= pose_e(1.70,-4.6,130)

# target_detectionF7 = pose_e(0.40,-4.6,250)
# target_detectionF8 = pose_e(0.20,-4.4,45)

# target_detectionF9 = pose_e(0.30,-2.5,100)
# target_detectionF10 = pose_e(0.20,-2.5,300)

# target_detectionF11 = pose_e(0.9,-2.2,90)
# target_detectionF12 = pose_e(0.9,-0.3,90)


# ---------------------------设定停车位置目标-----------------------------------
# target_parking = pose_e(-0.09,0.20,180)
target_parking = pose_e(0.1,0,180)

targets=[
        target_detectionD3,
        target_detectionD4,
        target_detectionB1,
        target_detectionB2_1,
        target_detectionB2_3,
        target_detectionC1,
        target_detectionC2_1,
        target_detectionC2_3,
        target_detectionF1,
        target_detectionF2,
        target_detectionF3,
        target_detectionF4,
        target_detectionF5,
        target_detectionF7,
        target_detectionF8,
        target_detectionF9,
        target_detectionF10,
        target_detectionF11,
        target_parking
        ]

pos_flag = 0
resend_flag = 0

bridge = CvBridge()
B_list=[]
C_list=[]
D_list=[]
E_list=[]
F_list=[]
type_B=type_C=type_D=type_E=type_F="Unknown"
number_F=0
jump_flag="0"
maxB_count=0
maxC_count=0
maxD_count=0
maxE_count=0

# 识别过程中的图片数据保存下来
def callback_function(data):
    # 创建CvBridge对象，用于将ROS图像消息转换为OpenCV图像
    bridge = CvBridge()
    # 将ROS图像消息转换为OpenCV图像
    cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    # 设置保存图像的目录和文件名
    save_dir = "/home/ucar/ucar_ws/src/logic_moudle/qr_detection"
    filename = os.path.join(save_dir, "{}.jpg".format(callback_function.counter))
    # 保存OpenCV图像为JPG文件
    cv2.imwrite(filename, cv_image)
    #rospy.loginfo("Image saved to {}".format(filename))
    # 更新文件名计数器
    callback_function.counter += 1

#前往导航目标点位
def goto_point(point):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose = point
    goal_pub.publish(goal_pose)
    print('goal:',point)

# 麦克风语音唤醒
def mic_call_back(data):
    global pos_flag
    global targets
    # 前往第一个点
    goto_point(targets[pos_flag])

# 根据输入的列表，确定是否足以判断房间植被类型
def find_most_common(numbers):
    if not numbers:
        return 0
    # 使用Counter类创建一个数字频次字典
    freq_dict = Counter(numbers)
    # 使用most_common()方法找出出现次数最多的数字及其出现次数
    most_common_num, count = freq_dict.most_common(1)[0]
    if count>=4:
        if most_common_num==12 or most_common_num==13 or most_common_num==14 or most_common_num==15:
            return 1
        else:
            return 0
    else:
        return 0
    
#根据输入的列表，返回出现次数最多的数字   
def find_most_common_2(numbers):
    if not numbers:
        return 0
    # 使用Counter类创建一个数字频次字典
    freq_dict = Counter(numbers)
    # 使用most_common()方法找出出现次数最多的数字及其出现次数
    most_common_num, count = freq_dict.most_common(1)[0]
    return most_common_num

# 根据输入的列表，寻找出现次数第5的数字，和出现次数，进而确定是否足以判断F房间
def find_fifth_common(numbers):
    if not numbers:
        return 0
    # 使用Counter类创建一个数字频次字典
    freq_dict = Counter(numbers)
    # 使用most_common()方法找出出现次数前五的数字及其出现次数
    top_five = freq_dict.most_common(5)
    # 返回出现次数第五的数字及其出现次数
    if len(top_five) >= 5:
        fifth_common_num, count = top_five[4]
        if count>=4:
            return 1
        else:
            return 0
    else:
        return 0

# 根据输入的列表，返回出现次数前5的数字
def find_fifth_common_2(numbers):
    if not numbers:
        return 0
    # 使用Counter类创建一个数字频次字典
    freq_dict = Counter(numbers)
    # 使用most_common()方法找出出现次数前五的数字及其出现次数
    top_five = freq_dict.most_common(5)
    # 返回出现次数第五的数字
    return  [num[0]  for num in top_five]


# 根据yolo返回的信息将识别结果放入对应房间列表
def classes_callback(msg):
    global pos_flag,pos_B_finish,pos_C_finish,pos_D_finish
    global B_list, C_list, D_list,E_list,F_list
    global type_B,type_C,type_D,type_E,type_F
    global number_F
    global jump_flag
    num_boxes=0
    num_boxes = len(msg.bounding_boxes)

    i=0
    while i<num_boxes:
        img_id = int(msg.bounding_boxes[i].id)
        img_prob = msg.bounding_boxes[i].probability
        # img_class = msg.bounding_boxes[0].Class
        # img_xmin = int(msg.bounding_boxes[0].xmin)
        # img_xmax = msg.bounding_boxes[0].xmax
        # img_ymin = int(msg.bounding_boxes[0].ymin)
        # img_ymax = msg.bounding_boxes[0].ymax
        if img_prob>0.5:
            loc_pose = rospy.wait_for_message('pose',PoseWithCovarianceStamped)
            if pos_flag==0 :
                if loc_pose.pose.pose.position.y > -1.8:#需要调整
                    if img_id==12 or img_id==13 or img_id==14 or img_id==15:
                        E_list.append(img_id)
                    if find_most_common(E_list):
                        if type_E=='Unknown':
                            type_E=class_zhibei(E_list)
                            rospy.loginfo("E_list")
                            rospy.loginfo( E_list)
                            rospy.loginfo("type_E" + type_E)
                            rospy.loginfo('pos_flg'+str(pos_flag))
                            print(loc_pose)
                else:
                    if img_id==12 or img_id==13 or img_id==14 or img_id==15:
                        D_list.append(img_id)
                    if find_most_common(D_list):
                        if type_D=="Unknown":
                            if loc_pose.pose.pose.position.y<-3.1:
                                jump_flag='D'
                                type_D=class_zhibei(D_list)
                                rospy.loginfo("D_list")
                                rospy.loginfo(D_list)
                                rospy.loginfo("type_D" + type_D)
                                rospy.loginfo('pos_flg'+str(pos_flag))
                                print( loc_pose)

            elif 1<=pos_flag<=2:
                if loc_pose.pose.pose.position.x<3.7:
                    if img_id==12 or img_id==13 or img_id==14 or img_id==15:
                        D_list.append(img_id)
                    if find_most_common(D_list):
                        if type_D=="Unknown":
                            if loc_pose.pose.pose.position.y<-3.1:
                                jump_flag='D'
                                type_D=class_zhibei(D_list)
                                rospy.loginfo("D_list")
                                rospy.loginfo(D_list)
                                rospy.loginfo("type_D" + type_D)
                                rospy.loginfo('pos_flg'+str(pos_flag))
                                print(loc_pose)
                else:
                    if img_id==12 or img_id==13 or img_id==14 or img_id==15:
                        B_list.append(img_id)
                    if find_most_common(B_list):
                        if type_B=="Unknown":
                            if loc_pose.pose.pose.position.y<-3.1 and loc_pose.pose.pose.position.x >3.7:
                                jump_flag='B'
                                type_B=class_zhibei(B_list)
                                rospy.loginfo("B_list" )
                                rospy.loginfo(B_list)
                                rospy.loginfo("type_B" + type_B)
                                rospy.loginfo('pos_flg'+str(pos_flag))
                                print( loc_pose)

            elif 3<=pos_flag<=5:
                if loc_pose.pose.pose.position.y < -3.0:
                    if img_id==12 or img_id==13 or img_id==14 or img_id==15:
                        B_list.append(img_id)
                    if find_most_common(B_list):
                        if type_B=="Unknown":
                            if loc_pose.pose.pose.position.y<-3.1 and loc_pose.pose.pose.position.x >3.7:
                                jump_flag='B'
                                type_B=class_zhibei(B_list)
                                rospy.loginfo("B_list" )
                                rospy.loginfo(B_list)
                                rospy.loginfo("type_B" + type_B)
                                rospy.loginfo('pos_flg'+str(pos_flag))
                                print( loc_pose)

                else:
                    if img_id==12 or img_id==13 or img_id==14 or img_id==15:
                        C_list.append(img_id)
                    if find_most_common(C_list):
                        if type_C=="Unknown":
                            if loc_pose.pose.pose.position.y>-1.6 and loc_pose.pose.pose.position.x >3.7:
                                jump_flag='C'
                                type_C=class_zhibei(C_list)
                                rospy.loginfo("C_list")
                                rospy.loginfo( C_list)
                                rospy.loginfo("type_C" + type_C)
                                rospy.loginfo('pos_flg'+str(pos_flag))
                                print( loc_pose)
                           

            elif 6<=pos_flag<=8:
                if loc_pose.pose.pose.position.x >3.7:
                    if img_id==12 or img_id==13 or img_id==14 or img_id==15:
                        C_list.append(img_id)
                    if find_most_common(C_list):
                        if type_C=="Unknown":
                            if loc_pose.pose.pose.position.y>-1.6 and loc_pose.pose.pose.position.x >3.7:
                                jump_flag='C'
                                type_C=class_zhibei(C_list)
                                rospy.loginfo("C_list")
                                rospy.loginfo( C_list)
                                rospy.loginfo("type_C" + type_C)
                                rospy.loginfo('pos_flg'+str(pos_flag))
                                print( loc_pose)
                       
                elif loc_pose.pose.pose.position.x <3.7 and loc_pose.pose.pose.position.x >2.0 and loc_pose.pose.pose.position.y>-3:
                    if img_id==12 or img_id==13 or img_id==14 or img_id==15:
                        E_list.append(img_id)
                    if find_most_common(E_list):
                        type_E=class_zhibei(E_list)
                else:
                    pass


            elif pos_flag>=9:
                if img_prob>0.6:
                    if 0<=img_id<=11:
                        F_list.append(img_id)
                    if find_fifth_common(F_list):
                        if type_F=="Unknown":
                            jump_flag='F'
                            type_F,number_F=class_guoshi(F_list)
                            rospy.loginfo("F_list" )
                            rospy.loginfo(F_list)
                            rospy.loginfo("type_F" + type_F)
                            rospy.loginfo("number_F" + str(number_F))
                            rospy.loginfo('pos_flg'+str(pos_flag))
                            print( loc_pose)
                        
        i+=1
        pub=rospy.Publisher('move_base/result',MoveBaseActionResult,queue_size=10)
        fake_msg=MoveBaseActionResult()
        fake_msg.status.status=100
        fake_msg.status.text='100'
        pub.publish(fake_msg)


# 新函数！取出现频次最高的标号
# def filter_list(my_list):
#     dic = {}
#     my_newlist = []
#     for i in my_list:
#         dic[i] = dic.get(i, 0) + 1
#     counts = dic.values()
#     counts = sorted(counts, reverse=True)

#     for i in counts:
#         for key, value in dic.items():
#             if i == value:
#                 my_newlist.append(key)
#         if len(my_newlist) >= 1:
#             break
#     return my_newlist

# 新函数！取出现频次5最高的标号
# def filter_list_5(my_list):
#     dic = {}
#     my_newlist = []
#     for i in my_list:
#         dic[i] = dic.get(i, 0) + 1
#     counts = dic.values()
#     counts = sorted(counts, reverse=True)

#     for i in counts:
#         for key, value in dic.items():
#             if i == value:
#                 my_newlist.append(key)
#         if len(my_newlist) >= 5:
#             break
#     return my_newlist

# 导航函数
def navi_fail_callback(msg):
    global pos_flag, resend_flag
    if msg.status_list != []:
        print('navi_fail',msg.status_list)
        if msg.status_list[0].status == 4 and resend_flag == 0:
            rospy.loginfo("Navigation Failed!")
            pos_flag+=1            
            goto_point(targets[pos_flag])
            resend_flag = 1

# 根据房间内识别到的植被种类，对房间类型进行标定
def class_zhibei(zhibei_list):
    i= find_most_common_2(zhibei_list)
    if i==12:
        type="黄瓜植被"
    elif i==13:
        type="水稻植被"
    elif i==14:
        type="小麦植被"
    elif i==15:
        type="玉米植被"
    else:
        type="Unknown"
    return type

# 根据房间内识别到的果实信息，标定房间果实类型和数量
def class_guoshi(guoshi_list):
    guoshi_list= find_fifth_common_2(guoshi_list)
    huanggua_num=xigua_num=yumi_num=0
    num=0
    for i in guoshi_list:
        if i==0:
            huanggua_num+=1
        elif i==1:
            huanggua_num+=3
        elif i==2:
            huanggua_num+=2
        elif i==3:
            huanggua_num+=2
        elif i==4:
            xigua_num+=1
        elif i==5:
            xigua_num+=1
        elif i==6:
            xigua_num+=1
        elif i==7:
            xigua_num+=1
        elif i==8:
            yumi_num+=2
        elif i==9:
            yumi_num+=2
        elif i==10:
            yumi_num+=1
        elif i==11:
            yumi_num+=1
        else:
            continue
    if huanggua_num>=xigua_num and huanggua_num>=yumi_num:
        type="黄瓜"
        num=huanggua_num
    elif xigua_num>=huanggua_num and xigua_num>=yumi_num:
        type="西瓜"
        num=xigua_num
    elif yumi_num>=xigua_num and yumi_num>=huanggua_num:
        type="玉米"
        num=yumi_num
    else:
        type="Unkwon"
        num=0
    return type,num

# 植被类型的最终确认
def zhibei_final(ty_B,ty_C,ty_D,ty_E):
    if ty_B!='Unkwon'and ty_C!='Unkwon'and ty_D!='Unkwon'and ty_E!='Unkwon':
        return ty_B,ty_C,ty_D,ty_E
    # 未完待续

# 新函数！取出现频次最高的标号
def filter_list(my_list):
    if len(my_list)<=2:
        my_list=[-1]
    dic = {}
    my_newlist = []
    for i in my_list:
        dic[i] = dic.get(i, 0) + 1
    counts = dic.values()
    counts = sorted(counts, reverse=True)
    for i in counts:
        for key, value in dic.items():
            if i == value:
                my_newlist.append(key)
        if len(my_newlist) >= 1:
            break
    return my_newlist ,counts[0]#种类，次数

def three_exclude_1():
   # 3排1
        global maxB_count,maxC_count,maxD_count,maxE_count
        global B_list,C_list,D_list,E_list
        global type_B,type_C,type_D,type_E
        list1=[B_list[0],C_list[0],D_list[0],E_list[0]]
        list0=[12,13,14,15]
        new_list1=list1[:]
        if -1 in new_list1:
            # new_list1.remove(-1)
            new_list1=[x for x in new_list1 if x!=-1]
            index_unknown=[i for i in range(len(list1)) if list1[i]==-1]
            if len(new_list1)==3:
                #index_unknown=list1.index(-1)
                list1[index_unknown[0]]=[x for x in list0 if x not in new_list1][0]
            elif len(new_list1)==2:
                list1[index_unknown[0]]=[x for x in list0 if x not in new_list1][0]
                list1[index_unknown[1]]=[x for x in list0 if x not in new_list1][1]
            elif len(new_list1)==1:
                list1[index_unknown[0]]=[x for x in list0 if x not in new_list1][0]
                list1[index_unknown[1]]=[x for x in list0 if x not in new_list1][1]
                list1[index_unknown[2]]=[x for x in list0 if x not in new_list1][2]
            else:
                list1[index_unknown[0]]=[x for x in list0 if x not in new_list1][0]
                list1[index_unknown[1]]=[x for x in list0 if x not in new_list1][1]
                list1[index_unknown[2]]=[x for x in list0 if x not in new_list1][2]
                list1[index_unknown[3]]=[x for x in list0 if x not in new_list1][3]
        # new_list1=list(set(list1)) 
        # if len(new_list1)==3:
        #   count_list=[maxB_count,maxC_count,maxD_count,maxE_count]
        #   duplicates = []
        #   for i, item in enumerate(list1):
        #     if item in list1[:i]:
        #        duplicates.append(i)
        #   if(count_list[duplicates[0]]<count_list[duplicates[1]]):
        #     list1[duplicates[0]]=set(list0)-set(new_list1)
        #   else:
        #     list1[duplicates[1]]=set(list0)-set(new_list1)
        B_list[0]=list1[0]
        C_list[0]=list1[1]
        D_list[0]=list1[2]
        E_list[0]=list1[3]
        type_B=class_zhibei(B_list)
        type_C=class_zhibei(C_list)
        type_D=class_zhibei(D_list)
        type_E=class_zhibei(E_list)

def play_zhibei(type):
    if type=="黄瓜植被":
        os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/huanggua.wav")
    elif type=="水稻植被":
        os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/shuidao.wav")
    elif type=="小麦植被":
        os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/xiaomai.wav")
    elif type=="玉米植被":
        os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/yumi.wav")
    else:
        os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/huanggua.wav")

def goal_callback(msg):
    global pos_flag
    global B_list, C_list, D_list,E_list,F_list
    global targets
    global type_B,type_C,type_E,type_F
    global number_F
    global jump_flag
    global maxB_count,maxC_count,maxD_count,maxE_count
    if jump_flag=='D':
        cancel_goal =rospy.ServiceProxy('move_base/cancel', Empty)
        cancel_goal
        pos_flag=2
        goto_point(targets[pos_flag])
        jump_flag='0'
    elif jump_flag=='B':
        cancel_goal =rospy.ServiceProxy('move_base/cancel', Empty)
        cancel_goal
        pos_flag=5
        goto_point(targets[pos_flag])
        jump_flag='0'
    elif jump_flag=='C':
        cancel_goal =rospy.ServiceProxy('move_base/cancel', Empty)
        cancel_goal
        pos_flag=8
        goto_point(targets[pos_flag])
        jump_flag='0'
    elif jump_flag=='F':
        pos_flag=16
        goto_point(targets[pos_flag])
        jump_flag='0'
    elif msg.status.status == 3:
        # loc_pose = rospy.wait_for_message('/odom',Odometry)  #防跳点
        pos_flag += 1
        if pos_flag<len(targets):
            goto_point(targets[pos_flag])  
        else:
            #标签对应关系
            '''
            序号 label
            0   huanggua1
            1   huanggua2
            2   huanggua3
            3   huanggua4
            4   xigua5
            5   xigua6
            6   xigua7
            7   xigua8
            8   yumi9
            9   yumi10
            10  yumi11
            11  yumi12
            12  huangguazhibei
            13  shuidaozhibei
            14  xiaomaizhibei
            15  yumizhibei
            '''
            rospy.loginfo("原始结果：")
            rospy.loginfo("======================================")
            rospy.loginfo("B_list: %s", B_list)
            rospy.loginfo("======================================")   
            rospy.loginfo("C_list: %s", C_list)
            rospy.loginfo("======================================")
            rospy.loginfo("D_list: %s", D_list)
            rospy.loginfo("======================================")
            rospy.loginfo("E_list: %s", E_list)
            rospy.loginfo("======================================")
            rospy.loginfo("F_list: %s", F_list)
            rospy.loginfo("======================================")

        #    # 取出现频次的标号
        #     B_list = find_most_common_2(B_list)
        #     C_list = find_most_common_2(C_list)
        #     D_list = find_most_common_2(D_list)
        #     E_list = find_most_common_2(E_list)
        #     F_list = find_fifth_common_2(F_list)

            # rospy.loginfo("处理结果：")
            # rospy.loginfo("======================================")
            # rospy.loginfo("B_list: %s", B_list)
            # rospy.loginfo("======================================")   
            # rospy.loginfo("C_list: %s", C_list)
            # rospy.loginfo("======================================")
            # rospy.loginfo("D_list: %s", D_list)
            # rospy.loginfo("======================================")
            # rospy.loginfo("E_list: %s", E_list)
            # rospy.loginfo("======================================")
            # rospy.loginfo("F_list: %s", F_list)
            # rospy.loginfo("======================================")

            # type_B=class_zhibei(B_list)
            # type_C=class_zhibei(C_list)
            # type_D=class_zhibei(D_list)
            # type_E=class_zhibei(E_list)


            B_list,maxB_count= filter_list(B_list)
            C_list,maxC_count= filter_list(C_list)
            D_list ,maxD_count= filter_list(D_list)
            E_list ,maxE_count= filter_list(E_list)
            three_exclude_1()

            type_F,number_F=class_guoshi(F_list)

            # type_B,type_C,type_D,type_E = zhibei_final(type_B,type_C,type_D,type_E)

            rospy.loginfo("最终结果：")
            rospy.loginfo("======================================")
            rospy.loginfo("B房间的类型为："+ type_B)
            rospy.loginfo("C房间的类型为："+ type_C)
            rospy.loginfo("D房间的类型为："+ type_D)
            rospy.loginfo("E房间的类型为："+ type_E)
            rospy.loginfo("F房间的果实种类为："+ type_F+ "数量为："+ str(number_F))


            os.system("play /home/ucar/ucar_ws/src/logic_moudle/wav/finish.wav")
            os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/B.wav")
            play_zhibei(type_B)
            os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/C.wav")
            play_zhibei(type_C)
            os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/D.wav")
            play_zhibei(type_D)
            os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/E.wav")
            play_zhibei(type_E)

            if type_F=="黄瓜":
                os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/F-huanggua.wav")
            elif type_F=="西瓜":
                os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/F-xigua.wav")
            elif type_F=="玉米":
                os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/F-yumi.wav")
            else:
                os.system("play /home/ucar/catkin_ws/src/logic_moudle/wav/F-huanggua.wav")

            if 1<=number_F<=8:
                code = 'play /home/ucar/catkin_ws/src/logic_moudle/wav/num-{}.wav' .format(number_F)
                os.system(code)
            else:
                code = 'play /home/ucar/catkin_ws/src/logic_moudle/wav/num-{}.wav' .format(min(max(1,number_F),8))
                os.system(code)



if __name__ == '__main__':
    rospy.init_node('play', anonymous=True)
    
    # 语音唤醒后，麦克风模块发布/start_others消息，msg = 1，此时进入mic_call_back函数
    rospy.Subscriber("/start_others", Int8, mic_call_back)

    # # 每次导航完成时，move_base节点会发布/move_base/result，消息类型是MoveBaseActionResult，此时进入goal_callback函数
    rospy.Subscriber("move_base/result", MoveBaseActionResult, goal_callback)
    # # rospy.Subscriber("move_base/status", GoalStatusArray, navi_fail_callback)

    # # 每次拍照后，yolo节点会识别图片，并发布"/darknet_ros/bounding_boxes"，此时进入classes_callback函数
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes , classes_callback)

    # # 初始化文件名计数器
    # # callback_function.counter = 1
    # # rospy.Subscriber('usb_cam/image_raw', Image, callback_function)

    # # 发布/move_base_simple/goal，给小车指定下一个坐标点
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    
    # # move_base的初始化
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.spin()

