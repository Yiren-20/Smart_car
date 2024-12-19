#! /usr/bin/env python2
# -*- coding: utf-8 -*-
from numpy.core.numeric import moveaxis
import rospy
from darknet_ros_msgs.msg import BoundingBoxes
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseActionResult, MoveBaseGoal, MoveBaseAction
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import Image
from std_msgs.msg import Int8
from cv_bridge import CvBridge
import cv2
import os
import math
from tf.transformations import quaternion_from_euler

# ===========================================初始化===========================================
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
# ---------------------------设定目标点-----------------------------------
# target_detection1=pose_e(2.7,2.7,0)
target_recognition=pose_e(1.0,0.5,190)
target_left=pose_e(0.45,0.35,220)
target_right = pose_e(0.45,0.65,160)



bridge = CvBridge()
yolo_result = {}
if_recognition=0
pos_flag=0
face_rego=cv2.CascadeClassifier('/home/ucar/FRRB-6/src/logic_moudle/sound_play/scripts/haarcascade_frontalface_default.xml')

def goto_point(point):
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = "map"
    goal_pose.pose = point
    goal_pub.publish(goal_pose)


def mic_call_back(data):
    # 前往第一个点
    goto_point(target_recognition)


def classes_callback(msg):
    global if_recognition
    global yolo_result

    # img_class = msg.bounding_boxes[0].Class
    # print(msg.bounding_boxes)
    if if_recognition == 1 and len(msg.bounding_boxes)==2:
        for box in msg.bounding_boxes:
            img_id = int(box.id)
            img_prob = box.probability
            img_xmin = int(box.xmin)
            # print(img_id)
            if img_prob>0.5:
                if img_id==0 and 'person1' not in yolo_result:
                    yolo_result['person1']=img_xmin
                elif img_id==1 and 'person2' not in yolo_result:
                    yolo_result['person2']=img_xmin
            else:
                yolo_result={}
            # print(yolo_result)
        
def cv_callback(data):
    global if_recognition
    global yolo_result
    global face_rego

    # img_class = msg.bounding_boxes[0].Class
    # print(msg.bounding_boxes)
    # if if_recognition == 1:
    if 1:
        bridge = CvBridge()
        image=bridge.imgmsg_to_cv2(data, "bgr8")
        gray=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        faces=face_rego.detectMultiScale(gray,1.3,5)
        # print(faces,len(faces))
        if len(faces)==2:
        # if 1:
            yolo_result['person1']=faces[0][0]
            yolo_result['person2']=faces[1][0]
            (x,y,w,h)=faces[0]
            cv2.rectangle(image,(x,y),(x+w,y+h),(0,255,0),2)
            (x,y,w,h)=faces[1]
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
        for (x,y,w,h) in faces:
            cv2.rectangle(image,(x,y),(x+w,y+h),(255,0,0),2)
        
        cv2.imshow('face',image)


            
                    
                
                    


def navi_fail_callback(msg):
    if msg.status_list != []:
        if msg.status_list[0].status == 4:
            rospy.loginfo("Navigation Failed!")
            goto_point(target_recognition)
        

def goal_callback(msg):
    global if_recognition
    global yolo_result
    global pos_flag

    if msg.status.status == 3:
        if pos_flag==0:
            pos_flag=1
        # loc_pose = rospy.wait_for_message('pose',PoseWithCovarianceStamped)
        # if  loc_pose.pose.pose.position.y < 2 and loc_pose.pose.pose.position.x < 1:#需要调整
            os.system("play /home/ucar/FRRB-6/src/logic_moudle/wav/arrive.wav")
            rospy.loginfo('已到达')
            if_recognition=1
            # rospy.sleep(2)
            while if_recognition:
                # print(yolo_result)
                if 'person1' in yolo_result and 'person2' in yolo_result:
                    if_recognition=0
                else:
                    yolo_result={}
            # rospy.loginfo('请输入1或2:')
            while 1:
                # key=(cv2.waitKey(1) & 0xFF)
                # if key==ord('1') or key==ord('2'):
                #     break
                key=input('请输入1或2:')
                # print('key',key)
                if key==1 or key==2:
                    break
                
            
            if yolo_result['person1']>yolo_result['person2']:
                print('run')
                if key==1:
                    goto_point(target_left)
                else:
                    goto_point(target_right)
            else:
                if key==1:
                    goto_point(target_right)
                else:
                    goto_point(target_left)
            
            # if yolo_result['person1']>yolo_result['person2']:
            #     if key==ord('1'):
            #         goto_point(target_right)
            #     else:
            #         goto_point(target_left)
            # else:
            #     if key==ord('1'):
            #         goto_point(target_left)
            #     else:
            #         goto_point(target_right)
                    
        elif pos_flag==1:
            os.system("play /home/ucar/FRRB-6/src/logic_moudle/wav/complete.wav")
            rospy.loginfo('已完成抓捕')



            
    # rospy.loginfo("======================================")
    # rospy.loginfo(B_list)
    # rospy.loginfo("======================================")

    # os.system("play /home/ucar/ucar_ws/src/logic_moudle/wav/5.wav")

if __name__ == '__main__':
    rospy.init_node('play', anonymous=True)

    # 语音唤醒后，麦克风模块发布 start_others消息，msg = 1，此时进入mic_call_back函数。
    rospy.Subscriber("/start_others", Int8, mic_call_back)

    # 每次导航完成时，move_base节点会发布 move_base/result，消息类型是MoveBaseActionResult，此时进入goal_callback函数。
    rospy.Subscriber("move_base/result", MoveBaseActionResult, goal_callback)

    rospy.Subscriber("move_base/status", GoalStatusArray, navi_fail_callback)


    # 每次拍照后，yolo节点会识别图片，并发布"/darknet_ros/bounding_boxes"，此时进入classes_callback函数。
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes , classes_callback)

    # rospy.Subscriber('usb_cam/image_raw', Image, cv_callback)  

    # 发布/move_base_simple/goal，给小车指定下一个坐标点。
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

    # move_base的初始化
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    rospy.spin()

