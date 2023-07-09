#! /usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import numpy as np
import tf
import cv2
import math
from geometry_msgs.msg import PoseStamped,Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
current_state = State()



def state_cb(msg):
    global current_state
    current_state = msg

class image_subscribers:
    def __init__(self):
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image,self.depth_callback)

    def depth_callback(self,msg):
        
        try:
            global img_np
            img_n = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width, 1)
            img_np = cv2.normalize(img_n, None, 1, 0, cv2.NORM_MINMAX)
            
            
        except Exception as e:
            rospy.logerr(e)
            return
        
        cv2.imshow('Depth Image', img_np)
        cv2.waitKey(1)


        
        sensors ={
            'LEFT' : min(min(img_n[240][0:209]),10),
            'FRONT' : min(min(img_n[240][210:429]),10),
            'RIGHT' : min(min(img_n[240][430:639]),10),
        }
        
        if sensors['FRONT'] > 3 and sensors['LEFT'] > 1 and sensors['RIGHT'] > 1 :
            move_by(0.01,0,0,0)
            
        elif sensors['FRONT'] < 3 and sensors['LEFT'] > 1 and sensors['RIGHT'] > 1:
            move_by(0,0,0,math.pi*0.05/180)
            
        elif sensors['FRONT'] > 3 and sensors['LEFT'] > 1 and sensors['RIGHT'] < 1:
            move_by(0,0,0,math.pi*0.05/180)
        
        elif sensors['FRONT'] > 3 and sensors['LEFT'] < 1 and sensors['RIGHT'] > 1:
            move_by(0,0,0,-math.pi*0.05/180)
            
        elif sensors['FRONT'] < 3 and sensors['LEFT'] > 1 and sensors['RIGHT'] < 1:
            move_by(0,0,0,math.pi*0.05/180)
            
        elif sensors['FRONT'] < 3 and sensors['LEFT'] < 1 and sensors['RIGHT'] > 1:
            move_by(0,0,0,-math.pi*0.05/180)
            
        elif sensors['FRONT'] < 3 and sensors['LEFT'] < 1 and sensors['RIGHT'] < 1:
            move_by(0,0,0,0)
            
        elif sensors['FRONT'] > 3 and sensors['LEFT'] < 1 and sensors['RIGHT'] < 1:
            move_by(0.01,0,0,0)
            
        else:
            rospy.loginfo("nothing")
        
       
        
            
        
    
     

def move_by(x,y,z,yaw):
    global v
    
    pose.pose.position.x = pose.pose.position.x + x
    pose.pose.position.y = pose.pose.position.y + y
    pose.pose.position.z = pose.pose.position.z + z
    #q = tf.transformations.quaternion_from_euler(0, 0, np.deg2rad(yaw))
    #pose.pose.orientation.x = pose.pose.orientation.x + q[0]
    #pose.pose.orientation.y = pose.pose.orientation.y + q[1]
    v = v + yaw

    pose.pose.orientation = Quaternion(*tf.transformations.quaternion_from_euler(0, 0, v))
    
    
    return

if __name__ == "__main__":
    rospy.init_node("image_subscriber")


    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=1)

    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)


    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    
    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()
    
    global pose
    pose = PoseStamped()
    pose.header.frame_id = "base_link"
    v = 0
    
    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0
    pose.pose.orientation.w = 0

    # Send a few setpoints before starting
    for i in range(100):
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        # First set the mode to offboard (refer to PX4 Flight Modes)
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")

            last_req = rospy.Time.now()

        # Armed the vehicle
        elif(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(arming_client.call(arm_cmd).success == True):
                rospy.loginfo("Vehicle armed")

            last_req = rospy.Time.now()

        # Move the vehicle
        elif(current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            img_subs = image_subscribers()
            
            last_req = rospy.Time.now()
        
           

            

        local_pos_pub.publish(pose)

        rate.sleep()
