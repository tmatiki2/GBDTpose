#! /usr/bin/env python3.9

"""
 * File: offb_node.py
 * Stack and tested in Gazebo Classic 9 SITL
"""
from scipy.spatial.transform import Rotation as R
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
import math
from sensor_msgs.msg import Image
from rospy.numpy_msg import numpy_msg

agent_obj = ModelStates()
def vib_cb(msg):
    global agent_obj
    global agent_ind
    agent_obj = msg

roll_c_u, pitch_c_u, yaw_c_u = -(3.141592/2), 0, 3.141592/2 #r_xr, r_yr, r_zr#  
rx = np.matrix([[1, 0, 0],[0, math.cos(roll_c_u),-math.sin(roll_c_u)], [0, math.sin(roll_c_u), math.cos(roll_c_u)]])
ry = np.matrix([[math.cos(pitch_c_u), 0, math.sin(pitch_c_u)],[0, 1, 0],[-math.sin(pitch_c_u), 0, math.cos(pitch_c_u)]])
rz = np.matrix([[math.cos(yaw_c_u), -math.sin(yaw_c_u), 0],[math.sin(yaw_c_u), math.cos(yaw_c_u), 0],[0, 0, 1]])
global R_c_u
R_c_u = rz@ry@rx 

global N_ags_uav
N_ags_uav = []
n_agents = 6
for i in range(n_agents):
    i_agent = ModelState()
    i_agent.model_name = str(i)+"_uav_kinect"
    N_ags_uav.append(i_agent)

def plan_uav_cam(all_pubs, agent_p, n_c, ang_min, ang_max, tmin, tmax, uimg_virtual_dict, eps=1e-12):
    """Updates the fisheye camera location and orientation... Hence the captured images
       And publishes the captured images and corresponding poses for training GBDTpose"""
    
    angs = np.random.uniform(ang_min, ang_max, (n_c,3))
    rot2 = R.from_euler('xyz',angs,degrees=True)
    qt = rot2.as_quat()
    xyz = np.random.uniform(tmin, tmax, (n_c,3))

    ags_pubs = []
    for i in range(n_c):
        agent_odom = Odometry()
        ags = N_ags_uav[i]
        ags.pose.position.x = xyz[i,0]
        ags.pose.position.y = xyz[i,1]
        ags.pose.position.z = xyz[i,2]
        ags.pose.orientation.x = qt[i,0]
        ags.pose.orientation.y = qt[i,1]
        ags.pose.orientation.z = qt[i,2]
        ags.pose.orientation.w = qt[i,3]
        agent_p.publish(ags)

        agent_indx = agent_obj.name.index(str(i)+"_uav_kinect")
        curr_x = agent_obj.pose[agent_indx].position.x
        curr_y = agent_obj.pose[agent_indx].position.y
        curr_z = agent_obj.pose[agent_indx].position.z
        cqx = agent_obj.pose[agent_indx].orientation.x
        cqy = agent_obj.pose[agent_indx].orientation.y
        cqz = agent_obj.pose[agent_indx].orientation.z
        cqw = agent_obj.pose[agent_indx].orientation.w
        currp = np.array([curr_x, curr_y, curr_z, cqx, cqy, cqz, cqw])
        req_p = np.array([xyz[i,0], xyz[i,1], xyz[i,2], qt[i,0], qt[i,1], qt[i,2], qt[i,3]])
        diff = np.sum(np.abs(currp-req_p))
        while diff>eps:
            publishers(all_pubs)
            agent_p.publish(ags)
            curr_x = agent_obj.pose[agent_indx].position.x
            curr_y = agent_obj.pose[agent_indx].position.y
            curr_z = agent_obj.pose[agent_indx].position.z
            cqx = agent_obj.pose[agent_indx].orientation.x
            cqy = agent_obj.pose[agent_indx].orientation.y
            cqz = agent_obj.pose[agent_indx].orientation.z
            cqw = agent_obj.pose[agent_indx].orientation.w
            currp = np.array([curr_x, curr_y, curr_z, cqx, cqy, cqz, cqw])
            diff = np.sum(np.abs(currp-req_p))

        tm_now = rospy.Time.now()
        while rospy.Time.now()-tm_now <= rospy.Duration(3.0):
            publishers(all_pubs)
            uimg_ki_msg = uimg_virtual_dict['key_'+str(i)]
            agent_p.publish(ags)

        qv = qt[i, :]
        qvx, qvy, qvz, qvw = qv[0], qv[1], qv[2], qv[3]
        rot_gbt = R.from_quat([qvx, qvy, qvz, qvw])
        R_p_gbt = rot_gbt.as_matrix() # Rotation from camera body frame to world
        #R_c_u --- rotation from image frame to camera body frame
        R_agbt = (np.matmul(R_c_u.T, R_p_gbt.T))
        rot4_quat = R.from_matrix(R_agbt)
        rot_quat = rot4_quat.as_quat()
        agent_odom.pose.pose.position.x = xyz[i,0]
        agent_odom.pose.pose.position.y = xyz[i,1]
        agent_odom.pose.pose.position.z = xyz[i,2]
        agent_odom.pose.pose.orientation.x = rot_quat[0]
        agent_odom.pose.pose.orientation.y = rot_quat[1]
        agent_odom.pose.pose.orientation.z = rot_quat[2]
        agent_odom.pose.pose.orientation.w = rot_quat[3]
        
        uimg_ki_msg = uimg_virtual_dict['key_'+str(i)]
        if (uimg_ki_msg is not None):
            curr_time = rospy.Time.now()
            uimg_ki_msg.header.stamp = curr_time
            agent_odom.header.stamp = curr_time
            udetect_pub = rospy.Publisher("/uagent_img_"+str(i), Image, queue_size=10)
            agentPose_pub = rospy.Publisher("/uav_agent_pose_"+str(i), Odometry, queue_size=10)
            ags_pubs.append([udetect_pub, uimg_ki_msg])
            ags_pubs.append([agentPose_pub, agent_odom])
            
    return ags_pubs

u_agents = 6
u_img_init = None
global uimg_virtual_dict
uimg_virtual_dict = {f"key_{i}":u_img_init for i in range(u_agents)}
def uvimg_callback_0(msg):
    uimg_virtual_dict['key_0'] = msg        
def uvimg_callback_1(msg):
    uimg_virtual_dict['key_1'] = msg        
def uvimg_callback_2(msg):
    uimg_virtual_dict['key_2'] = msg       
def uvimg_callback_3(msg):
    uimg_virtual_dict['key_3'] = msg         
def uvimg_callback_4(msg):
    uimg_virtual_dict['key_4'] = msg       
def uvimg_callback_5(msg):
    uimg_virtual_dict['key_5'] = msg       
  
def publishers(pubs):
    for pub in pubs:
        pubb, topic = pub[0], pub[1]
        pubb.publish(topic)
    
if __name__ == "__main__":
    rospy.init_node("agent")
    agent_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, callback= vib_cb)
    agent1_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
    agent2_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10)
    agent_pub = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=10) 
    uimage_sub_0 = rospy.Subscriber("/0uav_camera"+"/color/image_raw", numpy_msg(Image), callback = uvimg_callback_0)
    uimage_sub_1 = rospy.Subscriber("/1uav_camera"+"/color/image_raw", numpy_msg(Image), callback = uvimg_callback_1)
    uimage_sub_2 = rospy.Subscriber("/2uav_camera"+"/color/image_raw", numpy_msg(Image), callback = uvimg_callback_2)
    uimage_sub_3 = rospy.Subscriber("/3uav_camera"+"/color/image_raw", numpy_msg(Image), callback = uvimg_callback_3)
    uimage_sub_4 = rospy.Subscriber("/4uav_camera"+"/color/image_raw", numpy_msg(Image), callback = uvimg_callback_4)
    uimage_sub_5 = rospy.Subscriber("/5uav_camera"+"/color/image_raw", numpy_msg(Image), callback = uvimg_callback_5)

    rate = rospy.Rate(3)
    all_pubs = []
    n_uav_ags = 6
    
    #TODO
    ang_min, ang_max = -30, 30 # Adjust this parameters for the orientation range of the cameras according to size of your GBDT
    tmin, tmax = -5.0, 5.0 # Also adjust according to size of your GBDT
    uags_pubs = plan_uav_cam(all_pubs, agent_pub, n_uav_ags, ang_min, ang_max, tmin, tmax, uimg_virtual_dict)
    all_pubs.extend(uags_pubs)

    while(not rospy.is_shutdown()):
        uags_pubs, xyz_now = plan_uav_cam(all_pubs, agent_pub, n_uav_ags, ang_min, ang_max, tmin, tmax, uimg_virtual_dict)
        all_pubs = []
        all_pubs.extend(uags_pubs)
        rospy.sleep(1)
