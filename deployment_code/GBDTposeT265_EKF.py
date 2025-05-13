#!/usr/bin/env python3.8

import os
import cv2 
import numpy as np
import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("Qt5Agg")

def ekf_function(measure, xhat, Qv, Rv, rgbdt, P, dt=0.033):
    delta_theta, delta_x, delta_q, delta_v, xyz, v_xyz, q_wxyz = measure
    quat = np.array([[xhat[6,0], xhat[7,0], xhat[8,0], xhat[9,0]]]).T
    q1 = quat[0,0]
    q2 = quat[1,0]
    q3 = quat[2,0]
    q4 = quat[3,0]
    R_bi = np.array([[pow(q1,2)+pow(q2,2)-pow(q3,2)-pow(q4,2), 2*(q2*q3-q1*q4), 2*(q2*q4+q1*q3)],
                        [2*(q2*q3+q1*q4), pow(q1,2)-pow(q2,2)+pow(q3,2)-pow(q4,2), 2*(q3*q4-q1*q2)],
                        [2*(q2*q4-q1*q3), 2*(q3*q4+q1*q2), pow(q1,2)-pow(q2,2)-pow(q3,2)+pow(q4,2)]])
    
    ## Forward propapgation step with T265 odometry data
    p = delta_theta[0]/dt
    q = delta_theta[1]/dt
    r = delta_theta[2]/dt
    delta_vb = (R_bi.T@delta_v.T).flatten()
    fx = delta_vb[0]/dt
    fy = (delta_vb[1]/dt)
    fz = (delta_vb[2]/dt)-9.81
    Omega = np.array([[0, p, q, r], [-p, 0, -r, q],[-q, r, 0, -p],[-r, -q, p, 0]])
    xhat[0,0] = xhat[0,0] + delta_x[0]
    xhat[1,0] = xhat[1,0] + delta_x[1]
    xhat[2,0] = xhat[2,0] + delta_x[2]
    xhat[3,0] = xhat[3,0] + delta_v[0]
    xhat[4,0] = xhat[4,0] + delta_v[1]
    xhat[5,0] = xhat[5,0] + delta_v[2]
    xhat[6,0] = xhat[6,0] + delta_q[-1]
    xhat[7,0] = xhat[7,0] + delta_q[0]
    xhat[8,0] = xhat[8,0] + delta_q[1]
    xhat[9,0] = xhat[9,0] + delta_q[2]
  
    quat = np.array([[xhat[6,0], xhat[7,0], xhat[8,0], xhat[9,0]]]).T
    quat = quat/np.linalg.norm(quat)
    xhat[6,0] = quat[0,0]
    xhat[7,0] = quat[1,0]
    xhat[8,0] = quat[2,0]
    xhat[9,0] = quat[3,0]
    
    ## computation of the Jacobians for EKF
    Fvq = np.array([[2*(xhat[6,0]*fx - xhat[9,0]*fy + xhat[8,0]*fz), 
    2*(xhat[7,0]*fx + xhat[8,0]*fy + xhat[9,0]*fz), 
    2*(-xhat[8,0]*fx + xhat[7,0]*fy + xhat[6,0]*fz),
    2*(-xhat[9,0]*fx - xhat[6,0]*fy + xhat[7,0]*fz)],
    [2*(xhat[9,0]*fx + xhat[6,0]*fy - xhat[7,0]*fz), 
    2*(xhat[8,0]*fx - xhat[7,0]*fy - xhat[6,0]*fz),
    2*(xhat[7,0]*fx + xhat[8,0]*fy + xhat[9,0]*fz),
    2*(xhat[6,0]*fx - xhat[9,0]*fy + xhat[8,0]*fz)],
    [2*(-xhat[8,0]*fx + xhat[7,0]*fy + xhat[6,0]*fz),
    2*(xhat[9,0]*fx + xhat[6,0]*fy - xhat[7,0]*fz),
    2*(-xhat[6,0]*fx + xhat[9,0]*fy - xhat[8,0]*fz),
    2*(xhat[7,0]*fx + xhat[8,0]*fy + xhat[9,0]*fz)]])

    Fvb = -R_bi 
    Fqq = -0.5*Omega 

    Fqb = 0.5*np.array([[xhat[7,0], xhat[8,0], xhat[9,0]],
    [-xhat[6,0], xhat[9,0], -xhat[8,0]], 
    [-xhat[9,0], -xhat[6,0], xhat[7,0]],
    [xhat[8,0], -xhat[7,0], -xhat[6,0]]])
    # Linearized Transition matrix A
    Z = np.zeros((3,3))
    I = np.eye(3,3)
    I44 = np.eye(4,4)
    Z34 = np.zeros((3,4))
    Z43 = np.zeros((4,3))
    Z36 = np.zeros((3,6)) 
    
    A = np.block([[Z, I, Z34, Z, Z],
    [Z, Z, Fvq, Z, Fvb], 
    [Z43, Z43, Fqq, Fqb, Z43],
    [Z, Z, Z34, Z, Z],
    [Z, Z, Z34, Z, Z]]) # ..

    Pdot = A@P + P@A.transpose() + Qv 
    P = P + Pdot*dt
    
    ## Correction step with GBDTpose data
    z = np.array([[xyz[0], xyz[1], xyz[2], v_xyz[0], v_xyz[1], v_xyz[2], q_wxyz[0], q_wxyz[1], q_wxyz[2], q_wxyz[3]]]).T
    
    ## Linearized observation matrix H
    Hvq = np.array([[rgbdt[0]*2*xhat[8,0]*q + rgbdt[0]*2*xhat[9,0]*r, 
    rgbdt[0]*2*xhat[9,0]*q - rgbdt[0]*2*xhat[8,0]*r,
    rgbdt[0]*2*xhat[6,0]*q - rgbdt[0]*2*xhat[7,0]*r,
    rgbdt[0]*2*xhat[7,0]*q + rgbdt[0]*2*xhat[6,0]*r],
    [-rgbdt[0]*2*xhat[7,0]*q - rgbdt[0]*2*xhat[6,0]*r,
    rgbdt[0]*2*xhat[7,0]*r - rgbdt[0]*2*xhat[6,0]*q,
    rgbdt[0]*2*xhat[9,0]*q - rgbdt[0]*2*xhat[8,0]*r,
    rgbdt[0]*2*xhat[8,0]*q + rgbdt[0]*2*xhat[9,0]*r],
    [rgbdt[0]*2*xhat[6,0]*q - rgbdt[0]*2*xhat[7,0]*r,
    -rgbdt[0]*2*xhat[7,0]*q - rgbdt[0]*2*xhat[6,0]*r,
    -rgbdt[0]*2*xhat[8,0]*q - rgbdt[0]*2*xhat[9,0]*r,
    rgbdt[0]*2*xhat[9,0]*q - rgbdt[0]*2*xhat[8,0]*r]]) 
    

    Hxq = np.array([[-rgbdt[0]*2*xhat[6,0], -rgbdt[0]*2*xhat[7,0], rgbdt[0]*2*xhat[8,0], rgbdt[0]*2*xhat[9,0]],
    [-rgbdt[0]*2*xhat[9,0], -rgbdt[0]*2*xhat[8,0], -rgbdt[0]*2*xhat[7,0], -rgbdt[0]*2*xhat[6,0]],
    [rgbdt[0]*2*xhat[8,0], -rgbdt[0]*2*xhat[9,0], rgbdt[0]*2*xhat[6,0], -rgbdt[0]*2*xhat[7,0]]]) # ..
    H = np.block([[I, Z, Hxq, Z36],[Z, I, Hvq, Z36],[Z43, Z43, I44, Z43, Z43]]) # .. 
    L = P@H.T@np.linalg.inv(H@P@H.T + Rv) # Kalman gain
    xhat = xhat + L@(z- H@xhat) # Error correction
    P = (np.eye(16,16)-L@H)@P #Update covariance matrix
    return xhat, P

t265 = None
def t265_cb(msg):
    global t265
    t265 = msg

t265_accel = None
def accel_cb(msg):
    global t265_accel
    t265_accel = msg

t265_gyro = None
def gyro_cb(msg):
    global t265_gyro
    t265_gyro = msg

gbdtpose = None
def gbdtpose_cb(msg):
    global gbdtpose
    gbdtpose = msg

def extract_odom(odom_msg):
    x = odom_msg.pose.pose.position.x
    y = odom_msg.pose.pose.position.y
    z = odom_msg.pose.pose.position.z
    q1 = odom_msg.pose.pose.orientation.x
    q2 = odom_msg.pose.pose.orientation.y
    q3 = odom_msg.pose.pose.orientation.z
    q4 = odom_msg.pose.pose.orientation.w
    vx = odom_msg.twist.twist.linear.x
    vy = odom_msg.twist.twist.linear.y
    vz = odom_msg.twist.twist.linear.z
    avx = odom_msg.twist.twist.angular.x
    avy = odom_msg.twist.twist.angular.y
    avz = odom_msg.twist.twist.angular.z
    return [x,y,z],[q1,q2,q3,q4],[vx,vy,vz],[avx,avy,avz]


def comp_delta(p_t, p_0):
    return np.array(p_t) - np.array(p_0)

def plot_pose(gb_p, t2_p, ek_p):
    plt.figure(figsize=(18,5))
    plt.suptitle("Position estimates")
    for i in range(3):
        plt.subplot(1,3,i+1)
        plt.plot(gb_p[0][:,i], '--*')
        plt.plot(t2_p[0][:,i], '--*')
        plt.plot(ek_p[0][:,i], '--*')
        plt.legend(["GBDTpose", "T265-VIO", "EKF"])

    plt.figure(figsize=(8,8))
    plt.suptitle("Plan View")
    plt.plot(gb_p[0][:,0], gb_p[0][:,1], '--*')
    plt.plot(t2_p[0][:,0], t2_p[0][:,1], '--*')
    plt.plot(ek_p[0][:,0], ek_p[0][:,1], '--*')
    plt.legend(["GBDTpose", "T265-VIO", "EKF"])
    
    plt.figure(figsize=(18,5))
    plt.suptitle("Orientation estimates")
    for i in range(3):
        plt.subplot(1,3,i+1)
        plt.plot(gb_p[1][:,i], '--*')
        plt.plot(t2_p[1][:,i], '--*')
        plt.plot(ek_p[1][:,i], '--*')
        plt.legend(["GBDTpose", "T265-VIO", "EKF"])
    plt.ion()
    plt.show()
    plt.pause(0.01)
    plt.close()
    

if __name__ == '__main__':
    rospy.init_node("ekf_node")
    sub_t265 = rospy.Subscriber("/camera/odom/sample", Odometry, callback = t265_cb) #T265 odometry topic
    sub_accel = rospy.Subscriber("/camera/accel/sample", Imu, callback = accel_cb) #T265 raw acceleration measurements
    sub_gyro = rospy.Subscriber("/camera/gyro/sample", Imu, callback = gyro_cb) #T265 raw qyroscope measurements
    sub_gbdtpose = rospy.Subscriber("/gbdtpose", Odometry, callback = gbdtpose_cb) #GBDTpose measurements (XYZ, RPY, Covariance)
    real_pub = rospy.Publisher("/mavros/vision_pose/pose_cov", PoseWithCovarianceStamped, queue_size=10) # Publish poses to Mavros for UAV localization

    rate = rospy.Rate(30)
    while (t265 is None) or (gbdtpose is None) or (t265_accel is None) or (t265_gyro is None):
        pass

    #TODO
    Qv = 0.1*np.diag(np.ones(16)) # Process covarinace matrix -- you can update this
    Rv = np.diag([0.2, 0.3, 0.1, 0.02, 0.02, 0.02, 0.1, 0.05, 0.15, 0.06]) # Measurement Covarinace matrix -- you can update this
    rgbdt = 0.05*np.ones(3) #offset from camera body frame to flight controller -- you can update this
    P = np.diag([0.03, 0.03, 0.02, 0.03, 0.03, 0.03, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01]) # -- you can update this

    gbdt_odom = extract_odom(gbdtpose)
    T0 = np.array(gbdt_odom[0]) ## Initial location of UAV is obtained from GBDTpose
    R_e_cpt = R.from_quat(gbdt_odom[1]).as_matrix()
    R_cpt_c = np.array([[-1,0,0.0],[0,-1,0],[0,0,1]])
    R_e_c = R_cpt_c@R_e_cpt ## Rotation to align T265 camera frame to GBDTpose frame i.e., G

    t265_odom = extract_odom(t265)
    t265_odom_0 = t265_odom
    R_c_cppt = R.from_quat(t265_odom[1]).as_matrix()
    R_e_cppt = R_c_cppt.T@R_e_c

    
    xyz_t265_0 = ((R_e_cppt.T)@np.array(t265_odom[0]).T + T0).flatten()
    q_t265_0 = R.from_matrix(R_e_cppt.T).as_quat()
    eul_t265_0 = R.from_quat(q_t265_0).as_euler('xyz')
    v_xyz_t265_0 = ((R_e_cppt.T)@np.array(t265_odom[2]).T).flatten()
    vx,vy,vz = tuple(v_xyz_t265_0.tolist())
    
    delta_x = comp_delta(xyz_t265_0, xyz_t265_0)
    delta_q = comp_delta(q_t265_0, q_t265_0)
    delta_theta = comp_delta(eul_t265_0, eul_t265_0)
    delta_v = comp_delta(v_xyz_t265_0, v_xyz_t265_0)
    
    q1, q2, q3, q0 = tuple(q_t265_0)
    bp, bq, br = tuple(np.random.uniform(0,0.000000005,3).tolist()) #np.random.uniform(0,0.05,3)
    bfx, bfy, bfz = tuple(np.random.uniform(0,0.000000005,3).tolist())
    x,y,z = tuple(gbdt_odom[0])
    xhat = np.array([[x, y, z, vx, vy, vz, q0, q1, q2, q3, bp, bq, br, bfx, bfy, bfz]]).T
    
    real_pose = PoseWithCovarianceStamped()
    t0 = rospy.Time.now()
    t1 = rospy.Time.now()
    viz_traj = False
    viz_after = 5 ## visualize the trajectory after 5 seconds --  change as needed

    xyz_g, xyz_t, xyz_e = [], [], []
    e_g, e_t, e_e = [], [], []
    while not(rospy.is_shutdown()):
        real_pose.header.stamp = rospy.Time.now()
        gbdt_odom = extract_odom(gbdtpose)
        t265_odom = extract_odom(t265)
        
        R_c_cppt = (R.from_quat(t265_odom[1]).as_matrix()).T
        R_e_cppt = R_c_cppt@R_e_c
        vx,vy,vz = tuple(t265_odom[2])
        xyz_t265 = ((R_e_cppt.T)@np.array(t265_odom[0]).T + T0).flatten()
        q_t265 = R.from_matrix(R_e_cppt.T).as_quat()
        v_xyz_t265 = ((R_e_cppt.T)@np.array(t265_odom[2]).T).flatten()
        
        delta_x = comp_delta(xyz_t265, xyz_t265_0)
        delta_q = comp_delta(q_t265, q_t265_0)
        delta_theta = comp_delta(R.from_quat(q_t265).as_euler('xyz'), eul_t265_0)
        delta_v = comp_delta(v_xyz_t265, v_xyz_t265_0)

        xyz = gbdt_odom[0]
        v_xyz = v_xyz_t265.tolist()
        R_e_cpppt = R.from_quat(gbdt_odom[1]).as_matrix()

        R_e_cppt_gbdt = np.array(R_cpt_c@R_e_cpppt)
        q_gbdt = R.from_matrix(R_e_cppt_gbdt.T).as_quat()
        q_wxyz = [q_gbdt[-1], q_gbdt[0], q_gbdt[1], q_gbdt[2]]
        measure = [delta_theta, delta_x, delta_q, delta_v, xyz, v_xyz, q_wxyz]
        xhat, P = ekf_function(measure, xhat, Qv, Rv, rgbdt, P)

        if rospy.Time.now() - t0 >= rospy.Duration(2):
            xyz_t265_0 =  np.array(xyz).flatten() ## reinitialize the coordinate frame of T265 to correct its drift
            t0 = rospy.Time.now()
        else:
            xyz_t265_0 = xyz_t265

        q_t265_0 = q_t265
        eul_t265_0 = R.from_quat(q_t265).as_euler('xyz')
        v_xyz_t265_0 = v_xyz_t265
        eul_gbdt = R.from_quat(q_gbdt).as_euler('xyz', degrees=True)
        q_e = xhat[6:10,0]
        eul_ekf = R.from_quat([q_e[1],q_e[2],q_e[3],q_e[0]]).as_euler('xyz', degrees=True)
        eul_t265 = R.from_quat(q_t265).as_euler('xyz', degrees=True)

        qekf = [q_e[1],q_e[2],q_e[3],q_e[0]]
        real_pose.pose.pose.position.x = xyz[0]
        real_pose.pose.pose.position.y =  xyz[0]
        real_pose.pose.pose.position.z =  xyz[0]
        real_pose.pose.pose.orientation.x = qekf[0]
        real_pose.pose.pose.orientation.y = qekf[1]
        real_pose.pose.pose.orientation.z = qekf[2]
        real_pose.pose.pose.orientation.w = qekf[3]

        cov_pos = np.zeros(36)
        cov_pos[0:9] = P[0:3, 0:3].flatten()
        real_pose.pose.covariance = cov_pos.tolist()
        real_pub.publish(real_pose)
        
        if viz_traj:
            xyz_g.extend(xyz), xyz_t.extend(xyz_t265.tolist()), xyz_e.extend(xhat[0:3,0].tolist())
            e_g.extend(list(eul_gbdt)), e_t.extend(list(eul_t265)), e_e.extend(list(eul_ekf))
            gb_p = [np.array(xyz_g).reshape(-1,3), np.array(e_g).reshape(-1,3)]
            t2_p = [np.array(xyz_t).reshape(-1,3), np.array(e_t).reshape(-1,3)] 
            ek_p = [np.array(xyz_e).reshape(-1,3), np.array(e_e).reshape(-1,3)]
            if rospy.Time.now()-t1 >= rospy.Duration(75):
                plot_pose(gb_p, t2_p, ek_p)
                t1 = rospy.Time.now()
        rate.sleep()

