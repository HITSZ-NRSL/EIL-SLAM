#!/usr/bin/env python3

from __future__ import print_function
import cv2
from os.path import join
import numpy as np
import utils
import calc2
from time import time
import tensorflow as tf
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge,CvBridgeError

vh = calc2.vh
vw = calc2.vw
K = 5 
N = 50
C = 2
W = 9
receiveFlag = True
calc = utils.CALC2('model', tf.Session(), ret_c5=True)
curIndex = 0
db_count = 0
loops = []
db = []
dbkp = []
avg_rate = 0
loop_count = 0
last_loop_id = -1
skipped = False
j = 0
pts = []
ims = []
bridge = CvBridge()
loopClosureTPFlag = False 

def close_loop(db, dbkp, descr, kp):
    matcher = cv2.BFMatcher(cv2.NORM_L2)
    kp, kp_d = kp
    db = np.concatenate(tuple(db), axis=0)
    sim = np.sum(descr * db, axis=-1) 
    
    top_k_sim_ind = np.argpartition(sim, -K)[-K:] 

    max_sim = -1.0
    i_max_sim = -1
    best_match_tuple = None
    # print(top_k_sim_ind)
    for k in top_k_sim_ind:
        if k<len(dbkp) and k >=0:
            db_kp, db_kp_d = dbkp[k]
            matches = matcher.knnMatch(kp_d, db_kp_d, 2)
            good = []
            pts1 = []
            pts2 = []
            for m,n in matches:
                if m.distance < 0.7*n.distance:
                    good.append(m)
                    pts1.append(db_kp[m.trainIdx].pt)
                    pts2.append(kp[m.queryIdx].pt)
            if len(good) > 7:
                pts1 = np.int32(pts1)
                pts2 = np.int32(pts2)
                curr_sim = sim[k]
                if curr_sim > max_sim:
                    max_sim = curr_sim
                    i_max_sim = k
                    best_match_tuple = (kp, db_kp, good, pts1, pts2)
        else:
            print(k)
    
    if i_max_sim > -1:
        F, mask = cv2.findFundamentalMat(best_match_tuple[3],
                     best_match_tuple[4], cv2.FM_RANSAC)
        if F is None:
            max_sim=-1.0
            i_max_sim = -1
    return i_max_sim,top_k_sim_ind


def infrared_loop(image,header):

    im = image.copy()
    im_cp = np.copy(im)
    ims.append(im_cp)
    im = cv2.cvtColor(im, cv2.COLOR_BGR2RGB)

    im = cv2.resize(im,(vw,vh))
    t0 = time()

    descr, c5 = calc.run(im)
    kp, kp_d = utils.kp_descriptor(c5)
    dbkp.append((kp, kp_d))
    db.append(descr)
    global curIndex,loop_count,skipped,avg_rate,db_count,pub,pubDbInd
    db_count += 1

    InfraredInd = PointStamped()
    InfraredInd.header = header
    InfraredInd.point.x = db_count
    pubDbInd.publish(InfraredInd)

    if db_count > 2*N:
        t1 = time()
        j,topKInd = close_loop(db[:-N], dbkp, descr, (kp, kp_d))
        t = (time() - t1) * 1000
        if j > 0:
            loopValid = PoseStamped()
            loopValid.header = header
            loopValid.pose.position.x = j 
            loopValid.pose.position.y = topKInd[0]
            loopValid.pose.position.z = topKInd[1]
            loopValid.pose.orientation.x = topKInd[2]
            loopValid.pose.orientation.y = topKInd[3]
            loopValid.pose.orientation.z = topKInd[4]
            loopValid.pose.orientation.w = db_count
            pub.publish(loopValid)

def callback(data):
    global curIndex
    curIndex += 1
    cv_img = bridge.imgmsg_to_cv2(data,"bgr8")
    if curIndex%10==0:
        curHeader = data.header 
        infrared_loop(cv_img,curHeader)
    else:
        pass

def infrared_received():

    global pub,pubDbInd
    rospy.init_node('infraredLoop',anonymous=True) 

    rospy.Subscriber('/camera/image_raw',Image,callback)
    pub = rospy.Publisher('/InfraredLoop',PoseStamped,queue_size=1)
    pubDbInd = rospy.Publisher('/InfraredDBInd',PointStamped,queue_size=1)
    rospy.spin()

if __name__ == '__main__':

    infrared_received()
