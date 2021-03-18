 #! /user/bin/python3


import rospy
import std_msgs
from  sensor_msgs.msg import PointCloud2 as pc2
from geometry_msgs.msg import PoseStamped as ps
import open3d as open3d
import math
import numpy as np
import csv
import vectors
from std_msgs.msg import MultiArrayLayout as mal
from std_msgs.msg import Float32MultiArray as fm32
from std_msgs.msg import MultiArrayDimension as mad
from vpython import *
from vpython.cyvector import vector as vpVec
from soft2util import numpy_msg as npMsg
import ros_numpy



"""
/orb_slam2_mono/pose
/orb_slam2_mono/map_points
/bebop/odeom
/bebop/states/ardrone3/CameraState/Orientation
"""

"""
listen to point cloud  ->points to list (pts,time)
listen to pose ->points to list (pts,time)
check if the lists are synchronised (same length etc.)


"""
class OrientClass():
    def __init__(self,data):
        
        self.q0 = data.pose.orientation.w
        self.q1 = data.pose.orientation.x
        self.q2 = data.pose.orientation.y
        self.q3 = data.pose.orientation.z
        self.x = 0
        self.y = 0
        self.z = 0
        self.vector = vector(0,0,0)
        self.Quad2Euler()
        self.EulerToPlane()

        print(self.roll,self.pitch,self.yaw)
    def Quad2Euler(self):
        self.roll = math.atan2 (2*(self.q0*self.q1 + self.q2*self.q3) , 1-2*(self.q1**2+self.q2**2))
        self.pitch = math.asin(2*(self.q0*self.q2-self.q3*self.q1))
        self.yaw = math.atan2(  2*(self.q0*self.q3 + self.q1*self.q2),1-2*(self.q2**2+self.q3**2))                  
    def EulerToPlane(self):

        k = vectors.Vector(math.cos(self.yaw)*math.cos(self.pitch),math.sin(self.pitch),math.sin(self.yaw)*math.cos(self.pitch))
        y = vectors.Vector(0,1,0)
        s = vectors.Vector.cross(k,y)
        v = vectors.Vector.cross(s,k)
        #vrot = v*math.cos(self.roll)  +vectors.Vector.cross(k,v)*math.sin(self.roll)

        self.x =  vpVec(k.x,k.y,k.z) 
        self.y  = vpVec(s.x,s.y,s.z)
        self.z =  vpVec(v.x,v.y,v.z)
    def GetQutroMsg(self):
        #TODO - create a transforamation matrix from the quads and traslation vector
        msg = fm32()

        msg.data = np.array([self.q0,self.q1,self.q2,self.q3],dtype=float)

        
        return msg
        
class PoseClass():
    def __init__(self,data):
        
        self.x = data.pose.orientation.w
        self.y = data.pose.orientation.x
        self.z = data.pose.orientation.y
    


class PtCloudXYZPacket():
    def __init__(self,data):
        self.pts = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(data)
        self.pts.dtype = float
        self.pts = np.unique(self.pts,axis = 0)
    def GetPtsMsg(self):

        self.ptMsg = np.reshape(self.pts,-1)
        self.ptMsg =np.array(self.ptMsg.tolist())

        msg = fm32()

        msg.data = self.ptMsg

        return msg
    
class RTVisual():
    def __init__(self):
        #self.drone = box(length = 2,width = 2, height = 0.2)
        scene.forward =vector(-1,-1,-1)
        scene.width  = 1200
        scene.height = 1200
        rate(20)
        self.xArrow = arrow(length = 6,shaftwidth = 0.01,color = color.red,axis = vector(1,0,0))
        self.yArrow = arrow(length = 6,shaftwidth = 0.01,color = color.green,axis = vector(0,1,0))
        self.zArrow = arrow(length = 6,shaftwidth = 0.01,color = color.blue,axis = vector(0,0,1))

        self.frontArrow = arrow(length = 4,shaftwidth = 0.1,color = color.magenta,axis = vector(1,0,0))
        self.sideArrow = arrow(length = 4,shaftwidth = 0.1,color = color.orange,axis = vector(0,0,1))
        self.upArrow = arrow(length = 4,shaftwidth = 0.1,color = color.blue,axis = vector(0,1,0))
    def UpdateOrient(self,orObj):
        self.frontArrow.axis =  orObj.x
        self.sideArrow.axis = orObj.y
        self.upArrow.axis =  orObj.z

    


class PtCollector(object):
    #static class to collect the points
    @classmethod
    def Init(cls):

        loc = open('loc.csv','w')
        loc.close()
        orient = open('orient.csv','w')
        orient.close()
        cls.pc2Objs = []
        cls.PoseObjs = []
        cls.orientObj = []
        cls.visual = RTVisual()
    @classmethod
    def addPc(cls,data):
        print ("here")
        cls.pc2Objs.append([PtCloudXYZPacket(data),data.header.stamp])
        
        #print (len(cls.pc2Objs),"pcl length")
    @classmethod
    def addPose(cls,data):
        
        cls.PoseObjs.append([PoseClass(data),data.pose.position,data.header.stamp])
        orObj = OrientClass(data)
        cls.orientObj.append([orObj,data.header.stamp])
        
        if len(cls.orientObj)>3:cls.orientObj.pop(0)
        if len(cls.PoseObjs)>3:cls.PoseObjs.pop(0)
        cls.visual.UpdateOrient(orObj)
        
       
        
        locString =   "%s,%s,%s\n"%(data.pose.position.x,data.pose.position.y,data.pose.position.z)
        orientString = "x:%s**y:%s**z:%s**w:%s\n"%(data.pose.orientation.x,data.pose.orientation.y,data.pose.orientation.z,data.pose.orientation.w)
        orientString2 = "x:%s**y:%s**z:%s\n"%(orObj.x,orObj.y,orObj.z)
        with open('loc.csv','a') as locFile:
            locFile.write(locString)
        with open('orient.csv','a') as orientFile:
            orientFile.write(orientString2)
    @classmethod
    def GetTrasformatioMatrix(cls):
        if not len(cls.PoseObjs)==len(cls.orientObj):return None
        pose = cls.PoseObjs[-1][0]
        orient = cls.orientObj[-1][0]
        time = cls.orientObj[-1][1]
        mat =np.eye(4)
        mat[:3,:3] = open3d.geometry.Geometry3D.get_rotation_matrix_from_quaternion(np.array([orient.q0,orient.q1,orient.q2,orient.q3]))
        
        mat[0,3] = pose.x  
        mat[1,3] = pose.y  
        mat[2,3] = pose.z  
        mat[3,3] = 1
        
        mat =  mat.reshape(-1)
        mat = mat.tolist()
        print (mat)
        #np.concatenate([mat,np.array(int(time.nsecs),dtype=float)])
        timeString = "%s.%s"%(time.secs,time.nsecs)

        mat.append(float(timeString))
        print (mat)
        
        mat = np.array(mat)
        msg = fm32()
        msg.data = mat
        return msg
    



def CallbackPts(data):
    rospy.loginfo("Found PT Message")
    PtCollector.addPc(data)
    ptData = PtCollector.pc2Objs[-1][0].GetPtsMsg()
    pub2.publish(ptData)
   
def CallbackPose(data):
    rospy.loginfo("Found Pose Message")
    PtCollector.addPose(data)
    mat = PtCollector.GetTrasformatioMatrix()   

    """    try:
        mat.any()
        print ("publish mat")
    except Exception as e:
        print (e)
        return"""
    if not mat ==None:
        pub1.publish(mat)   
    



def Point_Grabber(pub1):
    PtCollector.Init()
    rospy.init_node('soft2_Point_Grabber')
    sub1 = rospy.Subscriber('/orb_slam2_mono/map_points',pc2,CallbackPts, queue_size = 10)
    sub2 = rospy.Subscriber('/orb_slam2_mono/pose',ps,CallbackPose, queue_size = 10)
    rospy.spin()
    

if __name__ =='__main__':
    toRad = 2*np.pi/360
    toDeg = 1/toRad
    global pub1
    global pub2

    pub1 = rospy.Publisher('/soft2/transMatrix',fm32)
    pub2 = rospy.Publisher('/soft2/PartialPts',fm32)
    Point_Grabber(pub1)

