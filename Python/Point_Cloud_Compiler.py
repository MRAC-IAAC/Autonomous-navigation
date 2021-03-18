
import rospy
import std_msgs
import open3d
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
import os
import datetime as dt
import time
import csv
toRad = 2*np.pi/360
toDeg = 1/toRad



"""
0.open an empty point cloud
1. subscribe to followoing topics:

    Topic Name = '/soft2/transMatrix', dType =fm32
    Topic Name = '/soft2/PartialPts',  dType =fm32
2. transform the points according to tMat
3. register points to the point cloud
4. publish accumilating  point cloud?
"""

class CloudCompiler(object):
    def __init__(self,path = ""):
        self.compiledPcl = open3d.geometry.PointCloud()
        Now = time.time()
        Now = str(Now).replace(".","_")
        self.fileNameCsv = os.path.join(path,"soft2_compiled_%s.csv"%(Now))
        self.fileNamePcl = os.path.join(path,"soft2_compiled_%s.XYZ"%(Now))
        self.ptsQue = []
        self.matQue = []
        self.ptFlag = False
        self.matFlag = False
        open3d.io.write_point_cloud(self.fileNamePcl, self.compiledPcl)
        f = open(self.fileNameCsv,'w')
        f.close()
    def CollectPartialPts(self,data):
        ndArray = np.array(data.data)
        ndArray = ndArray.reshape(-1,3)
        print (ndArray.shape)
        self.ptsQue.append(ndArray)
        self.ptFlag = True
        #self.ptsQue.append([data.data,data.header.stamp])
    def CollectTMatrix(self,data):
        self.headerOrg = data.data[0]
        ndArray = np.array(data.data[:-1])
        ndArray = ndArray.reshape(4,4)
        print (ndArray.shape)
        self.matQue.append(ndArray)  
        self.matFlag = True
    def TransformPoints(self):
        if self.ptFlag and self.matFlag:
            xyz =  self.ptsQue.pop(0)
            locMat = self.matQue.pop(0)
         
            locPts = open3d.cpu.pybind.utility.Vector3dVector(xyz)
            locPts = open3d.geometry.PointCloud(locPts)
            locPts = locPts.transform(locMat)
            self.matFlag = False
            self.ptFlag = False
            
            
            np.savetxt(self.fileNameCsv, locPts.points, fmt='%1.3f')
            

            











def pointCloud_CallBack(data):
     ptManager.CollectPartialPts(data)
     ptManager.TransformPoints()
def TMat_Callback(data):
    ptManager.CollectTMatrix(data)
    ptManager.TransformPoints()



if __name__ == "__main__":
    ptManager  = CloudCompiler()
    rospy.init_node("soft2_point_compiler")
    sub1 = rospy.Subscriber( '/soft2/transMatrix',fm32,TMat_Callback)
    sub2 = rospy.Subscriber( '/soft2/PartialPts',fm32,pointCloud_CallBack)
    rospy.spin()

















