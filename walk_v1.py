from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
from pinocchio import SE3, Model, Inertia, JointModelFreeFlyer, JointModelRX, JointModelRY, JointModelRZ, JointModelPX, JointModelPY, JointModelPZ, forwardKinematics, neutral
import gepetto.corbaserver
from display import Display
from inverse_kinematics_2 import InverseKinematics
import numpy as np
import time


class Piecelinear:
    
    def __init__(self,initialValue):
      
      self.value=[initialValue]
      self.durations=[0]
      self.times=[0]
      
    def addSegment(self,duration,value):
      self.value.append(value)
      self.times.append(self.times[-1]+duration)
      self.durations.append(duration)
      
       
    def eval(self,t):
       #savoir entre quels 2 valeurs on se trouve
       
       for i in range(len(self.times)-1) :
        if self.times[i]<=t  and  self.times[i+1]>t :
            a= (self.value[i+1]-self.value[i])/ (self.times[i+1]-self.times[i])
        
            b= self.value[i]
            u=t-self.times[i]
            v=a*u+ b
            
            return v
       
       raise    RuntimeError ("{} is bigger than {}".format(t,self.times[-1]))
       
       
       
       

    
