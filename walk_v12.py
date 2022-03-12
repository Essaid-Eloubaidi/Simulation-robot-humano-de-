from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
from pinocchio import SE3, Model, Inertia, JointModelFreeFlyer, JointModelRX, JointModelRY, JointModelRZ, JointModelPX, JointModelPY, JointModelPZ, forwardKinematics, neutral
import gepetto.corbaserver
from display import Display
from inverse_kinematics_2 import InverseKinematics
import numpy as np
import time
from walk_v1 import Piecelinear as PL
from legged_robot import Robot

z_waist = 3.6

leftFootRefPose=PL(np.array([0, -1, 0.4]))
rightFootRefPose=PL(np.array([0, 1, 0.4]))
waistRefPose =PL(np.array([0,0,z_waist]))

waist=[np.array([0,0,z_waist]),np.array([0,-2,z_waist]), np.array([1,-2,z_waist]),np.array([1,-2,z_waist]),np.array([1,2,z_waist]),np.array([2,2,z_waist]),np.array([2,2,z_waist]), np.array([2,-2,z_waist]), np.array([3,-2,z_waist]), np.array([3,-2,z_waist]), np.array([3,2,z_waist]), np.array([3,2,z_waist]), np.array([3,2,z_waist]), np.array([3,0,z_waist])]

RF=[np.array([0, 1, 0.4]),np.array([0, 1, 0.4]), np.array([1,1,1]),np.array([1,1,0.4]),np.array([1,1,0.4]),np.array([1,1,0.4]),np.array([1,1,0.4]),np.array([1,1,0.4]),np.array([3,1,1]) ,np.array([3,1,0.4]), np.array([3,1,0.4]), np.array([3,1,0.4]), np.array([3,1,0.4]), np.array([3,1,0.4])]
LF=[np.array([0, -1, 0.4]),np.array([0, -1, 0.4]),np.array([0,-1,0.4]), np.array([0,-1,0.4]), np.array([0,-1,0.4]),np.array([2,-1,1]), np.array([2,-1,0.4]), np.array([2,-1,0.4]), np.array([2,-1,0.4]), np.array([2,-1,0.4]), np.array([2,-1,0.4]), np.array([3,-1,1]),np.array([3,-1,0.4]),np.array([3,-1,0.4]) ]
deltaT = 0.3
T=[deltaT ,deltaT,deltaT,deltaT,deltaT,deltaT,deltaT,deltaT,deltaT,deltaT,deltaT,deltaT,deltaT,deltaT]

for ti, w, lf, rf in zip(T, waist, LF, RF):
    leftFootRefPose.addSegment(ti,lf)
    rightFootRefPose.addSegment(ti,rf)
    waistRefPose.addSegment(ti, w)


#EVAL
robot=Robot()
ik=InverseKinematics(robot)

t=0
q=1e-2*np.ones(robot.model.nq)
while t<deltaT*len(T):
    ik.leftFootRefPose.translation= leftFootRefPose.eval(t)
    ik.rightFootRefPose.translation=rightFootRefPose.eval(t)
    ik.waistRefPose.translation= waistRefPose.eval(t)
    t=t+1E-2
    q=ik.solve(q)
    robot.display(q)
    
     


    
    
      
      

