import numpy as np
import numpy.linalg
from scipy.optimize import fmin_bfgs
from pinocchio import forwardKinematics, log, neutral
import eigenpy
eigenpy.switchToNumpyMatrix()
import legged_robot



class CallbackLogger:
     def __init__(self, ik):
          self.nfeval = 1
          self.ik = ik
     def __call__(self,x):
          print ('===CBK=== {0:4d}   {1}'.format(self.nfeval,
                                                self.ik.latestCost))
          self.nfeval += 1


class InverseKinematics(object):
    leftFootJoint = 'left_leg_6_joint'
    rightFootJoint = 'right_leg_6_joint'
    waistJoint = 'waist_joint'

    def __init__ (self, robot):
        self.q = neutral (robot.model)
        forwardKinematics (robot.model, robot.data, self.q)
        self.robot = robot
        # Initialize references of feet and center of mass with initial values
        self.leftFootRefPose = robot.data.oMi [robot.leftFootJointId].copy ()
        self.rightFootRefPose = robot.data.oMi [robot.rightFootJointId].copy ()
        self.waistRefPose = robot.data.oMi [robot.waistJointId].copy ()
        
        self.P1= np.array([-1, 0, 0])
        self.P2= np.array([0, 1, 0])

        
    def cost (self, q):
        forwardKinematics (robot.model, robot.data, q)
        
        leftFootActualPose = robot.data.oMi [robot.leftFootJointId]
        rightFootActualPose = robot.data.oMi [robot.rightFootJointId]
        waistActualPose = robot.data.oMi [robot.waistJointId]
        
        leftFootErr = abs(leftFootActualPose.translation - self.leftFootRefPose.translation)
        rightFootErr = abs(rightFootActualPose.translation - self.rightFootRefPose.translation)
        waistErr = abs(waistActualPose.translation - self.waistRefPose.translation)
        
        leftFootRefPoseP1 = self.leftFootRefPose.act(self.P1)
        leftFootRefPoseP2 = self.leftFootRefPose.act(self.P2)
        leftFootActualPoseP1 = leftFootActualPose.act(self.P1)
        leftFootActualPoseP2 = leftFootActualPose.act(self.P2)
        
        rightFootRefPoseP1 = self.rightFootRefPose.act(self.P1)
        rightFootRefPoseP2 = self.rightFootRefPose.act(self.P2)
        rightFootActualPoseP1 = rightFootActualPose.act(self.P1)
        rightFootActualPoseP2 = rightFootActualPose.act(self.P2)

        
        leftFootErrP1 = np.linalg.norm(abs(leftFootActualPoseP1 - leftFootRefPoseP1))
        leftFootErrP2 = np.linalg.norm(abs(leftFootActualPoseP2 - leftFootRefPoseP2))
        rightFootErrP1 = np.linalg.norm(abs(rightFootActualPoseP1 - rightFootRefPoseP1))
        rightFootErrP2 = np.linalg.norm(abs(rightFootActualPoseP2 - rightFootRefPoseP2))

        
        
        
        normErr = np.linalg.norm(leftFootErr)**2 + np.linalg.norm(rightFootErr)**2 + np.linalg.norm(waistErr)**2 + leftFootErrP1**2+ leftFootErrP2**2 + rightFootErrP1**2 +rightFootErrP2**2
        
        return (normErr)
        
        
    def solve (self, q0):
        return(fmin_bfgs(self.cost, q0))
       

robot = legged_robot.Robot()
robot.viewer.viewer.gui.setVisibility('world/floor','OFF')
ik=InverseKinematics(robot)
# #ICI On change la position initiale
#ik.waistRefPose.translation[2]=15
#ik.leftFootRefPose.translation[0]=5
#ik.rightFootRefPose.translation[2]=5



q0 = np.ones((15))*0
q_res=ik.solve(q0)
robot.display(q_res)
print(q_res)

    
    
   
