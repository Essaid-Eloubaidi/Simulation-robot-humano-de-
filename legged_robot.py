from pinocchio.utils import *
from pinocchio.explog import exp,log
from numpy.linalg import pinv,norm
from pinocchio import SE3, Model, Inertia, JointModelFreeFlyer, JointModelRX, JointModelRY, JointModelRZ, \
    JointModelPX, JointModelPY, JointModelPZ, forwardKinematics, neutral
import gepetto.corbaserver
from display import Display
import numpy as np
import time
from pinocchio import neutral


class Visual:
    '''
    Class representing one 3D mesh of the robot, to be attached to a joint. The class contains:
    * the name of the 3D objects inside Gepetto viewer.
    * the ID of the joint in the kinematic tree to which the body is attached.
    * the placement of the body with respect to the joint frame.
    This class is only used in the list Robot.visuals (see below).
    '''
    def __init__(self,name,jointParent,placement):
        self.name = name                  # Name in gepetto viewer
        self.jointParent = jointParent    # ID (int) of the joint
        self.placement = placement        # placement of the body wrt joint, i.e. bodyMjoint
    def place(self,display,oMjoint):
        oMbody = oMjoint*self.placement
        display.place(self.name,oMbody,False)

class Robot:
    '''
    Define a class Robot representing a biped robot
    The members of the class are:
    * viewer: a display encapsulating a gepetto viewer client to create 3D
      objects and place them.
    * model: the kinematic tree of the robot.
    * data: the temporary variables to be used by the kinematic algorithms.
    * visuals: the list of all the 'visual' 3D objects to render the robot,
      each element of the list being an object Visual (see above).
    '''

    def __init__(self):
        self.viewer = Display()
        self.visuals = []
        self.model = Model ()
        self.createRobot()
        self.data = self.model.createData()
        self.q0 = neutral (self.model)

    def createLeg(self,rootId=0, prefix='', jointPlacement=SE3.Identity()):
        # Write your code here
        color   = [red,green,blue,transparency] = [1,1,0.78,1.0]
        colorred = [1.0,0.0,0.0,1.0]

        jointId = rootId

        
        jointName          = prefix + "hip_joint_x"
        #jointPlacement     = SE3(eye(3),np.array( [0., 0., -1.0] )) # position joint
        joint              = JointModelRX()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)
        
        jointName          = prefix + "hip_joint_y"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] )) # position joint
        joint              = JointModelRY()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)

        jointName          = prefix + "hip_joint_z"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] )) # position joint
        joint              = JointModelRZ()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)

        
        self.model.appendBodyToJoint(jointId,Inertia.Random(),SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'hip', 0.3,colorred) # weight joint
        self.visuals.append( Visual('world/' + prefix + 'hip',jointId,SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'upper_leg', .2, .2, 2.0,color) # weight member
        self.visuals.append( Visual('world/' + prefix + 'upper_leg',jointId,SE3(eye(3),np.array([0., 0., -1.])))) # position member


        
        
        jointName          = prefix + "knee_joint"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, -2.0] ))
        joint              = JointModelRY()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)
        
        self.model.appendBodyToJoint(jointId,Inertia.Random(),SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'knee', 0.3,colorred)
        self.visuals.append( Visual('world/' + prefix + 'knee',jointId,SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'lower_leg', 0.2, 0.2, 2.,color)
        self.visuals.append( Visual('world/' + prefix + 'lower_leg',jointId,SE3(eye(3),np.array([0., 0., -1.]))))

        
        

        jointName          = prefix + "ankle_joint_x"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, -2.0] ))
        joint              = JointModelRX()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)

        jointName          = prefix + "ankle_joint_y"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] ))
        joint              = JointModelRY()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)
        
        self.model.appendBodyToJoint(jointId,Inertia.Random(),SE3.Identity())
        self.viewer.viewer.gui.addSphere('world/' + prefix + 'ankle', 0.3,colorred)
        self.visuals.append( Visual('world/' + prefix + 'ankle',jointId,SE3.Identity()) )
        self.viewer.viewer.gui.addBox('world/' + prefix + 'foot', 1.0, 0.5, .3,color)
        self.visuals.append( Visual('world/' + prefix + 'foot',jointId,SE3(eye(3),np.array([0.,0., -0.25]))))


        
    def createWaist(self,jointId=0):
        color   = [red,green,blue,transparency] = [1,1,0.78,1.0] 
        colorred = [1.0,0.0,0.0,1.0]
        
        jointName          = "waist_joint_x"
        jointPlacement     = SE3(eye(3),np.array( [0., 0., 0.] )) # position joint
        joint              = JointModelPX()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)
        
        jointName          = "waist_joint_y"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] )) # position joint
        joint              = JointModelPY()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)

        jointName          = "waist_joint_z"
        jointPlacement     = SE3(eye(3),np.array( [0, 0, 0] )) # position joint
        joint              = JointModelPZ()
        jointId = self.model.addJoint(jointId,joint,jointPlacement,jointName)

        self.model.appendBodyToJoint(jointId,Inertia.Random(),SE3.Identity())
        self.viewer.viewer.gui.addBox('world/' + 'weist', 1., 2., 0.5,color) # weight member
        self.visuals.append( Visual('world/'  + 'weist',jointId,SE3(eye(3),np.array([0.,0., 0.5])))) # position member
        self.waistJointId= self.model.nq


        
    def createRobot(self) :
        self.createWaist()
        self.createLeg(self.waistJointId, 'left_', SE3(eye(3),np.array([0.,-1, 0.])) )
        self.leftFootJointId = self.model.nq
        self.createLeg(self.waistJointId, 'right_', SE3(eye(3),np.array([0.,1, 0.])) )
        self.rightFootJointId = self.model.nq


    def display(self,q):
        forwardKinematics(self.model,self.data,q)
        for visual in self.visuals:
            visual.place( self.viewer,self.data.oMi[visual.jointParent] )
        self.viewer.viewer.gui.refresh()

        
        # Create a 7DOF robot.
if __name__ == '__main__':
    robot = Robot()

    # Hide the floor.
    robot.viewer.viewer.gui.setVisibility('world/floor','OFF')

    # Move the robot during 10secs at velocity v.
    dt = 1e-3
    for j in range (robot.model.nv):
        v = np.array (robot.model.nv * [0])
        v [j] = 1
        q = neutral (robot.model)
        for i in range(500):
            q += v*dt
            robot.display(q)
            time.sleep(dt)
        for i in range(500):
            q -= v*dt
            robot.display(q)
            time.sleep(dt)
