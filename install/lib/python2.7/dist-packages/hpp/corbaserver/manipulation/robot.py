#!/usr/bin/env python
# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-manipulation-corba.
# hpp-manipulation-corba is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-manipulation-corba is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-manipulation-corba.  If not, see
# <http://www.gnu.org/licenses/>.

import warnings
from hpp import Transform
from hpp.corbaserver.manipulation import Client as ManipulationClient
from hpp.corbaserver import Client as BasicClient
from hpp.corbaserver.robot import Robot as Parent

## Corba clients to the various servers
#
class CorbaClient:
    """
    Container for corba clients to various interfaces.
    """
    def __init__ (self, url = None, context = "corbaserver"):
        self.basic = BasicClient (url = url, context = context)
        self.manipulation = ManipulationClient (url = url, context = context)

## Load and handle a composite robot for manipulation planning
#
#  A composite robot is a kinematic chain composed of several sub-kinematic
#  chains rooted at an anchor joint.
class Robot (Parent):
    ## Constructor
    # \param robotName name of the first robot that is loaded now,
    # \param rootJointType type of root joint among ("freeflyer", "planar",
    #        "anchor"),
    # \param load whether to actually load urdf files. Set to no if you only
    #        want to initialize a corba client to an already initialized
    #        problem.
    def __init__ (self, compositeName = None, robotName = None, rootJointType = None, load = True, client = None):
        if client is None: client = CorbaClient()
        super (Robot, self).__init__ (robotName = compositeName,
                rootJointType = rootJointType,
                load = False, client = client,
                hppcorbaClient = client.basic)
        self.rootJointType = dict()
        if compositeName is None:
            load = False
        self.load = load
        self.robotNames = list()
        if robotName is None:
            if load:
                self.client.basic.robot.createRobot (self.name)
        else:
            self.loadModel (robotName, rootJointType)

    ## Virtual function to load the robot model
    def loadModel (self, robotName, rootJointType):
        if self.load:
            self.client.basic.robot.createRobot (self.name)
        self.insertRobotModel (robotName, rootJointType, self.packageName,
                               self.urdfName, self.urdfSuffix, self.srdfSuffix)

    ## Load robot model and insert it in the device
    #
    #  \param robotName key of the robot in hpp::manipulation::ProblemSolver object
    #         map (see hpp::manipulation::ProblemSolver::addRobot)
    #  \param rootJointType type of root joint among "anchor", "freeflyer",
    #         "planar",
    #  \param packageName Name of the ROS package containing the model,
    #  \param modelName Name of the package containing the model
    #  \param urdfSuffix suffix for urdf file,
    #
    #  The ros url are built as follows:
    #  \li "package://${packageName}/urdf/${modelName}${urdfSuffix}.urdf"
    #  \li "package://${packageName}/srdf/${modelName}${srdfSuffix}.srdf"
    def insertRobotModel (self, robotName, rootJointType, packageName,
            modelName, urdfSuffix, srdfSuffix):
        if self.load:
            self.client.manipulation.robot.insertRobotModel (robotName,
                    rootJointType, packageName, modelName, urdfSuffix,
                    srdfSuffix)
        self.robotNames.append (robotName)
        self.rootJointType[robotName] = rootJointType
        self.rebuildRanks ()

    ## Insert robot model as a child of a frame of the Device
    #
    # \param robotName key of the robot in ProblemSolver object map
    #        (see hpp::manipulation::ProblemSolver::addRobot)
    # \param frameName name of the existing frame that will the root of the added robot,
    # \param rootJointType type of root joint among "anchor", "freeflyer",
    # "planar",
    # \param packageName Name of the ROS package containing the model,
    # \param modelName Name of the package containing the model
    # \param urdfSuffix suffix for urdf file,
    #
    # The ros url are built as follows:
    # "package://${packageName}/urdf/${modelName}${urdfSuffix}.urdf"
    # "package://${packageName}/srdf/${modelName}${srdfSuffix}.srdf"
    #
    def insertRobotModelOnFrame (self, robotName, frameName, rootJointType,
            packageName, modelName, urdfSuffix, srdfSuffix):
        if self.load:
            self.client.manipulation.robot.insertRobotModelOnFrame (robotName,
                    frameName, rootJointType, packageName, modelName,
                    urdfSuffix, srdfSuffix)
        self.robotNames.append (robotName)
        self.rootJointType[robotName] = rootJointType
        self.rebuildRanks ()

    ## Same as Robot.insertRobotModel
    #
    #  \param urdfString XML string of the URDF,
    #  \param srdfString XML string of the SRDF
    def insertRobotModelFromString (self, robotName, rootJointType, urdfString, srdfString):
        if self.load:
            self.client.manipulation.robot.insertRobotModelFromString (robotName,
                    rootJointType, urdfString, srdfString)
        self.robotNames.append (robotName)
        self.rootJointType[robotName] = rootJointType
        self.rebuildRanks ()

    ## Load a SRDF for the robot. Several SRDF can thus be loaded for the same robot
    #
    #  \param robotName key of the robot in hpp::manipulation::Device object
    #         map (see hpp::manipulation::Device)
    #  \param packageName Name of the ROS package containing the model,
    #  \param modelName Name of the package containing the model
    #  \param srdfSuffix suffix for srdf file,
    #
    #  The ros url are built as follows:
    #  \li "package://${packageName}/srdf/${modelName}${srdfSuffix}.srdf"
    def insertRobotSRDFModel (self, robotName, packageName,
            modelName, srdfSuffix):
        if self.load:
            self.client.manipulation.robot.insertRobotSRDFModel (robotName,
                    packageName, modelName, srdfSuffix)

    ## Load humanoid robot model and insert it in the device
    #
    #  \param robotName key of the robot in ProblemSolver object map
    #         (see hpp::manipulation::ProblemSolver::addRobot)
    #  \param rootJointType type of root joint among "anchor", "freeflyer",
    #         "planar",
    #  \param packageName Name of the ROS package containing the model,
    #  \param modelName Name of the package containing the model
    #  \param urdfSuffix suffix for urdf file,
    #
    #  The ros url are built as follows:
    #  \li "package://${packageName}/urdf/${modelName}${urdfSuffix}.urdf"
    #  \li "package://${packageName}/srdf/${modelName}${srdfSuffix}.srdf"
    def insertHumanoidModel (self, robotName, rootJointType, packageName,
                           modelName, urdfSuffix, srdfSuffix):
        if self.load:
            self.client.manipulation.robot.insertHumanoidModel \
                (robotName, rootJointType, packageName, modelName,
                 urdfSuffix, srdfSuffix)
        self.robotNames.append (robotName)
        self.rootJointType[robotName] = rootJointType
        self.rebuildRanks ()

    def loadHumanoidModel (self, robotName, rootJointType, packageName,
                           modelName, urdfSuffix, srdfSuffix):
        self.insertHumanoidModel (robotName, rootJointType, packageName,
                           modelName, urdfSuffix, srdfSuffix)

    ## Load environment model and store in local map.
    #  Contact surfaces are build from the corresping srdf file.
    #  See hpp-manipulation-urdf for more details about contact surface
    #  specifications.
    #
    #  \param envName key of the object in ProblemSolver object map
    #         (see hpp::manipulation::ProblemSolver::addRobot)
    #  \param packageName Name of the ROS package containing the model,
    #  \param modelName Name of the package containing the model
    #  \param urdfSuffix suffix for urdf file,
    #  \param srdfSuffix suffix for srdf file.
    #
    #  The ros url are built as follows:
    #  \li "package://${packageName}/urdf/${modelName}${urdfSuffix}.urdf"
    #  \li "package://${packageName}/srdf/${modelName}${srdfSuffix}.srdf"
    def loadEnvironmentModel (self, packageName, modelName,
                         urdfSuffix, srdfSuffix, envName):
        if self.load:
            self.client.manipulation.robot.loadEnvironmentModel (packageName,
                    modelName, urdfSuffix, srdfSuffix, envName)
        self.rootJointType[envName] = "Anchor"

    ## \name Joints
    #\{

    ## Set the position of root joint of a robot in world frame
    ## \param robotName key of the robot in ProblemSolver object map.
    ## \param position constant position of the root joint in world frame in
    ##        initial configuration.
    def setRootJointPosition (self, robotName, position):
        return self.client.manipulation.robot.setRootJointPosition (robotName, position)

    ## \}

    ## \name Bodies
    #  \{

    ## Return the joint name in which a gripper is and the position relatively
    #  to the joint
    def getGripperPositionInJoint (self, gripperName):
        return self.client.manipulation.robot.getGripperPositionInJoint (gripperName)

    ## Return the joint name in which a handle is and the position relatively
    #  to the joint
    def getHandlePositionInJoint (self, handleName):
        return self.client.manipulation.robot.getHandlePositionInJoint (handleName)

    ## \}

from hpp.corbaserver.robot import StaticStabilityConstraintsFactory
class HumanoidRobot (Robot, StaticStabilityConstraintsFactory):
    ## Constructor
    # \param compositeName name of the composite robot that will be built later,
    # \param robotName name of the first robot that is loaded now,
    # \param rootJointType type of root joint among ("freeflyer", "planar",
    #        "anchor"),
    def __init__ (self, compositeName = None, robotName = None, rootJointType = None, load = True, client = None):
        Robot.__init__ (self, compositeName, robotName, rootJointType, load, client)

    def loadModel (self, robotName, rootJointType):
        self.client.basic.robot.createRobot (self.name)
        self.insertHumanoidModel \
            (robotName, rootJointType, self.packageName, self.urdfName,
             self.urdfSuffix, self.srdfSuffix)
