#!/usr/bin/env python

# Copyright (c) 2014 CNRS
# Author: Florent Lamiraux
#
# This file is part of hpp-gepetto-viewer.
# hpp-gepetto-viewer is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-gepetto-viewer is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-gepetto-viewer.  If not, see
# <http://www.gnu.org/licenses/>.

import os
from hpp.gepetto import Viewer as Parent

## Simultaneous control to hpp-manipulation-server and gepetto-viewer-server.
#
class Viewer (Parent):
    def __init__ (self, problemSolver, viewerClient = None, collisionURDF = False) :
        self.compositeRobotName = problemSolver.robot.client.basic.robot.getRobotName()
        Parent.__init__ (self, problemSolver, viewerClient, collisionURDF)

    def _initDisplay (self):
        if not self.client.gui.nodeExists(self.compositeRobotName):
            self.client.gui.createGroup (self.compositeRobotName)
            self.client.gui.addToGroup (self.compositeRobotName, self.sceneName)
        dataRootDir = "" # Ignored for now. Will soon disappear
        path = self.robot.urdfPath()
        name = self.compositeRobotName + '/' + self.robot.robotNames[0]
        self.client.gui.addURDF (name, path, dataRootDir)
        if self.collisionURDF:
            self.toggleVisual(False)
        #self.client.gui.addToGroup (name, self.compositeRobotName)

    def loadRobotModel (self, RobotType, robotName, guiOnly = False, collisionURDF = False, frame = None):
        if not guiOnly:
            if frame is None:
                self.robot.insertRobotModel (robotName, RobotType.rootJointType,
                                           RobotType.packageName,
                                           RobotType.urdfName, RobotType.urdfSuffix,
                                           RobotType.srdfSuffix)
            else:
                self.robot.insertRobotModelOnFrame (robotName, frame, RobotType.rootJointType,
                                           RobotType.packageName,
                                           RobotType.urdfName, RobotType.urdfSuffix,
                                           RobotType.srdfSuffix)
        self.buildRobotBodies ()
        self.loadUrdfInGUI (RobotType, robotName)

    def loadHumanoidModel (self, RobotType, robotName, guiOnly = False):
        if not guiOnly:
            self.robot.loadHumanoidModel (robotName, RobotType.rootJointType,
                                          RobotType.packageName,
                                          RobotType.modelName, RobotType.urdfSuffix,
                                          RobotType.srdfSuffix)
        self.buildRobotBodies ()
        self.loadUrdfInGUI (RobotType, robotName)

    def loadEnvironmentModel (self, EnvType, envName, guiOnly = False):
        if not guiOnly:
            self.robot.loadEnvironmentModel (EnvType.packageName, EnvType.urdfName,
                EnvType.urdfSuffix, EnvType.srdfSuffix, envName + "/")
        self.loadUrdfObjectsInGUI (EnvType, envName)
        self.computeObjectPosition ()

    def loadObjectModel (self, RobotType, robotName, guiOnly = False):
        if not guiOnly:
            self.robot.insertRobotModel (robotName, RobotType.rootJointType,
                                    RobotType.packageName, RobotType.urdfName,
                                    RobotType.urdfSuffix, RobotType.srdfSuffix)
        self.buildRobotBodies ()
        base = "collision_" if self.collisionURDF else ""
        self.loadUrdfInGUI (RobotType, base + robotName)
        self.computeObjectPosition ()

    def buildCompositeRobot (self, robotNames):
        self.robot.buildCompositeRobot (robotNames)
        self.buildRobotBodies ()

    def loadUrdfInGUI (self, RobotType, robotName):
        # Load robot in viewer
        dataRootDir = "" # Ignored for now. Will soon disappear
        path = "package://" + RobotType.packageName + '/urdf/' + RobotType.urdfName + RobotType.urdfSuffix + '.urdf'
        nodeName = self.compositeRobotName + "/" + robotName
        if self.collisionURDF:
            self.client.gui.addUrdfCollision (nodeName, path, dataRootDir)
        else:
            self.client.gui.addURDF (nodeName, path, dataRootDir)

    def loadUrdfObjectsInGUI (self, RobotType, robotName):
        dataRootDir = "" # Ignored for now. Will soon disappear
        path = "package://" + RobotType.packageName + '/urdf/' + RobotType.urdfName + RobotType.urdfSuffix + '.urdf'
        self.client.gui.addUrdfObjects (robotName, path, dataRootDir,
                                        not self.collisionURDF)
        self.client.gui.addToGroup (robotName, self.sceneName)
