#!/usr/bin/env python
#
# Copyright (c) 2014 CNRS
# Author: Steve Tonneau
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


######################################################
#
# Script for exporting a motion computed with gepetto viewer to blender
#
###################################################

import gepetto.corbaserver.exporttoblender as etb

## Export object position for a given robot configuration
#
# \param robot the considered robot,
# \param configuration current robot configuration
# \param outData array containing each object position, indexed first by object then by frame
def exportState(viewer, robot, configuration, outData):
	save = viewer.robotConfig
	viewer(configuration)
	etb.exportState(viewer, robot.getRobotName(), outData)
	viewer(save)
	#~ robot.setCurrentConfig(configuration)
	#~ objNames = set([])
	#~ #retrieve object names
	#~ for joint in robot.getAllJointNames():
		#~ for obj in robot.getJointInnerObjects(joint):
			#~ objNames.add(obj)
		#~ for obj in robot.getJointOuterObjects(joint):
			#~ objNames.add(obj)
	#~ while len(objNames) > 0:
		#~ obj = objNames.pop()
		#~ if not outData.has_key(obj):
			#~ outData[obj] = []
		#~ objFrame = outData[obj]
		#~ objFrame.append(robot.getObjectPosition(obj))

## Export object position for a given robot configuration
#
# \param outData data computed by the exportState calls
# \param filename name of the output file where to save the output
def writeDataToFile(robot, outData, filename):
	etb.writeDataToFile(robot.getRobotName(), outData, filename)
	

## Export object positions for a list of robot configurations
#
# \param robot the considered robot,
# \param configurations list of configurations to consider
# \param filename name of the output file where to save the output
def exportStates(viewer, robot, configurations, filename):
	outData = {}
	for configuration in configurations:
		exportState(viewer, robot, configuration, outData)
	writeDataToFile(robot, outData, filename)
	
## Export object positions for a path
#
# \param robot the considered robot,
# \param problem the problem associated with the path computed for the robot
# \param stepsize increment along the path
# \param pathId if of the considered path
# \param filename name of the output file where to save the output
def exportPath(viewer, robot, problem, pathId, stepsize, filename):
	length = problem.pathLength (pathId)
	t = 0
	tau = []
	dt = stepsize / length
	while t < length :
		q = problem.configAtParam (pathId, t)
		tau.append(q)
		t += dt
	exportStates(viewer, robot, tau, filename)

