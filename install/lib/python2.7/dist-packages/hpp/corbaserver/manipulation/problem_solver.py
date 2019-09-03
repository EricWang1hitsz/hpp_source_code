#!/usr/bin/env python
#
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

def newProblem (client = None, name = None):
    from hpp.corbaserver.problem_solver import newProblem
    if client is None:
        from hpp.corbaserver.manipulation import Client
        client = Client()
    newProblem (client = client, name = name)

from hpp.corbaserver.problem_solver import _convertToCorbaAny, ProblemSolver as Parent

## Definition of a manipulation planning problem
#
#  This class wraps the Corba client to the server implemented by
#  libhpp-manipulation-corba.so
#
#  Some method implemented by the server can be considered as private. The
#  goal of this class is to hide them and to expose those that can be
#  considered as public.
class ProblemSolver (Parent):
    def __init__ (self, robot):
        super (ProblemSolver, self).__init__ (robot, hppcorbaClient = robot.client.basic)

    ## Select a problem by its name.
    #  If no problem with this name exists, a new
    #  hpp::manipulation::ProblemSolver is created and selected.
    #  \param name the problem name.
    #  \return true if a new problem was created.
    def selectProblem (self, name):
        return self.client.manipulation.problem.selectProblem (name)

    ## Return a list of available elements of type type
    #  \param type enter "type" to know what types I know of.
    #              This is case insensitive.
    def getAvailable (self, type):
        if type.lower () == "type":
            res = self.client.basic.problem.getAvailable (type) + \
                  self.client.manipulation.problem.getAvailable (type)
            return res
        try:
            return self.client.basic.problem.getAvailable (type)
        except:
            return self.client.manipulation.problem.getAvailable (type)

    ## Return a list of selected elements of type type
    #  \param type enter "type" to know what types I know of.
    #              This is case insensitive.
    #  \note For most of the types, the list will contain only one element.
    def getSelected (self, type):
        try:
            return self.client.basic.problem.getSelected (type)
        except:
            return self.client.manipulation.problem.getSelected (type)

    ## \name Constraints
    #  \{

    ## Create placement and pre-placement constraints
    #
    # \param width set to None to skip creation of pre-placement constraint
    #
    # See hpp::corbaserver::manipulation::Problem::createPlacementConstraint
    # and hpp::corbaserver::manipulation::Problem::createPrePlacementConstraint
    def createPlacementConstraints (self, placementName, shapeName, envContactName, width = 0.05):
        name = placementName
        self.client.manipulation.problem.createPlacementConstraint (name, shapeName, envContactName)
        if width is not None:
            prename = "pre_" + name
            self.client.manipulation.problem.createPrePlacementConstraint (prename, shapeName, envContactName, width)
            return name, prename
        return name

    ## Return balance constraints created by method
    #  ProblemSolver.createStaticStabilityConstraints
    def balanceConstraints (self):
        return self.balanceConstraints_

    ## Get whether right hand side of a numerical constraint is constant
    #  \param constraintName Name of the numerical constraint,
    #  \return whether right hand side is constant
    #  \note LockedJoint have non constant right hand side
    def getConstantRightHandSide (self, constraintName) :
        if constraintName in self.getAvailable ('LockedJoint'):
            return False
        return self.client.basic.problem.getConstantRightHandSide \
            (constraintName)

    ## Lock degree of freedom of a FreeFlyer joint
    # \param freeflyerBname base name of the joint
    #        (It will be completed by '_xyz' and '_SO3'),
    # \param lockJointBname base name of the LockedJoint constraints
    #        (It will be completed by '_xyz' and '_SO3'),
    # \param values config of the locked joints (7 float)
    def lockFreeFlyerJoint (self, freeflyerBname, lockJointBname,
                            values = (0,0,0,0,0,0,1)):
        lockedJoints = list ()
        self.createLockedJoint (lockJointBname, freeflyerBname, values)
        lockedJoints.append (lockJointBname)
        return lockedJoints

    ## Lock degree of freedom of a planar joint
    # \param jointName name of the joint
    #        (It will be completed by '_xy' and '_rz'),
    # \param lockJointName name of the LockedJoint constraint
    # \param values config of the locked joints (4 float)
    def lockPlanarJoint (self, jointName, lockJointName, values = (0,0,1,0)):
        lockedJoints = list ()
        self.createLockedJoint (lockJointName, jointName, values)
        lockedJoints.append (lockJointName)
        return lockedJoints

    ## \}

    ## \name Solve problem and get paths
    #  \{

    ## Set the problem target to stateId
    # The planner will look for a path from the init configuration to a configuration in
    # state stateId
    def setTargetState (self, stateId):
        self.client.manipulation.problem.setTargetState(stateId)
    ## \}
