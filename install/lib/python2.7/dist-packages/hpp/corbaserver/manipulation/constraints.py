#!/usr/bin/env python
#
# Copyright (c) 2014 CNRS
# Author: Joseph Mirabel
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


class Constraints (object):
    """
    Store constraints of various types

      - grasps,
      - pregrasps.
      - numerical constraints,
      - lockedJoints
    """
    def __init__ (self, grasps = [], pregrasps = [], numConstraints = [],
                  lockedJoints = []):
        if type (grasps) is str:
            raise TypeError ("argument grasps should be a list of strings")
        if type (pregrasps) is str:
            raise TypeError ("argument pregrasps should be a list of strings")
        if type (numConstraints) is str:
            raise TypeError \
                ("argument numConstraints should be a list of strings")
        if type (lockedJoints) is str:
            raise TypeError \
                ("argument lockedJoints should be a list of strings")
        self._grasps = set (grasps)
        self._pregrasps = set (pregrasps)
        self._numConstraints = set (numConstraints)
        self._lockedJoints = set (lockedJoints)

    def __add__ (self, other):
        res = Constraints (grasps = self._grasps | other._grasps,
                           pregrasps = self._pregrasps |  other._pregrasps,
                           numConstraints =
                           self._numConstraints | other._numConstraints,
                           lockedJoints =
                           self._lockedJoints | other._lockedJoints)
        return res

    def __sub__ (self, other):
        res = Constraints (grasps = self._grasps - other._grasps,
                           pregrasps = self._pregrasps - other._pregrasps,
                           numConstraints = self._numConstraints - \
                           other._numConstraints,
                           lockedJoints = self._lockedJoints - \
                           other._lockedJoints)
        return res

    def __iadd__ (self, other):
        self._grasps |= other._grasps
        self._pregrasps |= other._pregrasps
        self._numConstraints |= other._numConstraints
        self._lockedJoints |= other._lockedJoints
        return self

    def __isub__ (self, other):
        self._grasps -= other._grasps
        self._pregrasps -= other._pregrasps
        self._numConstraints -= other._numConstraints
        self._lockedJoints -= other._lockedJoints
        return self

    def empty (self):
        for s in [ self._grasps, self._pregrasps, self._numConstraints, self._lockedJoints ]:
            if len(s) > 0: return False
        return True

    @property
    def grasps (self):
        return list (self._grasps)

    @property
    def pregrasps (self):
        return list (self._pregrasps)

    @property
    def numConstraints (self):
        return list (self._numConstraints)

    @property
    def lockedJoints (self):
        return list (self._lockedJoints)

    def __str__ (self):
        res = "constraints\n"
        res += "  grasps: "
        for c in self._grasps:
            res += c + ', '
        res += "\n  pregrasps: "
        for c in self._pregrasps:
            res += c + ', '
        res += "\n  numConstraints: "
        for c in self._numConstraints:
            res += c + ', '
        res += "\n  lockedJoints: "
        for c in self._lockedJoints:
            res += c + ', '
        return res
