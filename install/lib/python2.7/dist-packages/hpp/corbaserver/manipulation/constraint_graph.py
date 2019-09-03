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

from __future__ import print_function
from subprocess import Popen
from .constraints import Constraints

## Association of a numerical constraint with the associated passive joints
#
#  Passive joints are information provided to the constraint solver to get
#  better performance and behavior in the resolution.
class _ConstraintAndPassiveJoints (object):
    def __init__ (self, constraint, passiveJoints):
        self.constraint_ = constraint
        self.passiveJoints_ = passiveJoints
    @property
    def constraint (self):
        return self.constraint_
    @property
    def passiveJoints (self):
        return self.passiveJoints_

### Definition of a constraint graph.
##
##  This class wraps the Corba client to the server implemented by
##  libhpp-manipulation-corba.so
##
##  Some method implemented by the server can be considered as private. The
##  goal of this class is to hide them and to expose those that can be
##  considered as public.
class ConstraintGraph (object):
    cmdDot = {
            'pdf': ['dot', '-Gsize=7.5,10', '-Tpdf'],
            'svg': ['dot', '-Gsize=7.5,10', '-Tsvg']
            }
    cmdViewer = {
            'pdf': ['evince'],
            'svg': ['firefox']
            }

    def __init__ (self, robot, graphName, makeGraph = True):
        self.client = robot.client.manipulation
        self.clientBasic = robot.client.basic
        self.graph = robot.client.manipulation.graph
        self.name = graphName
        self.grasps = dict ()
        self.pregrasps = dict ()
        ## A dictionnary mapping the node names to their ID
        self.nodes = dict ()
        ## A dictionnary mapping the edge names to their ID
        self.edges = dict ()
        if makeGraph:
            self.graphId = self.graph.createGraph (graphName)
            self.subGraphId = self.graph.createSubGraph (graphName + "_sg")
        else:
            # fetch graph
            try:
                g = self.graph.getGraph ()
                self.graphId = g[0].id
                self.subGraphId = self.graphId + 1
                for n in g[1].nodes:
                    if n.name in self.nodes:
                        print("Erasing node", n.name, "id", self.nodes[n.name])
                    self.nodes[n.name] = n.id
                for e in g[1].edges:
                    if e.name in self.edges:
                        print("Erasing edge", e.name, "id", self.edges[e.name])
                    self.edges[e.name] = e.id
            except:
                pass

        self.textToTex = dict ()

    ##
    # \name Building the constraint graph
    # \{

    ### Create one or several node
    ## \param node name (resp. list of names) of the node(s) to be created.
    ## \param waypoint set to True when creating waypoint nodes.
    ## \param priority integer (resp. list of) used to order the states. If two states have
    ##                 the same priority, then the order is the order of creation.
    ## \note The order is important. The first should be the most restrictive one as a configuration
    ## will be in the first node for which the constraint are satisfied.
    def createNode (self, node, waypoint = False, priority = None):
        if type (node) is str:
            node = [node]
        if priority is None:
            priority = [ 0, ] * len(node)
        elif isinstance(priority, int):
            priority = [priority]
        for n, p in zip(node, priority):
            self.nodes [n] = self.graph.createNode (self.subGraphId, self._(n), waypoint, p)

    ### Create an edge
    ## \param nodeFrom, nodeTo the extremities of the edge,
    ## \param name name of the edge,
    ## \param weight see note,
    ## \param isInNode name of the node in which paths of the edge are included.
    ##        if None, it consists of the node coming the latest in the list of
    ##        nodes.
    ## \note The weights define the probability of selecting an edge among all the
    ## outgoing edges of a node. The probability of an edge is \f$ \frac{w_i}{\sum_j{w_j}} \f$,
    ## where each \f$ w_j \f$ corresponds to an outgoing edge from a given node.
    ## To have an edge that cannot be selected by the M-RRT algorithm but is still acceptable,
    ## set its weight to zero.
    def createEdge (self, nodeFrom, nodeTo, name, weight = 1, isInNode = None):
        if (type (isInNode) != str) :
            if not isInNode is None:
                from warnings import warn
                warn ("argument isInNode should be of type string")
            else :
                if self.nodes[nodeFrom] > self.nodes[nodeTo]:
                    isInNode = nodeFrom
                else:
                    isInNode = nodeTo
        self.edges [name] = self.graph.createEdge \
                            (self.nodes[nodeFrom], self.nodes[nodeTo],
                             self._(name), weight, self.nodes[isInNode])
        return self.edges [name]

    ## Set in which node an edge is.
    #  \param edge the edge,
    #  \param node the node.
    #  Paths satisfying the edge constraints satisfy the node constraints.
    def setContainingNode (self, edge, node) :
        return self.graph.setContainingNode (self.edges [edge],
                                             self.nodes [node])

    ## Get in which node an edge is.
    #  \param edge the edge,
    #  Paths satisfying the edge constraints satisfy the node constraints.
    def getContainingNode (self, edge) :
        return self.graph.getContainingNode (self.edges [edge])

    ## Set that an edge is short
    #  \param edge name of the edge
    #  \param True or False
    #
    #  When an edge is tagged as short, extension along this edge is
    #  done differently in RRT-like algorithms. Instead of projecting
    #  a random configuration in the destination node, the
    #  configuration to extend itself is projected in the destination
    #  node. This makes the rate of success higher.
    def setShort (self, edge, isShort) :
      return self.client.graph.setShort (self.edges [edge], isShort)

    def isShort (self, edge) :
      return self.client.graph.isShort (self.edges [edge])

    ### Create a WaypointEdge.
    ## \param nodeFrom, nodeTo, name, weight, isInNode see createEdge note,
    ## \param nb number of waypoints,
    ## \note See documentation of class hpp::manipulation::graph::WaypointEdge for more information.
    ##
    ## \warning Waypoint are now specified by hand to allow finer handling
    ## of edge types between waypoints. This function has been updated to be
    ## backward compatible but except for the return value.
    ## For a finer control of what you are doing, set automaticBuilder to
    ## False.
    def createWaypointEdge (self, nodeFrom, nodeTo, name, nb = 1, weight = 1,
                            isInNode = None, automaticBuilder = True):
        if (type (isInNode) != str) :
            if not isInNode is None:
                from warnings import warn
                warn ("argument isInNode should be of type string")
            else :
                if self.nodes[nodeFrom] > self.nodes[nodeTo]:
                    isInNode = nodeFrom
                else:
                    isInNode = nodeTo

        if automaticBuilder:
            n = name + "_e" + str(nb)
        else:
            n = name
        wid = self.edges[n] = self.graph.createWaypointEdge (
                self.nodes[nodeFrom], self.nodes[nodeTo], self._(name),
                nb, weight, self.nodes [isInNode])

        if not automaticBuilder:
            return

        waypoints = list ()
        previous = nodeFrom
        for i in range(nb):
            waypoints.append((name + "_e" + str(i), name + "_n" + str(i)))
            n = waypoints[-1][1]
            e = waypoints[-1][0]
            newN = self.nodes[n] = \
                    self.graph.createNode (self.subGraphId, self._(n), True)
            newE = self.edges[e] = \
                    self.createEdge (previous, n, self._(e), -1,
                                     isInNode)
            self.graph.setWaypoint (wid, i, newE, newN);
            previous = n

    ### Create a LevelSetEdge.
    ## \param nodeFrom, nodeTo, name, weight, isInNode see createEdge note.
    ## \note See documentation of class hpp::manipulation::graph::LevelSetEdge for more information.
    def createLevelSetEdge (self, nodeFrom, nodeTo, name, weight = 1, isInNode = None):
        if isInNode is None:
            if self.nodes[nodeFrom] > self.nodes[nodeTo]:
                isInNode = nodeFrom
            else:
                isInNode = nodeTo
        self.edges [name] =\
            self.graph.createLevelSetEdge (self.nodes[nodeFrom], self.nodes[nodeTo], self._(name), weight, isInNode)

    ## Create grasp constraints between robot gripper and object handle
    #
    #  Creates two contraints between a handle and a gripper.
    #  \li The first constraint named "${name}" is defined by
    #  the type of handle. For instance, an axial handle defines
    #  a five degree of freedom constraint with free rotation
    #  around the x-axis.
    #  \li the second constraint named "${name}/complement" is
    #  the complement to the full transformation constraint. For the axial
    #  handle, it corresponds to the rotation around x.
    #
    #  \param name prefix of the constraint names for storing in
    #         ProblemSolver map,
    #  \param gripper name of the gripper used when it has been created
    #  \param handle name of the handle in the form "object/handle"
    #  where object is the name of the object owning the handle and handle
    #  is the name of the handle in this object.
    #  \param passiveJoints name of the set of passive joints associated to
    #         the grasp constraints as register in ProblemSolver
    #         \sa manipulation.problem_solver.ProblemSolver::addPassiveDofs.
    #
    #  \sa method hpp::corbaserver::manipulation::Problem::createGrasp.
    #
    #  \note Passive joints are only used for path constraints and are for
    #        computational optimization only.
    def createGrasp (self, name, gripper, handle, passiveJoints = ""):
        self.client.problem.createGrasp (self._(name), gripper, handle)
        self.grasps [name] = (_ConstraintAndPassiveJoints (self._(name),
                                                          passiveJoints),)

    ## Create pre-grasp constraints between robot gripper and object handle
    #
    #  Creates two contraints between a handle and a gripper.
    #  \li The first constraint named "${name}" is the same as the grasp
    #  defined in createGrasp, except that the translation along x is not
    #  constrained. For instance, an axial handle defines
    #  a four degree of freedom constraint with free rotation and translation
    #  around/along the x-axis,
    #  \li the second constraint named "${name}/double_ineq" is a double
    #  inequality on the relative x-position of the handle and of the gripper.
    #  the bounds of the inequality are for now [-.001 c, 2.001 c].
    #
    #  \param name prefix of the constraint names for storing in
    #         ProblemSolver map,
    #  \param gripper name of the gripper used when it has been created
    #  \param handle name of the handle in the form "object/handle"
    #  where object is the name of the object owning the handle and handle
    #  is the name of the handle in this object,
    #  \param passiveJoints name of the set of passive joints associated to
    #         the pre-grasp constraints as register in ProblemSolver.
    #         \sa manipulation.problem_solver.ProblemSolver::addPassiveDofs.
    #
    #  \sa hpp::corbaserver::manipulation::Problem::createPreGrasp
    #
    #  \note Passive joints are only used for path constraints and are for
    #        computational optimization only.
    def createPreGrasp (self, name, gripper, handle, passiveJoints = ""):
        self.client.problem.createPreGrasp (self._(name), gripper, handle)
        self.pregrasps [name] = \
            (_ConstraintAndPassiveJoints (self._(name), passiveJoints),)

    ## Set the problem constraints to the specified constraint.
    # 
    #  \param idComp ID of a node or a configuration
    #  \param target: ignored for states. For edges:
    #         \li true: uses the edge target constraint
    #         \li false: uses the edge path constraint
    def setProblemConstraints (self, name, target):
        if name in self.nodes:
            id = self.nodes[name]
        elif name in self.edges:
            id = self.edges[name]
        else:
            raise RuntimeError ("No node or edge with name {0}".format (name))
        return self.client.problem.setConstraints (id, target)

    ## Add the constraints to an edge, a node or the whole graph
    #
    # This method adds the constraints to an element of the graph and handles
    # the special cases of grasp and pregrasp constraints.
    #
    # \param graph set to true if you are defining constraints for every nodes,
    # \param node edge name of a component of the graph,
    #
    # \param constraints set of constraints containing grasps, pregrasps,
    #                    numerical constraints and locked joints.
    #                    It must be of type hpp.corbaserver.manipulation.Constraints.
    # \param passiveJoints passive joints (not modified by constraint
    #                      resolution)
    # \note Exaclty one of the parameter graph, node and edge must be set.
    def addConstraints (self, graph = False, node = None, edge = None,
                        constraints = None, passiveJoints = []):
        """
        Add the constraints to an edge, a node or the whole graph

          This method adds the constraints to an element of the graph and
          handles the special cases of grasp and pregrasp constraints.

          input
            graph: set to true if you are defining constraints for every nodes,
            node, edge: name of a component of the graph,
            constraints: set of constraints containing grasps, pregrasps,
                         numerical constraints and locked joints.
                         It must be of type hpp.corbaserver.manipulation.Constraints.
            passiveJoints: passive joints (not modified by constraint
                           resolution)
          note: Exaclty one of the parameter graph, node and edge must be set.
        """
        if not isinstance (constraints, Constraints):
            raise TypeError ("argument constraints should be of type Constraints")
        return self._addConstraints \
            (graph = graph, node = node, edge = edge,
             grasps = constraints.grasps,
             pregrasps = constraints.pregrasps,
             lockDof = constraints.lockedJoints,
             numConstraints = constraints.numConstraints,
             passiveJoints = passiveJoints)

    def _addConstraints (self, graph = False, node = None, edge = None,
                         grasps = None, pregrasps = None, lockDof = [],
                         numConstraints = [], passiveJoints = []):
        if not type (graph) is bool:
            raise TypeError ("ConstraintGraph.addConstraints: " +\
                             "graph argument should be a boolean, got " + \
                             repr (graph))
        nc = numConstraints [:]
        nopdofs = ["" for i in range (len(numConstraints))]
        pdofs = nopdofs [::]
        pdofs [:len(passiveJoints)] = passiveJoints [:]
        if grasps is not None:
            for g in grasps:
                for pair in self.grasps [g]:
                    if edge is not None:
                        nc.append (pair.constraint + "/complement")
                    else:
                        nc.append (pair.constraint)
                    nopdofs.append("")
                    pdofs.append (pair.passiveJoints)
        if pregrasps is not None:
            for g in pregrasps:
                for pair in self.pregrasps [g]:
                    nc.append (pair.constraint)
                    nopdofs.append("")
                    pdofs.append (pair.passiveJoints)

        if node is not None:
            self.graph.addNumericalConstraints (self.nodes [node], nc, nopdofs)
            self.graph.addNumericalConstraintsForPath (self.nodes [node], nc,
                                                       pdofs)
            self.graph.addLockedDofConstraints (self.nodes [node], lockDof)
        elif edge is not None:
            self.graph.addNumericalConstraints (self.edges [edge], nc, nopdofs)
            self.graph.addLockedDofConstraints (self.edges [edge], lockDof)
        elif graph:
            self.graph.addNumericalConstraints (self.graphId, nc, nopdofs)
            self.graph.addLockedDofConstraints (self.graphId, lockDof)

    ## Remove collision pairs from an edge
    #
    #  \param edge name of the edge,
    #  \param joint1, joint2, names of the joints defining the pair.
    def removeCollisionPairFromEdge (self, edge, joint1, joint2):
        return self.graph.removeCollisionPairFromEdge \
            (self.edges [edge], joint1, joint2)

    def setLevelSetFoliation (self, *args, **kwargs):
        return self.addLevelSetFoliation (*args, **kwargs)

    ## Add the numerical constraints to a LevelSetEdge that create the foliation.
    #  \param edge name of a LevelSetEdge of the graph.
    #  \param condGrasps, condPregrasps name, or list of names, of grasp or pregrasp that define the foliated manifold
    #  \param condNC, condLJ numerical constraints and locked joints that define the foliated manifold
    #  \param paramGrasps, paramPregrasps name, or list of names, of grasp or pregrasp that parametrize the foliation
    #  \param paramNC, paramPassiveJoints, paramLJ numerical constraints and locked joints that parametrize the foliation
    #  \note If passiveDofsNames is a shorter list than numConstraints, passiveDofsNames is extended with an empty string,
    #        which corresponds to an empty vector of passive dofs.
    def addLevelSetFoliation (self, edge,
            condGrasps = None, condPregrasps = None, condNC = [], condLJ = [],
            paramGrasps = None, paramPregrasps = None, paramNC = [], paramPassiveJoints = [], paramLJ = []):

        cond_nc = condNC [:]
        if condGrasps is not None:
            for g in condGrasps:
                for pair in self.grasps [g]:
                    cond_nc.append (pair.constraint)
        if condPregrasps is not None:
            for g in condPregrasps:
                for pair in self.pregrasps [g]:
                    cond_nc.append (pair.constraint)

        param_nc = paramNC [:]
        pdofs = ["" for i in range (len(paramNC))]
        pdofs [:len(paramPassiveJoints)] = paramPassiveJoints [:]
        if paramGrasps is not None:
            for g in paramGrasps:
                for pair in self.grasps [g]:
                    param_nc.append (pair.constraint)
                    pdofs.append (pair.passiveJoints)
        if paramPregrasps is not None:
            for g in paramPregrasps:
                for pair in self.pregrasps [g]:
                    param_nc.extend (pair.constraint)
                    pdofs.extend (pair.passiveJoints)

        self.graph.addLevelSetFoliation (self.edges [edge], cond_nc, condLJ, param_nc, pdofs, paramLJ)

    ## Get weight of an edge
    #
    def getWeight (self, edge):
        return self.client.graph.getWeight (self.edges [edge])

    ## Set weight of an edge
    #
    def setWeight (self, edge, weight):
        if self.client.graph.getWeight (self.edges [edge]) == -1:
            raise RuntimeError ('You cannot set weight for "' + edge +
                                '". Perhaps it is a waypoint edge ?')
        return self.client.graph.setWeight (self.edges [edge], weight)

    ## \}

    ## Add entry to the local dictionnary
    # \param text plain text
    # \param tex its latex translation
    # \sa ConstraintGraph.setTextToTeXTranslation
    def addTextToTeXTranslation (self, text, tex):
        self.textToTex[text] = tex

    ## Set the local dictionnary
    # \param textToTex a dictionnary of (plain text, TeX replacment)
    # If the name of a node or an edges is a key of the dictionnary,
    # it is replaced by the corresponding value.
    def setTextToTeXTranslation (self, textToTex):
        if type (textToTex) is not dict:
            raise TypeError ("Argument textToTex must be a dictionnary.")
        self.textToTex = textToTex

    ##
    # \name Working with the constraint graph
    # \{

    ### Display the current graph.
    ## The graph is printed in DOT format. Command dot must be
    ## available.
    ## \param dotOut full path of the generated DOT file.
    ## \param pdfOut fill path of the generated PDF document.
    ## \param openPDF set to False if you just want to generate the PDF.
    ## \note DOT and PDF files will be overwritten and are not automatically
    ## deleted so you can keep them.
    def display (self, dotOut = '/tmp/constraintgraph.dot', pdfOut = '/tmp/constraintgraph', format = 'pdf', open = True):
        self.graph.display (dotOut)
        if format not in self.cmdDot:
            raise TypeError ("This format is not supported. See member cmdDot for supported format.")
        dotCmd = self.cmdDot [format]
        dotCmd.append ('-o' + pdfOut + '.' + format)
        dotCmd.append (dotOut)
        dot = Popen (dotCmd)
        dot.wait ()
        if open and format in self.cmdViewer:
            viewCmd = self.cmdViewer [format]
            viewCmd.append (pdfOut + '.' + format)
            Popen (viewCmd)

    ## Get nodes connected by an edge
    #
    #  \param edge name of the edge
    #  \param from name of the node the edge starts from,
    #  \param to name of the node the edge finishes in.
    def getNodesConnectedByEdge (self, edge):
        return self.client.graph.getNodesConnectedByEdge (self.edges [edge])

    ## Apply constaints to a configuration
    #
    #  \param node name of the node the constraints of which to apply
    #  \param input input configuration,
    #  \retval output output configuration,
    #  \retval error norm of the residual error.
    def  applyNodeConstraints (self, node,  input) :
        return self.client.problem.applyConstraints (self.nodes [node],
                                                     input)

    ## Apply edge constaints to a configuration
    #
    #  \param edge name of the edge
    #  \param qfrom configuration defining the right hand side of the edge
    #         constraint,
    #  \param input input configuration,
    #  \retval output output configuration,
    #  \retval error norm of the residual error.
    #
    #  Compute a configuration in the destination node of the edge,
    #  reachable from qFrom.
    def generateTargetConfig (self, edge, qfrom, input) :
        return self.client.problem.applyConstraintsWithOffset \
            (self.edges [edge], qfrom, input)

    ## Build a path from qb to qe using the Edge::build.
    #  \param edge name of the edge to use.
    #  \param qb configuration at the beginning of the path
    #  \param qe configuration at the end of the path
    #  \retval return true if the path is built and fully projected.
    #  \retval indexNotProj -1 is the path could not be built. The index
    #                       of the built path (before projection) in the
    #                       in the ProblemSolver path vector.
    #  \retval indexProj -1 is the path could not be fully projected. The
    #                    index of the built path (before projection) in the
    #                    in the ProblemSolver path vector.
    #  No path validation is made. The paths can be retrieved using
    #  corbaserver::Problem::configAtParam
    def buildAndProjectPath (self, edge, qb, qe) :
        return self.client.problem.buildAndProjectPath \
            (self.edges [edge], qb, qe)

    ## Get error of a config with respect to a node constraint
    #
    #  \param node name of the node.
    #  \param config Configuration,
    #  \retval error the error of the node constraint for the
    #         configuration
    #  \return whether the configuration belongs to the node.
    #  Call method core::ConstraintSet::isSatisfied for the node
    #  constraints.
    def getConfigErrorForNode (self, nodeId, config) :
        return self.client.graph.getConfigErrorForNode \
          (self.nodes [nodeId], config)

    ##  Get the node corresponding to the state of the configuration.
    #  \param dofArray the configuration.
    #  \return the name of the node
    def getNode (self, config):
        nodeId = self.client.graph.getNode (config)
        for n,id in self.nodes.items ():
            if id == nodeId: return n
        raise RuntimeError ("No node with id {0}".format (nodeId))

    ## Get error of a config with respect to a edge constraint
    #
    #  \param edge name of the edge.
    #  \param config Configuration,
    #  \retval error the error of the edge constraint for the
    #         configuration
    #  \return whether the configuration belongs to the edge.
    #  Call methods core::ConfigProjector::rightHandSideFromConfig with
    #  the input configuration and then core::ConstraintSet::isSatisfied
    #  on the edge constraints.
    def getConfigErrorForEdge (self, edgeId, config) :
        return self.client.graph.getConfigErrorForEdge \
          (self.edges [edgeId], config)

    ## Get error of a config with respect to an edge foliation leaf
    #
    #  \param edgeId id of the edge.
    #  \param leafConfig Configuration that determines the foliation leaf,
    #  \param config Configuration the error of which is computed
    #  \retval error the error
    #  \return whether config can be the end point of a path of the edge
    #          starting at leafConfig
    #  Call methods core::ConfigProjector::rightHandSideFromConfig with
    #  leafConfig and then core::ConstraintSet::isSatisfied with config.
    #  on the edge constraints.
    def getConfigErrorForEdgeLeaf (self, edgeId, leafConfig, config) :
        return self.client.graph.getConfigErrorForEdgeLeaf \
            (self.edges[edgeId], leafConfig, config)

    ## Get error of a config with respect to the target of an edge foliation leaf
    #
    #  \param edgeId id of the edge.
    #  \param leafConfig Configuration that determines the foliation leaf,
    #  \param config Configuration the error of which is computed
    #  \retval error the error
    #  \return whether config can be the end point of a path of the edge
    #          starting at leafConfig
    #  Call methods core::ConfigProjector::rightHandSideFromConfig with
    #  leafConfig and then core::ConstraintSet::isSatisfied with config.
    #  on the edge constraints.
    def getConfigErrorForEdgeTarget (self, edgeId, leafConfig, config) :
        return self.client.graph.getConfigErrorForEdgeTarget \
            (self.edges[edgeId], leafConfig, config)

    ## Print set of constraints relative to a node in a string
    #
    #  \param config Configuration,
    #  \param nodeId id of the node.
    #  \return string displaying constraints
    def displayNodeConstraints (self, node) :
        return self.graph.displayNodeConstraints (self.nodes [node])

    ## Print set of constraints relative to an edge in a string
    #
    #  \param config Configuration,
    #  \param edgeId id of the edge.
    #  \return string displaying path constraints of the edge
    def displayEdgeConstraints (self, edge) :
        return self.graph.displayEdgeConstraints (self.edges [edge])

    ## Print set of constraints relative to an edge in a string
    #
    #  \param config Configuration,
    #  \param edgeId id of the edge.
    #  \return string displaying constraints of the edge and of the target
    #          node
    def displayEdgeTargetConstraints (self, edge) :
        return self.graph.displayEdgeTargetConstraints (self.edges [edge])
    ##
    # \}

    ##
    # \name Automatic building
    # \{
    @staticmethod
    ## Build a graph
    # \return a Initialized ConstraintGraph object
    # \sa hpp::corbaserver::manipulation::Graph::autoBuild for complete
    #     documentation.
    def buildGenericGraph (robot, name, grippers, objects, handlesPerObjects, shapesPerObjects, envNames, rules = []):
        robot.client.manipulation.graph.autoBuild \
                (name, grippers, objects, handlesPerObjects, shapesPerObjects, envNames, rules)
        graph = ConstraintGraph (robot, name, makeGraph = False); 
        graph.initialize()
        return graph

    def initialize (self):
        self.graph.initialize()

    ##
    # \}

    ## get the textToTex translation
    def _ (self, text):
        return self.textToTex.get (text, text)
