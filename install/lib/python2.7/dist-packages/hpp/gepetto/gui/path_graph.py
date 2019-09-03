from __future__ import print_function
from PythonQt import QtGui, QtCore, Qt

from hpp import Quaternion
import hpp_idl
from hpp.corbaserver import Client
from gepetto.corbaserver import Client as GuiClient
from gepetto.corbaserver.tools import Linear, Vector6
from gepetto.color import Color

import numpy as np
from math import pi, sqrt, cos, sin

colors = (
        # Qt.Qt.white,
        Qt.Qt.black,
        Qt.Qt.red,
        Qt.Qt.green,
        Qt.Qt.blue,
        Qt.Qt.cyan,
        Qt.Qt.magenta,
        Qt.Qt.yellow,
        Qt.Qt.gray,
        Qt.Qt.darkRed,
        Qt.Qt.darkGreen,
        Qt.Qt.darkBlue,
        Qt.Qt.darkCyan,
        Qt.Qt.darkMagenta,
        Qt.Qt.darkYellow,
        Qt.Qt.darkGray,
        Qt.Qt.lightGray,
        )
lineStyles = (
        Qt.Qt.SolidLine,
        Qt.Qt.DashLine,
        Qt.Qt.DotLine,
        Qt.Qt.DashDotLine,
        Qt.Qt.DashDotDotLine,
        )

pens = []
for ls in lineStyles:
    for c in colors:
        qpen = Qt.QPen(c)
        qpen.setStyle (ls)
        pens.append (qpen)

pens = tuple(pens)

class VelGetter:
    def __init__(self, plugin, name):
        self.plugin = plugin
        self.name = str(name)
        self.vector6 = Vector6 (self.name)

    def getV(self):
        return self.plugin.client.robot.getJointVelocityInLocalFrame (self.name)

    def createNodes(self, color):
        self.group = str(self.plugin.jointgroupcreator.requestCreateJointGroup(self.name))
        self.vector6.color = color
        self.vector6.create(self.plugin.gui.gui)

    def removeNodes(self):
        self.vector6.remove(self.plugin.gui.gui)

    def apply(self):
        v = self.getV()
        self.vector6.set (self.plugin.gui.gui, v)

class ComGetter:
    def __init__(self, plugin, com):
        self.plugin = plugin
        self.name = str(com)
        self.linear = Linear (self.name)

    def getV(self):
        return self.plugin.client.robot.getVelocityPartialCom (self.name)

    def createNodes(self, color):
        self.group = str(self.plugin.comgroupcreator.requestCreateComGroup(self.name))
        self.linear.color = color
        self.linear.create (self.plugin.gui.gui)

    def removeNodes(self):
        self.linear.remove (self.plugin.gui.gui)

    def apply(self):
        v = self.getV()
        self.linear.set(self.plugin.gui.gui, v)

class Velocities:
    color = Color.red
    def __init__(self, plugin):
        self.plugin = plugin
        self.connected = False

        # For each arrow to plot, there are
        # - a node
        # - a getter
        self.getters = dict()

    def connect(self):
        if not self.connected:
            self.plugin.pathPlayer.setRobotVelocity(True)
            self.plugin.main.connect(QtCore.SIGNAL('applyCurrentConfiguration()'),
                    self.applyAll)
            self.connected = True

    def disconnect(self):
        if self.connected:
            self.plugin.main.disconnect(QtCore.SIGNAL('applyCurrentConfiguration()'),
                    self.applyAll)
            self.connected = False

    def add (self, getter):
        self.getters[getter.name] = getter
        getter.apply ()
        self.connect()

    def remove (self, node):
        getter = self.getters.pop(node)
        getter.removeNodes()
        if len(self.getters) == 0:
            self.disconnect()

    def toggleJoint (self, joint):
        if joint in self.getters: self.remove(joint)
        else:
            g = VelGetter(self.plugin, joint)
            g.createNodes (Velocities.color)
            self.add (g)

    def toggleCom (self, com):
        if com in self.getters: self.remove (com)
        else:
            g = ComGetter (self.plugin, com)
            g.createNodes (Velocities.color)
            self.add (g)

    def applyAll (self):
        for n, getV in list(self.getters.items()):
            getV.apply()

class JointAction(Qt.QAction):
    def __init__ (self, action, joint, velocities, parent):
        super(JointAction, self).__init__ (action, parent)
        self.joint = joint
        self.velocities = velocities
        self.connect (QtCore.SIGNAL('triggered(bool)'), self.trigger)

    def trigger (self):
        self.velocities.toggleJoint (self.joint)

class DataQCP:
    def __init__ (self, plugin):
        self.plugin = plugin
        self.qcp = self.plugin.qcpWidget
        self.timer = Qt.QTimer()
        self.timer.setSingleShot(True)

        self.pathLength = -1
        self.dl = -1
        self.pathId = -1

    def selectData (self, pid, dl, x, ys):
        if dl < 1e-8:
            return
        self.x = x
        self.ys = ys
        pl = self.plugin.client.problem.pathLength(pid)
        if   not self.pathLength == pl \
            or not self.pathId     == pid \
            or not self.dl         == dl:
            self.dataAreOld = True
            self.pathId = pid
            self.pathLength = pl
            self.dl = dl
            self.l = 0
            self.datas = list ()
        else:
            self.dataAreOld = False
        self.qcp.clearGraphs()
        for i, elt in enumerate(self.ys):
            graph = self.qcp.addGraph ()
            graph.name = elt[0]
            graph.pen = pens[i]
        self.qcp.xAxis().label = self.x[0]

    def acquireData (self):
        if self.dataAreOld:
            self._setNextCall (self._getNextData)
        else:
            self._setNextCall (self._genPlot)
        self.timer.start(0)

    def _setNextCall (self, f):
        self.timer.disconnect(Qt.SIGNAL("timeout()"))
        self.timer.connect(Qt.SIGNAL("timeout()"), f)

    def _getDerivative (self, order):
        try:
            if order == 0:
                return self.plugin.client.problem.configAtParam (self.pathId, self.l)
            else:
                return self.plugin.client.problem.derivativeAtParam (self.pathId, order, self.l)
        except hpp_idl.hpp.Error as e:
            if order == 0:
                return [np.nan] * self.plugin.client.robot.getConfigSize()
            else:
                return [np.nan] * self.plugin.client.robot.getNumberDof()
            print(str(e))

    def _getNextData (self):
        d = [ self.l, ]
        d.extend (self._getDerivative(0))
        d.extend (self._getDerivative(1))
        d.extend (self._getDerivative(2))
        self.datas.append (d)

        for i, elt in enumerate(self.ys):
            graph = self.qcp.graph (i)
            graph.addData (self.l, d[elt[1]])
        self.qcp.replot()

        self.l += self.dl
        if self.l < self.pathLength:
            self._setNextCall (self._getNextData)
        else:
            self._setNextCall (self._finishDataAcquisition)
        self.timer.start(0)

    def _finishDataAcquisition (self):
        self.npdatas = np.matrix (self.datas)
        self.qcp.rescaleAxes()
        self.qcp.replot()

    def _genPlot (self):
        self.qcp.clearGraphs()
        for i, elt in enumerate(self.ys):
            graph = self.qcp.addGraph ()
            graph.setData (self.npdatas [:,self.x[1]], self.npdatas [:,elt[1]])
            graph.name = elt[0]
            graph.pen = pens[i%len(pens)]
        self.qcp.xAxis().label = self.x[0]
        self.qcp.rescaleAxes()
        self.qcp.replot()
        # TODO Add a vertical bar at current param along the path.

        return False

class Plugin (QtGui.QDockWidget):
    def __init__ (self, mainWindow, flags = None):
        if flags is None:
            super(Plugin, self).__init__ ("Path graph plugin", mainWindow)
        else:
            super(Plugin, self).__init__ ("Path graph plugin", mainWindow, flags)
        self.setObjectName("Path graph plugin")

        self.main = mainWindow
        self.hppPlugin = self.main.getFromSlot("getHppIIOPurl")
        self.pathPlayer = self.main.getFromSlot("getCurrentPath")
        self.jointgroupcreator = self.main.getFromSlot("requestCreateJointGroup")
        self.comgroupcreator = self.main.getFromSlot("requestCreateComGroup")
        self.velocities = Velocities(self)
        self.jointActions = dict()

        # This avoids having a widget bigger than what it needs. It avoids having
        # a big dock widget and a small osg widget when creating the main osg widget.
        p = Qt.QSizePolicy.Ignored
        self.topWidget = QtGui.QSplitter(Qt.Qt.Horizontal, self)
        self.topWidget.setSizePolicy(Qt.QSizePolicy(p,p))
        self.setWidget (self.topWidget)

        self.leftPane = QtGui.QWidget (self)
        l = QtGui.QVBoxLayout ()
        self.makeLeftPane (l)
        self.leftPane.setLayout (l)

        self.topWidget.addWidget (self.leftPane)

        self.rightPane = QtGui.QWidget (self)
        l = QtGui.QVBoxLayout ()
        self.makeRightPane (l)
        self.rightPane.setLayout (l)
        self.topWidget.addWidget (self.rightPane)

        self.data = DataQCP(self)

    def refreshPlot (self):
        pid = self.pathPlayer.getCurrentPath()
        if pid < 0: return
        dl = self.pathPlayer.lengthBetweenRefresh()
        idxX = self.xselect.currentIndex
        x = (self.xselect.itemText(idxX),
             int(self.xselect.itemData(idxX)) + 1)

        ys = list ()
        for elt in self.yselectcb:
            cb = elt[0]
            if cb.checked:
                ys.append ((str(cb.text), elt[1]+1))

        self.data.selectData(pid, dl, x, ys)
        self.data.acquireData()

    def refreshJointList (self):
        jointNames = self.client.robot.getJointNames ()
        # Left pane
        saLayout = QtGui.QVBoxLayout ()
        formats = ( "%s (%s)", "%s (%s, %i)")

        self.yselectcb = list ()
        rank = 0
        for n in jointNames:
            size = self.client.robot.getJointConfigSize (n)
            if size == 1:
                cb = QtGui.QCheckBox (formats[0] % (n,"q") )
                self.yselectcb.append ((cb, rank))
                saLayout.addWidget (cb)
            else:
                for i in range (size):
                    cb = QtGui.QCheckBox (formats[1] % (n, "q", i))
                    self.yselectcb.append ((cb, rank + i))
                    saLayout.addWidget (cb)
            rank = rank + size
        for type in ("v", "a"):
            saLayout.addSpacing(5)
            for n in jointNames:
                size = self.client.robot.getJointNumberDof (n)
                if size == 1:
                    cb = QtGui.QCheckBox (formats[0] % (n,type) )
                    self.yselectcb.append ((cb, rank))
                    saLayout.addWidget (cb)
                else:
                    for i in range (size):
                        cb = QtGui.QCheckBox (formats[1] % (n,type,i))
                        self.yselectcb.append ((cb, rank + i))
                        saLayout.addWidget (cb)
                rank = rank + size

        saContent = QtGui.QWidget (self)
        saContent.setLayout (saLayout)
        self.scrollArea.setWidget (saContent)

        # Right pane
        self.xselect.clear()
        # time index is 0 and is value is -1
        self.xselect.addItem ("time", -1)
        rank = 0
        for n in jointNames:
            size = self.client.robot.getJointConfigSize (n)
            if size == 1:
                self.xselect.addItem (formats[0] % (n,"q"), rank)
            else:
                for i in range (size):
                    self.xselect.addItem (formats[1] % (n,"q",i), rank + i)
            rank = rank + size
        for type in ("v", "a"):
            for n in jointNames:
                size = self.client.robot.getJointNumberDof (n)
                if size == 1:
                    self.xselect.addItem (formats[0] % (n,type), rank)
                else:
                    for i in range (size):
                        self.xselect.addItem (formats[1] % (n,type,i), rank + i)
                rank = rank + size

    def refreshInterface (self):
        self.refreshJointList()

    def makeLeftPane (self, layout):
        layout.addWidget (QtGui.QLabel ("Select Y data"))
        refresh = QtGui.QPushButton ("Refresh", self)
        refresh.connect (QtCore.SIGNAL("clicked()"), self.refreshPlot)
        layout.addWidget (refresh)

        self.scrollArea = QtGui.QScrollArea (self)
        layout.addWidget (self.scrollArea)

    def makeRightPane (self, layout):
        from PythonQt.QCustomPlot import QCustomPlot, QCP
        self.qcpWidget = QCustomPlot()
        self.qcpWidget.autoAddPlottableToLegend = True
        self.qcpWidget.setInteraction (QCP.iRangeDrag , True) # iRangeDrap
        self.qcpWidget.setInteraction (QCP.iRangeZoom , True) # iRangeZoom
        self.qcpWidget.setInteraction (QCP.iSelectAxes, True) # iSelectAxes
        self.qcpWidget.legend().visible = True
        layout.addWidget (self.qcpWidget)
        self.qcpWidget.connect (Qt.SIGNAL("mouseDoubleClick(QMouseEvent*)"), self._mouseDoubleClick)
        self.qcpWidget.xAxis().connect (Qt.SIGNAL("selectionChanged(QCPAxis::SelectableParts)"), self._axesSelectionChanged)
        self.qcpWidget.yAxis().connect (Qt.SIGNAL("selectionChanged(QCPAxis::SelectableParts)"), self._axesSelectionChanged)

        self.xselect = QtGui.QComboBox(self)
        layout.addWidget (self.xselect)

        # time index is 0 and is value is -1
        self.xselect.addItem ("time", -1)

    def _mouseDoubleClick (self, event):
        try:
            if self.data.x[1] > 0: return
        except:
            return
        try: # Qt 5
            x = event.localPos().x()
        except: # Qt 4
            x = event.posF().x()
        t = self.qcpWidget.xAxis().pixelToCoord (x)
        self.pathPlayer.setCurrentTime(t)

    def _axesSelectionChanged (self, unused_parts):
        xAxis = self.qcpWidget.xAxis()
        yAxis = self.qcpWidget.yAxis()
        x = (xAxis.selectedParts != 0)
        y = (yAxis.selectedParts != 0)
        if not x and not y:
          self.qcpWidget.axisRect().setRangeZoomAxes(xAxis, yAxis)
        elif x:
          self.qcpWidget.axisRect().setRangeZoomAxes(xAxis, None)
        elif y:
          self.qcpWidget.axisRect().setRangeZoomAxes(None, yAxis)

    def resetConnection(self):
        self.client = Client(url= str(self.hppPlugin.getHppIIOPurl()))
        self.gui = GuiClient()

    def getJointActions(self, name):
        if name not in self.jointActions:
            self.jointActions[name] = (JointAction ("Show/Hide joint &velocity", name, self.velocities, self),)
        return self.jointActions[name]
