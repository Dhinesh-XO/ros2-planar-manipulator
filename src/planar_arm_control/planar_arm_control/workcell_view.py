"""Original primitive-mesh workcell; 3D presentation of the unchanged 2D arm.

No robot meshes, kinematics or control code are copied from Bifrost. Geometry
comes exclusively from the supplied FK. Rendering never generates commands.
"""

from collections import deque
import math

import numpy as np
from PyQt5 import QtGui
import pyqtgraph.opengl as gl
from pyqtgraph.opengl import shaders


def world(xy, depth=0.0):
    return np.array([xy[0], depth, xy[1]+0.8], dtype=float)


def box_mesh():
    vertices = np.array([[-1,-1,-1], [1,-1,-1], [1,1,-1], [-1,1,-1],
                         [-1,-1,1], [1,-1,1], [1,1,1], [-1,1,1]])/2
    faces = np.array([[0,2,1], [0,3,2], [4,5,6], [4,6,7], [0,1,5], [0,5,4],
                      [1,2,6], [1,6,5], [2,3,7], [2,7,6], [3,0,4], [3,4,7]])
    return gl.MeshData(vertexes=vertices, faces=faces)


class WorkcellView(gl.GLViewWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        if 'workcell_lit' not in shaders.ShaderProgram.names:
            shaders.ShaderProgram('workcell_lit', [
                shaders.VertexShader('''
                    varying vec3 n;
                    void main() {
                        n = normalize(gl_NormalMatrix * gl_Normal);
                        gl_FrontColor = gl_Color;
                        gl_BackColor = gl_Color;
                        gl_Position = ftransform();
                    }
                '''),
                shaders.FragmentShader('''
                    varying vec3 n;
                    void main() {
                        float key = max(0.0, dot(normalize(n), normalize(vec3(-0.4, 0.7, 1.0))));
                        float fill = abs(dot(normalize(n), normalize(vec3(1.0, 0.1, 0.3))));
                        gl_FragColor = vec4(gl_Color.rgb * (0.48 + 0.42*key + 0.10*fill), gl_Color.a);
                    }
                ''')])
        self.setBackgroundColor('#101b28')
        self.setMinimumHeight(310)
        self.home_camera()
        self.box = box_mesh()
        self.cylinder = gl.MeshData.cylinder(rows=1, cols=24, radius=[1,1], length=1)
        self.trail = deque(maxlen=400)
        self.last_command = None
        grid = gl.GLGridItem(color=(65,85,101,130))
        grid.setSize(18, 10)
        grid.setSpacing(1, 1)
        grid.translate(0,0,.005)
        self.addItem(grid)
        self.place_box(self.mesh(self.box, (0.13,0.19,0.25,1)), (0,0,-.10), (18,10,.2))
        self.place_box(self.mesh(self.box, (.18,.25,.32,1)), (0,.6,.25), (1.2,1.1,.5))
        self.place_cylinder(self.mesh(self.cylinder, (.28,.36,.43,1)),
                            (0,.6,.5), (0,.6,.8), .36)
        colors = [(.35,.85,.93,1), (1,.74,.41,1), (.75,.61,1,1)]
        self.links = [self.mesh(self.box, color) for color in colors]
        self.inlays = [self.mesh(self.box, (.16,.21,.28,1)) for _ in colors]
        self.hubs = [self.mesh(self.cylinder, (.29,.36,.44,1)) for _ in range(3)]
        self.caps = [self.mesh(self.cylinder, color) for color in colors]
        self.palm = self.mesh(self.box, (.77,.82,.87,1))
        self.tool_bracket = self.mesh(self.box, (.4,.48,.55,1))
        self.fingers = [self.mesh(self.box, (.86,.9,.94,1)) for _ in range(2)]
        self.payload = self.mesh(self.box, (.98,.4,.16,1))
        self.payload.setVisible(False)
        self.planned = gl.GLLinePlotItem(pos=np.zeros((2,3)), color=(.25,.75,.82,.65),
                                         width=1.5, antialias=True)
        self.actual = gl.GLLinePlotItem(pos=np.zeros((2,3)), color=(1,.68,.28,1),
                                        width=2.5, antialias=True)
        self.targets = gl.GLScatterPlotItem(pos=np.zeros((2,3)), size=10,
                     color=np.array([[1,.35,.35,1], [1,.75,.3,1]]), pxMode=True)
        for item in (self.planned, self.actual, self.targets):
            self.addItem(item)
        self.targets.setVisible(False)
        self.stations = []
        for label, color in [('PICK', (.12,.55,.60,1)), ('PLACE', (.54,.36,.72,1))]:
            parts = [self.mesh(self.box, (.21,.28,.34,1)), self.mesh(self.box, color)]
            text = gl.GLTextItem(pos=(0,0,0), text=label, color=(210,225,238,255),
                                 font=QtGui.QFont('Sans', 10))
            self.addItem(text)
            self.stations.append((*parts, text))
        self.update_stations((4,2), (-3,3))
        self.addItem(gl.GLTextItem(pos=(-6.8,1,.1), text='PLANAR WORKCELL  /  y ≥ 0',
                                   color=(125,158,182,255), font=QtGui.QFont('Sans', 9)))

    def home_camera(self):
        self.opts['center'] = QtGui.QVector3D(0, 0, 3.0)
        self.setCameraPosition(distance=17.5, elevation=19, azimuth=-78)

    def front_camera(self):
        self.opts['center'] = QtGui.QVector3D(0, 0, 3.3)
        self.setCameraPosition(distance=17, elevation=0, azimuth=-90)

    def mesh(self, data, color):
        item = gl.GLMeshItem(meshdata=data, color=color, smooth=False,
                             shader='workcell_lit', glOptions='opaque')
        self.addItem(item)
        return item

    @staticmethod
    def place_box(item, center, size, angle=0.0):
        item.resetTransform()
        item.scale(*size)
        item.rotate(-math.degrees(angle), 0, 1, 0)
        item.translate(*center)

    @staticmethod
    def place_cylinder(item, start, end, radius):
        start, end = np.asarray(start, dtype=float), np.asarray(end, dtype=float)
        delta = end-start
        length = np.linalg.norm(delta)
        direction = delta/max(length, 1e-9)
        axis = np.cross([0,0,1], direction)
        angle = math.degrees(math.acos(float(np.clip(direction[2], -1, 1))))
        if np.linalg.norm(axis) < 1e-8:
            axis = np.array([1,0,0])
        item.resetTransform()
        item.scale(radius, radius, float(length))
        item.rotate(angle, *axis)
        item.translate(*start)

    def update_stations(self, pick, place):
        for xy, (column, platform, label) in zip((pick, place), self.stations):
            height = xy[1]+.8-.175
            self.place_box(column, (xy[0],.65,height/2), (.38,.38,height))
            self.place_box(platform, (xy[0],.35,height-.08), (1.1,.95,.16))
            label.setData(pos=(xy[0]-.65,.8,height+.4),
                          text=f'{"PICK" if column is self.stations[0][0] else "PLACE"} ({xy[0]:g}, {xy[1]:g})')

    def set_path(self, message):
        points = [world((pose.pose.position.x, pose.pose.position.y), .025)
                  for pose in message.poses]
        self.planned.setVisible(len(points) > 1)
        if len(points) > 1:
            self.planned.setData(pos=np.asarray(points))

    def update_arm(self, points, q, status):
        points = np.asarray([world(point, .6) for point in points])
        angles = np.cumsum(q)
        for i, angle in enumerate(angles):
            center = (points[i]+points[i+1])/2
            length = np.linalg.norm(points[i+1]-points[i])
            self.place_box(self.links[i], center, (length-.12,.23,.23), angle)
            self.place_box(self.inlays[i], center+[0,-.125,0], (length*.64,.018,.105), angle)
            self.place_cylinder(self.hubs[i], points[i]+[0,-.21,0], points[i]+[0,.21,0], .24-i*.025)
            self.place_cylinder(self.caps[i], points[i]+[0,-.245,0], points[i]+[0,-.21,0], .14-i*.015)
        direction = np.array([math.cos(angles[-1]), 0, math.sin(angles[-1])])
        self.place_box(self.tool_bracket, points[-1]-[0,.3,0], (.14,.6,.14), angles[-1])
        tip = points[-1]-[0,.6,0]
        opening = status.gripper_opening if status else 1.0
        self.place_box(self.palm, tip-direction*.36, (.18,.76,.23), angles[-1])
        for sign, finger in zip((-1,1), self.fingers):
            self.place_box(finger, tip-direction*.13+[0,sign*(.205+.13*opening),0],
                           (.44,.06,.13), angles[-1])
        if status is not None:
            if status.command_id != self.last_command:
                self.trail.clear()
                self.actual.setVisible(False)
                self.last_command = status.command_id
            self.update_stations((status.pick_target.x,status.pick_target.y),
                                 (status.place_target.x,status.place_target.y))
            self.payload.setVisible(status.object_visible)
            self.place_box(self.payload, world((status.object_position.x,status.object_position.y)),
                           (.35,.35,.35), status.object_angle)
            self.targets.setVisible(bool(status.command_id))
            self.targets.setData(pos=np.array([
                world((status.requested_target.x,status.requested_target.y),-.07),
                world((status.resolved_target.x,status.resolved_target.y),-.07)]))
        if not self.trail or np.linalg.norm(tip-self.trail[-1]) > .015:
            self.trail.append(tip)
        if len(self.trail) > 1:
            self.actual.setVisible(True)
            self.actual.setData(pos=np.asarray(self.trail))
