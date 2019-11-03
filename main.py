import time
from datetime import datetime

import pyglet
import pymunk
from pymunk.pyglet_util import DrawOptions
from pymunk.constraint import PivotJoint, PinJoint
import numpy as np
import math

window = pyglet.window.Window(1280, 720, "Ball and Beam Balance", resizable=False)
options = DrawOptions()

time_scale = 10
ball_lift_counter = 0

space = pymunk.Space()
space.gravity = 0, -9.8

# Rigid Wall
wall_body = pymunk.Body(body_type=pymunk.Body.STATIC)
wall_body.position = 100, 360
wall_shape = pymunk.Poly.create_box(wall_body, size=(20, 1440))

# Beam Shape and body
beam_moment = pymunk.moment_for_box(mass=0.8, size=(500, 10))
beam_body = pymunk.Body(0.8, beam_moment)
beam_body.position = 365, 480
beam_shape = pymunk.Poly.create_box(beam_body, size=(500, 10))
beam_shape.friction = 0.8

# Wall and Beam Joint
w_b_joint = pymunk.PivotJoint(wall_body, beam_body, (115, 480))

# Beam Support Shape and body
support_moment = pymunk.moment_for_box(mass=0.3, size=(4, 300))
support_body = pymunk.Body(0.3, support_moment)
support_body.position = 613, 335
support_shape = pymunk.Poly.create_box(support_body, size=(4, 300))

# Support and Beam Joint
s_b_joint = PivotJoint(beam_body, support_body, (613, 480))
s_b_joint.collide_bodies = False

# Armature Body and shape
armature_moment = pymunk.moment_for_box(mass=0.3, size=(200, 4))
armature_body = pymunk.Body(0.3, armature_moment, body_type=pymunk.Body.KINEMATIC)
armature_body.position = 513, 180
armature_shape = pymunk.Poly.create_box(armature_body, size=(200, 4))
armature_body.angle = -math.pi / 10

# # Armature and support joint
a_s_joint = PivotJoint(armature_body, support_body, (100, 0), (0, -150))
a_s_joint.collide_bodies = False

# Ball Body and shape
ball_moment = pymunk.moment_for_circle(0.027, 0, 20)
ball_body = pymunk.Body(0.027, 0.72)
ball_body.position = 165, 485
ball_shape = pymunk.Circle(ball_body, 20)
ball_shape.friction = 1.2

# Control Parameters
L = np.array([[0.0735], [0.2940]])
K = np.array([[6.8027, 5.1020]])

A = np.array([[0, 1], [0, 0]], dtype='float')
B = np.array([[0], [1.1760]], dtype='float')
C = np.array([[1, 0]], dtype='float')
X = np.array([[0], [0]], dtype='float')
U = np.array([[0]])

Integrator = [0] * 100
Integrator_c = 0


def ball_pos():
    pos_x = ball_body.position.x
    pos_y = ball_body.position.y

    beam_angle = beam_body.angle

    beam_x = beam_body.position.x
    beam_y = beam_body.position.y
    beam_edge_x = math.cos(beam_angle) * 250 + beam_x
    beam_edge_y = math.sin(beam_angle) * 250 + beam_y

    rect_x = -math.sin(beam_angle) * 25 + beam_edge_x
    rect_y = math.cos(beam_angle) * 25 + beam_edge_x

    d = math.sqrt(math.pow(rect_x - pos_x, 2) + math.pow(rect_y - pos_y, 2)) / 1000

    z = ball_shape.shapes_collide(beam_shape)

    end_time = time.localtime()

    global time_scale
    global ball_lift_counter
    if len(z.points) == 0 and time.mktime(end_time) - time.mktime(start_time) > 2:
        ball_lift_counter = ball_lift_counter + 1
    else:
        ball_lift_counter = 0
    if ball_lift_counter > 40:
        time_scale = 0
    return d


def get_input_angle(dt=0):
    global X, U, Integrator_c
    x = ball_pos()

    X[0, 0] = x
    CX = np.matmul(C, X)
    LA = -CX + x

    LLA = np.matmul(L, LA)
    AX = np.matmul(A, X)
    BU = np.matmul(B, U)
    Integrator.pop(0)

    Integrator.append(+LLA[1, 0] + AX[1, 0] + BU[1, 0])
    X[1, 0] = sum(Integrator) / 100
    KX = np.matmul(K, X)
    U = 2.5-KX


start_time = time.localtime()

space.add(wall_shape, wall_body,
          beam_body, beam_shape, w_b_joint,
          support_body, support_shape, s_b_joint,
          armature_body, armature_shape, a_s_joint,
          ball_body, ball_shape)


@window.event
def on_draw():
    window.clear()
    space.debug_draw(options)


change = math.pi / 60


def update(dt):
    global time_scale
    space.step(time_scale * dt)
    get_input_angle(0)
    armature_angle = U[0, 0]
    if armature_angle > 1.3962:
        armature_angle = 1.3962
    elif armature_angle < -1.3962:
        armature_angle = -1.3962
    armature_body.angle = armature_angle
    # global change
    #
    # if math.fabs(armature_angle) >= 1.3962:
    #     change = change * -1
    # armature_body.angle = armature_angle + change


if __name__ == '__main__':
    pyglet.clock.schedule_interval(update, 1 / 60)
    pyglet.app.run()