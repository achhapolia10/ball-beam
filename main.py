import math

import pyglet
import pymunk
from pymunk.pyglet_util import DrawOptions
from pymunk.constraint import PivotJoint, PinJoint

window = pyglet.window.Window(1280, 720, "Ball and Beam Balance", resizable=False)
options = DrawOptions()

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
ball_body.position = 365, 495
ball_shape = pymunk.Circle(ball_body, 20)
ball_shape.friction = 0.8

space.add(wall_shape, wall_body,
          beam_body, beam_shape, w_b_joint,
          support_body, support_shape, s_b_joint,
          armature_body, armature_shape, a_s_joint,
          ball_body, ball_shape)


@window.event
def on_draw():
    window.clear()
    space.debug_draw(options)


change = math.pi / 100


def update(dt):
    space.step(10 * dt)
    armature_angle = armature_body.angle
    global change

    if math.fabs(armature_angle) >= 1.3962:
        change = change * -1
    armature_body.angle = armature_angle + change


if __name__ == '__main__':
    print(ball_moment)
    pyglet.clock.schedule_interval(update, 1 / 60)
    pyglet.app.run()
