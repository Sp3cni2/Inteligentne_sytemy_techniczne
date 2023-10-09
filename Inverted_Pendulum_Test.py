import os
import logging
import time
from datetime import datetime
import sys
from typing import List

import pygame

import pymunk
import pymunk.pygame_util
from pymunk.vec2d import Vec2d

import object
import math

from controller.controllers import Controller

# GLOBAL scope of Analysis module:
# pretty much a place for logger

# create logger
logger = logging.getLogger('MAIN')
logger.setLevel(logging.INFO)

# create console handler and set level to debug
ch = logging.StreamHandler()
ch.setLevel(logging.INFO)

# create file handler with filename and set level to debug
fh = logging.FileHandler(os.path.join('./logs/')+datetime.today().strftime("%Y-%m-%d")+__file__.split("\\")[-1].rstrip(".py")+'.log')
fh.setLevel(logging.DEBUG)

# create formatter
formatter = logging.Formatter('%(asctime)s - %(levelname)s: %(message)s')

# add formatter to ch
ch.setFormatter(formatter)
fh.setFormatter(formatter)

# add ch to logger
logger.addHandler(ch)
logger.addHandler(fh)

def spawn_cart(a=345,b=520,width=100,height=20,radius=10, **kwargs):

    #vs = [(0,0),(),(),()]
    p1vect=Vec2d(50,20)
    p2vect=Vec2d(-50,20)

    box_body = pymunk.Body(body_type=pymunk.Body.DYNAMIC)
    box_body.position = (a, b)

    wheel_body1 = pymunk.Body(body_type=pymunk.Body.DYNAMIC)
    wheel_body1.position = (a+int(width/2), b+int(height/2))

    wheel_body2 = pymunk.Body(body_type=pymunk.Body.DYNAMIC)
    wheel_body2.position = (a-int(width/2), b+int(height/2))

    wheel_1 = pymunk.Circle(wheel_body1, radius=radius)
    wheel_1.density=0.88
    wheel_1.friction=1.05

    wheel_2 = pymunk.Circle(wheel_body2, radius=radius)
    wheel_2.density=0.88
    wheel_2.friction=1.05

    box = pymunk.Poly.create_box(box_body,size=(width,height))
    box.density=0.88

    pole_width, pole_height = [10,200]
    polevect=Vec2d(0,-height//2 -5)
    pole_pivot_pos = (0,pole_height/2-5)
    pole_body = pymunk.Body(body_type=pymunk.Body.DYNAMIC)
    pole_body.position = (a - pole_width/2, b - pole_pivot_pos[1])
    pole = pymunk.Poly.create_box(pole_body,size=(pole_width, pole_height))
    pole.density=1.18
    pole_body.center_of_gravity = Vec2d(0,pole_height//2)

    shape_filter = pymunk.ShapeFilter(group=2)
    wheel_1.filter = shape_filter
    wheel_2.filter = shape_filter
    box.filter = shape_filter
    #pole.filter = shape_filter

    pivot_1 = pymunk.PivotJoint(box_body,wheel_body1,p1vect,(0,0))
    pivot_1.collide_bodies=False

    pivot_2 = pymunk.PivotJoint(box_body,wheel_body2,p2vect,(0,0))
    pivot_2.collide_bodies=False

    pivot_pole = pymunk.PivotJoint(box_body,pole_body,polevect,pole_pivot_pos)
    pivot_pole.collide_bodies=False

    joint_limit = math.radians(15)
    limit_joint = pymunk.RotaryLimitJoint(box_body, pole_body, -joint_limit, joint_limit)
    limit_joint.collide_bodies = False


    motor_wheel_1 = pymunk.SimpleMotor(wheel_body1, box_body, 0.0)
    motor_wheel_2 = pymunk.SimpleMotor(wheel_body2, box_body, 0.0)

    bodies=[wheel_body1,wheel_body2,box_body,pole_body]
    shapes=[wheel_1,wheel_2,box,pole]
    joints=[pivot_1,pivot_2,pivot_pole,limit_joint]
    motors=[motor_wheel_1,motor_wheel_2]
    return bodies,shapes,joints,motors


def main(*args,**kwargs):

    width, height = 690, 600

    ### PyGame init
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    clock = pygame.time.Clock()
    running = True
    font = pygame.font.SysFont("Arial", 16)
    logger.info("initialized PyGame")


    ### Physics stuff
    space = pymunk.Space()
    space.gravity = 0, 1000
    draw_options = pymunk.pygame_util.DrawOptions(screen)
    logger.info("initialized PyMunk")


    # walls
    static: List[pymunk.Shape] = [
        pymunk.Segment(space.static_body, (680, 550), (680, 500), 2),
        pymunk.Segment(space.static_body, (10, 550), (10, 500), 2),
        pymunk.Segment(space.static_body, (10, 550), (680, 550), 2),
    ]

    bodies,shapes,joints,motors=spawn_cart()

    for s in static:
        s.friction = 0.00
        s.elasticity = 0.0
        s.group = 1
    static[-1].friction=1.0
    space.add(*static,*bodies,*shapes,*joints,*motors)

    rotationRate=2.0

    fps = 60
    dt = 1.0 / fps

    ctrl = Controller(bodies[-1],logger)

    start_time = 0
    while running:

        for event in pygame.event.get():
            if (event.type == pygame.QUIT or event.type == pygame.KEYDOWN and (event.key in [pygame.K_ESCAPE, pygame.K_q])): running = False

            # elif event.type == pygame.KEYDOWN and event.key == pygame.K_w:
            #     ctrl.P+=0.1
            #
            # elif event.type == pygame.KEYUP and event.key == pygame.K_s:
            #     motors[0].rate = 0.0
            #     motors[1].rate = 0.0
            #
            # elif event.type == pygame.KEYDOWN and event.key == pygame.K_e:
            #     motors[0].rate = -rotationRate
            #     motors[1].rate = -rotationRate
            #
            # elif event.type == pygame.KEYUP and event.key == pygame.K_d:
            #     motors[0].rate = 0.0
            #     motors[1].rate = 0.0
            # elif event.type == pygame.KEYUP and event.key == pygame.K_r:
            #     rotationRate+=1.0
            # elif event.type == pygame.KEYUP and event.key == pygame.K_f:
            #     rotationRate-=1.0

        ctrl.PID(0.0,80.0,2.50,0.005,dt)
        motors[0].rate,motors[1].rate = ctrl.control_vector,ctrl.control_vector

        ### Clear screen
        screen.fill(pygame.Color("black"))
        ### Draw stuff
        space.debug_draw(draw_options)
        # Info and flip screen
        screen.blit(font.render("fps: " + str(round(clock.get_fps(),1)), True, pygame.Color("white")),(0, 0),)
        screen.blit(font.render("error: " + str(round(ctrl.current_error,2)), True, pygame.Color("white")),(0, 16),)
        screen.blit(font.render("control_force: " + str(round(ctrl.control_vector,2)), True, pygame.Color("white")),(80, 16),)
        screen.blit(font.render("angle of pendulum: " + str(round(math.degrees(bodies[-1].angle),2))+", ang vel:"+ str(round(bodies[-1].angular_velocity,2)), True, pygame.Color("white")),(0, 32),)
        screen.blit(font.render("W and S to change cart velocity",True,pygame.Color("darkgrey"),),(5, height - 50),)
        screen.blit(font.render("A and D to move the cart",True,pygame.Color("darkgrey"),),(5, height - 35),)
        screen.blit(font.render("Press ESC or Q to quit", True, pygame.Color("darkgrey")),(5, height - 20),)
        pygame.display.flip()
        ### Update physics
        space.step(dt)
        clock.tick(fps)

if __name__ == '__main__':

    sys.exit(main())