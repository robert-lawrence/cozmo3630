import random
import math

from utils import *
from grid import *
from setting import *


""" Particle class (base class for robot)
    A class for particle, each particle contains x, y, and heading information
"""
class Particle(object):

    # data members
    # x = "X coordinate in world frame"
    # y = "Y coordinate in world frame"
    # h = "Heading angle in world frame in degree. h = 0 when robot's head (camera) points to positive X"

    # functions members

    def __init__(self, x, y, heading=None):
        if heading is None:
            heading = random.uniform(0, 360)
        self.x = x
        self.y = y
        self.h = heading

    def __repr__(self):
        return "(x = %f, y = %f, heading = %f deg)" % (self.x, self.y, self.h)

    @property
    def xy(self):
        return self.x, self.y

    @property
    def xyh(self):
        return self.x, self.y, self.h

    @classmethod
    # create some random particles
    def create_random(cls, count, grid):
        return [cls(*grid.random_free_place()) for _ in range(0, count)]

    def move(self, rot1, trans, rot2):
        """ Rotate the particle with rot1 degree and drive forward trans, and then rotate rot2 degree
            Note that the robot *turn first, then drive forward, followed by final turn*

            Arguments:
            rot1 -- degree to turn, turn left is positive
            trans -- distance to drive forward (unit in grid)
            rot2 -- degree to turn, turn left is positive

            No return
        """
        self.h = self.h + rot1
        dx = math.cos(math.radians(self.h)) * trans
        dy = math.sin(math.radians(self.h)) * trans
        self.x += dx
        self.y += dy
        self.h = self.h + rot2

    def read_markers(self, grid):
        """ Helper function to simulate markers measurements by robot's camera
            Only markers in robot's camera view (in FOV) will be in the list

            Arguments:
            grid -- map grid with marker information

            Return: robot detected marker list, each marker has format:
                    measured_marker_list[i] = (rx, ry, rh)
                    rx -- marker's relative X coordinate in robot's frame
                    ry -- marker's relative Y coordinate in robot's frame
                    rh -- marker's relative heading in robot's frame, in degree
        """
        marker_list = []
        for marker in grid.markers:
            m_x, m_y, m_h = parse_marker_info(marker[0], marker[1], marker[2])
            # rotate marker into robot frame
            mr_x, mr_y = rotate_point(m_x - self.x, m_y - self.y, -self.h)
            if math.fabs(math.degrees(math.atan2(mr_y, mr_x))) < ROBOT_CAMERA_FOV_DEG / 2.0:
                mr_h = m_h - self.h
                marker_list.append((mr_x, mr_y, mr_h))
        return marker_list



""" Robot class
    A class for robot, contains same x, y, and heading information as particles
    but with some more utilities for robot motion / collision checking
"""
class Robot(Particle):

    def __init__(self, grid):
        super(Robot, self).__init__(*grid.random_free_place())

    def __init__(self, x, y, h):
        super(Robot, self).__init__(x, y, h)

    def __repr__(self):
        return "(x = %f, y = %f, heading = %f deg)" % (self.x, self.y, self.h)

    # return a random robot heading angle
    def chose_random_heading(self):
        return random.uniform(0, 360)

    def read_markers(self, grid):
        """ Helper function to simulate markers measurements by robot's camera
            Only markers in robot's camera view (in FOV) will be in the list

            Arguments:
            grid -- map grid with marker information

            Return: robot detected marker list, each marker has format:
                    measured_marker_list[i] = (rx, ry, rh)
                    rx -- marker's relative X coordinate in robot's frame
                    ry -- marker's relative Y coordinate in robot's frame
                    rh -- marker's relative heading in robot's frame, in degree
        """
        return super(Robot, self).read_markers(grid)

    def move(self, rot1, trans, rot2):
        """ Rotate the robot with rot1 degree and drive forward trans, and then rotate rot2 degree
            Note that the robot *turn first, then drive forward, followed by final turn*

            Arguments:
            rot1 -- degree to turn, turn left is positive
            trans -- distance to drive forward (unit in grid)
            rot2 -- degree to turn, turn left is positive

            No return
        """
        return super(Robot, self).move(rot1, trans, rot2)

    def check_collsion(self, rot1, trans, rot2, grid):
        """ Check whether moving the robot will cause collision.
            Note this function will *not* move the robot

            Arguments:
            rot1 -- degree to turn, turn left is positive
            trans -- distance to drive forward (unit in grid)
            rot2 -- degree to turn, turn left is positive
        
            Return: True if will cause collision, False if will not be a collision
        """
        h = self.h + rot1
        dx = math.cos(math.radians(h)) * trans
        dy = math.sin(math.radians(h)) * trans
        if grid.is_free(self.x+dx, self.y+dy):
            return False
        return True