from __future__ import absolute_import

import threading
import time

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *

#  Map_filename = "map_arena.json"
Map_filename = "map_test.json"

Use_GUI = True

Robot_speed = 0.5
Robot_init_pose = (6, 3, 0)
Move_circular = True
Dh_circular = 10

# move robot just forward
# if in collision, bouncing to a random direction which is collision free
def move_robot_forward(robot, rot1, trans, rot2, grid):
    odom_prev = [robot.x, robot.y, robot.h]
    while True:
        if not robot.check_collsion(rot1, trans, rot2, grid):
            robot.move(rot1, trans, rot2)
            odom_cur = [robot.x, robot.y, robot.h]
            break
        # Bumped into something, chose random new direction
        robot.h = robot.chose_random_heading()
    return [odom_prev, odom_cur]


# move robot circular
# if in collision throw error
# This is the motion mode auto-grader will use
def move_robot_circular(robot, rot1, trans, rot2, grid):
    if robot.check_collsion(rot1, trans, rot2, grid):
        raise ValueError('Robot collision')
    else:
        odom_prev = [robot.x, robot.y, robot.h]
        robot.move(rot1, trans, rot2)
        odom_cur = [robot.x, robot.y, robot.h]
    return [odom_prev, odom_cur]


class ParticleFilter:

    def __init__(self, particles, robbie, grid):
        self.particles = particles
        self.robbie = robbie
        self.grid = grid

    def update(self):
        # ---------- Move Robot ----------
        if Move_circular:
            odom = add_odometry_noise(move_robot_circular(self.robbie, Dh_circular/2, Robot_speed, Dh_circular/2, self.grid), \
                heading_sigma=ODOM_HEAD_SIGMA, trans_sigma=ODOM_TRANS_SIGMA)
        else:
            odom = add_odometry_noise(move_robot_forward(self.robbie, 0, Robot_speed, 0, self.grid), \
                heading_sigma=ODOM_HEAD_SIGMA, trans_sigma=ODOM_TRANS_SIGMA)

        print('\nrobot :', self.robbie)
        print('odometry measured: odom_prev =', odom[0], ', odom_cur =', odom[1])


        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)


        # ---------- Find markers in camera ----------
        # read markers
        r_marker_list_raw = self.robbie.read_markers(self.grid)
        #print("r_marker_list :", r_marker_list)

        # add noise to marker list
        r_marker_list = []
        for m in r_marker_list_raw:
            r_marker_list.append(add_marker_measurement_noise(m, \
                trans_sigma=MARKER_TRANS_SIGMA, rot_sigma=MARKER_ROT_SIGMA))


        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)


        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)


class ParticleFilterThread(threading.Thread):
    
    def __init__(self, particle_filter, gui):
        threading.Thread.__init__(self, daemon=True)
        self.filter = particle_filter
        self.gui = gui

    def run(self):
        while True:
            estimated = self.filter.update()
            self.gui.show_particles(self.filter.particles)
            self.gui.show_mean(estimated[0], estimated[1], estimated[2], estimated[3])
            self.gui.show_robot(self.filter.robbie)
            self.gui.updated.set()


if __name__ == "__main__":
    grid = CozGrid(Map_filename)
    
    # initial distribution assigns each particle an equal probability
    particles = Particle.create_random(PARTICLE_COUNT, grid)
    robbie = Robot(Robot_init_pose[0], Robot_init_pose[1], Robot_init_pose[2])
    particlefilter = ParticleFilter(particles, robbie, grid)

    if Use_GUI:
        gui = GUIWindow(grid)
        filter_thread = ParticleFilterThread(particlefilter, gui)
        filter_thread.start()
        gui.start()
    else:
        while True:
            particlefilter.update()

