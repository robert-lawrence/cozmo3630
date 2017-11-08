from __future__ import absolute_import

import threading
import time

from grid import CozGrid
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *
import json
import sys

# map file
Map_filename = "map_test.json"

"""
Auto-grader rubric
Total score = 100, two stage:
1.  Build tracking:
    If the filter can build tracking and output correct robot transformation
    within error tolerance anytime in 100 steps, give 50 points
2.  Maintain tracking:
    let the filter run 100 steps, give score
    score = correct pose percentage / 100 * 50
          = #correct pose / #total pose * 50
"""
# number of steps to allow build tracking
Steps_build_tracking = 100
# number of steps must have stable tracking
Steps_stable_tracking = 100
# translational error allowed
Err_trans = 1.0
# orientation error allowed in degree
Err_rot = 15


"""
Here we give 5 example circles you can test, just uncomment the one you like
But note that in real grading we will use *another* 5 different circles,
so don't try to hack anything!
"""
# example circle 1
Robot_init_pose = (6, 3, 0)
Dh_circular = 10
Robot_speed = 0.5

# example circle 2
# Robot_init_pose = (6, 1, 0)
# Dh_circular = 6
# Robot_speed = 0.5

# example circle 3
#  Robot_init_pose = (5, 4, 0)
#  Dh_circular = 20
#  Robot_speed = 0.3

# example circle 4
# Robot_init_pose = (3, 2, 0)
# Dh_circular = 20
# Robot_speed = 0.3

# example circle 5
# Robot_init_pose = (9, 9, 180)
# Dh_circular = 15
# Robot_speed = 0.5


# move robot circular
# if in collision throw error
def move_robot_circular(robot, rot1, trans, rot2, grid):
    if robot.check_collsion(rot1, trans, rot2, grid):
        raise ValueError('Robot collision')
    else:
        odom_prev = [robot.x, robot.y, robot.h]
        robot.move(rot1, trans, rot2)
        odom_cur = [robot.x, robot.y, robot.h]
    return[odom_prev, odom_cur]

class ParticleFilter:

    def __init__(self, particles, robbie, grid):
        self.particles = particles
        self.robbie = robbie
        self.grid = grid

    def update(self, Dh_circular, Robot_speed):

        # ---------- Move Robot ----------
        odom = add_odometry_noise(move_robot_circular(self.robbie, Dh_circular / 2, Robot_speed, Dh_circular / 2, self.grid), \
            heading_sigma=ODOM_HEAD_SIGMA, trans_sigma=ODOM_TRANS_SIGMA)


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

def grade(Robot_init_pose, Dh_circular, Robot_speed):
    grid = CozGrid(Map_filename)
    
    # initial distribution assigns each particle an equal probability
    particles = Particle.create_random(PARTICLE_COUNT, grid)
    robbie = Robot(Robot_init_pose[0], Robot_init_pose[1], Robot_init_pose[2])
    particlefilter = ParticleFilter(particles, robbie, grid)

    score = 0

    # 1. steps to build tracking
    steps_built_track = 9999
    for i in range(0, Steps_build_tracking):

        est_pose = particlefilter.update(Dh_circular, Robot_speed)
        
        if grid_distance(est_pose[0], est_pose[1], robbie.x, robbie.y) < Err_trans \
                and math.fabs(diff_heading_deg(est_pose[2], robbie.h)) < Err_rot \
                and i+1 < steps_built_track:
            steps_built_track = i+1

    #print("steps_built_track =", steps_built_track)
    if steps_built_track < 50:
        score = 50
    elif steps_built_track < 100:
        score = 100-steps_built_track
    else:
        score = 0

    print("\nPhrase 1")
    print("Number of steps to build track :", steps_built_track, "/", Steps_build_tracking)
    acc_err_trans, acc_err_rot = 0.0, 0.0
    max_err_trans, max_err_rot = 0.0, 0.0
    step_track = 0

    # 2. test tracking

    for i in range(0, Steps_stable_tracking):

        est_pose = particlefilter.update(Dh_circular, Robot_speed)

        err_trans = grid_distance(est_pose[0], est_pose[1], robbie.x, robbie.y)
        acc_err_trans += err_trans
        if max_err_trans < err_trans:
            max_err_trans = err_trans

        err_rot = math.fabs(diff_heading_deg(est_pose[2], robbie.h))
        acc_err_rot += err_rot
        if max_err_rot < err_rot:
            max_err_rot = err_rot

        if grid_distance(est_pose[0], est_pose[1], robbie.x, robbie.y) < Err_trans \
                and math.fabs(diff_heading_deg(est_pose[2], robbie.h)) < Err_rot:
            step_track += 1

    if step_track > 90:
        score += 50
    elif step_track > 40:
        score += step_track - 40
    else:
        score += 0

    print("\nPhrase 2")
    print("Number of steps error in threshold :", step_track, "/", Steps_stable_tracking)
    print("Average translational error :{:.2f}".format(acc_err_trans / Steps_stable_tracking))
    print("Average rotational error :{:.2f} deg".format(acc_err_rot / Steps_stable_tracking))
    print("Max translational error :{:.2f}".format(max_err_trans))
    print("Max rotational error :{:.2f} deg".format(max_err_rot))

    print("\nexample score =", score)
    return score

if __name__ == "__main__":
    tests = {}
    if len(sys.argv) > 1:
        try:
            with open(sys.argv[1]) as testfile:
                tests = json.loads(testfile.read())
        except:
            print("Error opening test file, please check filename and json format")
            raise
    else:
        print("correct usage: python3 autograder.py <testfile>")
        exit()
    score = 0.
    for key, value in tests.items():
        print("########################################")
        print("#########grading {}##############".format(key))
        print("########################################")
        score += grade(**value)

    print("\n\n\nFinal score is: {}".format(score/len(tests)))
