#!/usr/bin/env python3

# Lab 7 Robert Smith and Mitchell Myers

import cv2
import cozmo
import numpy as np
from numpy.linalg import inv
import threading
import time
import math

from ar_markers.hamming.detect import detect_markers

from grid import CozGrid
from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *
from cozmo.util import distance_mm, speed_mmps, degrees, distance_inches, Pose

# camera params
camK = np.matrix([[295, 0, 160], [0, 295, 120], [0, 0, 1]], dtype='float32')

#marker size in inches
marker_size = 3.5

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "map_arena.json"


async def image_processing(robot):

    global camK, marker_size

    event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # convert camera image to opencv format
    opencv_image = np.asarray(event.image)
    
    # detect markers
    markers = detect_markers(opencv_image, marker_size, camK)
    
    # show markers
    for marker in markers:
        marker.highlite_marker(opencv_image, draw_frame=True, camK=camK)
        #print("ID =", marker.id);
        #print(marker.contours);
    cv2.imshow("Markers", opencv_image)

    return markers

#calculate marker pose
def cvt_2Dmarker_measurements(ar_markers):
    
    marker2d_list = []
    
    for m in ar_markers:
        R_1_2, J = cv2.Rodrigues(m.rvec)
        R_1_1p = np.matrix([[0,0,1], [0,-1,0], [1,0,0]])
        R_2_2p = np.matrix([[0,-1,0], [0,0,-1], [1,0,0]])
        R_2p_1p = np.matmul(np.matmul(inv(R_2_2p), inv(R_1_2)), R_1_1p)
        #print('\n', R_2p_1p)
        yaw = -math.atan2(R_2p_1p[2,0], R_2p_1p[0,0])
        
        x, y = m.tvec[2][0] + 0.5, -m.tvec[0][0]
        # print('x =', x, 'y =', y,'theta =', yaw)
        
        # remove any duplate markers
        dup_thresh = 2.0
        find_dup = False
        for m2d in marker2d_list:
            if grid_distance(m2d[0], m2d[1], x, y) < dup_thresh:
                find_dup = True
                break
        if not find_dup:
            marker2d_list.append((x,y,math.degrees(yaw)))

    return marker2d_list


#compute robot odometry based on past and current pose
def compute_odometry(curr_pose, cvt_inch=True):
    global last_pose
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees

    if cvt_inch:
        last_x, last_y = last_x / 25.6, last_y / 25.6
        curr_x, curr_y = curr_x / 25.6, curr_y / 25.6

    return [[last_x, last_y, last_h],[curr_x, curr_y, curr_h]]

#particle filter functionality
class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)

# pf = ParticleFilter(grid)
# state = "In motion"
#
# async def is_kidnapped(robot: cozmo.robot.Robot):
#     global pf, state
#     while True:
#         if robot.is_picked_up:
#             await robot.say_text("Hey put me down!!").wait_for_completed()
#             pf = ParticleFilter(grid)
#             time.sleep(5)
#             state = "In motion"

async def run(robot: cozmo.robot.Robot):
    global last_pose
    global grid, gui
    # global pf, state

    # start streaming
    robot.camera.image_stream_enabled = True

    #start particle filter
    pf = ParticleFilter(grid)
    await robot.set_head_angle(degrees(0)).wait_for_completed()
    await robot.set_lift_height(0).wait_for_completed()
    state = "unknown"
    role = ""
    storage_cube_mult = 1
    ############################################################################
    ######################### YOUR CODE HERE####################################
    while True:
        robo_odom = compute_odometry(robot.pose)
        vis_markers = await image_processing(robot)
        markers_2d = cvt_2Dmarker_measurements(vis_markers)
        m_x, m_y, m_h, m_confident = ParticleFilter.update(pf, robo_odom, markers_2d)
        gui.show_mean(m_x, m_y, m_h, m_confident)
        gui.show_particles(pf.particles)
        robbie = Robot(robot.pose.position.x, robot.pose.position.y, robot.pose_angle.degrees)
        gui.show_robot(robbie)
        gui.updated.set()
        if m_confident:
            state = "known"
            #TODO: relax the constraints on m_confident
        if state == "known":
            if role == "":
                if m_x < 13:
                    role = "pickup"
                else:
                    role = "storage"
            print(role)
            print(m_x, m_y, m_h)
            time.sleep(3)
            h_offset = robot.pose_angle.degrees - m_h
            h_offset_rad = math.radians(robot.pose_angle.degrees - m_h)


            cube = None
            print("wheels driven")
            init_rx = robot.pose.position.x
            init_ry = robot.pose.position.y
            init_rh = robot.pose_angle.degrees
            while cube is None:
                try:
                    cube = await robot.world.wait_for_observed_light_cube(timeout=.5)
                    after_rx = robot.pose.position.x
                    after_ry = robot.pose.position.y
                    after_rh = robot.pose_angle.degrees
                    dx = .03937 * (after_rx - init_rx)
                    dy = .03937 * (after_ry - init_ry)
                    dh = after_rh - init_rh
                    new_x = m_x + (dx * math.cos(h_offset_rad)) - (dy * math.sin(h_offset_rad))
                    new_y = m_y + (dx * math.sin(h_offset_rad)) + (dy * math.cos(h_offset_rad))
                    new_h = m_h + dh
                    #init_rx = after_rx
                    #init_ry = after_ry
                    #init_rh = after_rh
                    cube_pose = get_cube_global_pose(robot, new_x, new_y, new_h, cube.pose.position.x * .03937,
                                                     cube.pose.position.y * .03937)
                    #start if-else for diff areas:
                    if role == "pickup":
                        if cube_pose[0] >= 9 or cube_pose[1] <= 4:
                            print ("Cube out of bounds! " + str(cube_pose))
                            cube = None
                            await robot.turn_in_place(degrees(15)).wait_for_completed()
                            continue
                        else:
                            print("In bound cube pose! " + str(cube_pose))
                    else:
                        if cube_pose[0] <= 9 or cube_pose[0] >= 17 or cube_pose[1] <= 4:
                            cube = None
                            print ("Cube out of bounds! " + str(cube_pose))
                            await robot.turn_in_place(degrees(15)).wait_for_completed()
                            continue
                except:
                    await robot.turn_in_place(degrees(15)).wait_for_completed()

            print(cube.pose)
            time.sleep(3)
            ##Cube found

            await robot.pickup_object(cube, use_pre_dock_pose=False, num_retries=6).wait_for_completed()
            #TODO: drive to correct destination
            #TODO: drop object
            #TODO: return to starting position, look in right direction
            after_rx = robot.pose.position.x
            after_ry = robot.pose.position.y
            after_rh = robot.pose_angle.degrees

            dx = .03937 * (after_rx - init_rx)
            dy = .03937 * (after_ry - init_ry)
            dh = after_rh - init_rh

            new_x = m_x + (dx * math.cos(h_offset_rad)) - (dy * math.sin(h_offset_rad))
            new_y = m_y + (dx * math.sin(h_offset_rad)) + (dy * math.cos(h_offset_rad))
            new_h = m_h + dh

            print(new_x, new_y, new_h)
            time.sleep(3)

            if role == "pickup":
                await move_dist_in_global_frame(robot, new_x, new_y, new_h, 12, 9)
                await robot.place_object_on_ground_here(cube).wait_for_completed()
                await move_dist_in_global_frame(robot, 11, 9, robot.pose_angle.degrees - h_offset, 9, 9)
                pf = ParticleFilter(grid)
                state = "unknown"
            else:
                await move_dist_in_global_frame(robot, new_x, new_y, new_h, 23, 15 * storage_cube_mult)
                await robot.place_object_on_ground_here(cube).wait_for_completed() #TODO: put second cube somewhere else
                await move_dist_in_global_frame(robot, 23, 15 * storage_cube_mult,
                                                robot.pose_angle.degrees - h_offset, 23, 9)
                storage_cube_mult -= .25
                pf = ParticleFilter(grid)
                state = "unknown"



        elif state == "unknown":
            last_pose = robot.pose
            if abs(m_x - 130) < 20 and abs(m_y - 90) < 10:
                await robot.turn_in_place(degrees(15)).wait_for_completed()
            else:
                await robot.turn_in_place(degrees(15)).wait_for_completed()
                # await robot.drive_straight(distance_mm(40), speed_mmps(40), False, False,
                #                                            0).wait_for_completed()

    ############################################################################

async def move_dist_in_global_frame(robot, m_x,m_y,m_h, dest_x, dest_y):
    #help
    goal = [dest_x,dest_y]
    # Part 1: Figure out Robot's origin + theta offset
    # For consistency, the global frame is A, the one used by the bot is B
    # need tp find angle theta_b_a (radians) that represents angle FROM X_b TO X_a
    theta_a_b = math.radians(m_h - robot.pose_angle.degrees)
    theta_b_a = math.radians(robot.pose_angle.degrees - m_h)

    # need to find vector t (in frame A) that represents the origin of B in A
    # Solution: We have robot pos in both B and A. a_to_r, b_to_r, and t make a triangle.
    # First we must get R_b in the coordinate system of A, then a_to_r - b_to_r = t.
    a_to_r_in_A = (m_x,m_y)
    b_to_r_in_B = (robot.pose.position.x * .03937, robot.pose.position.y * .03937) # in mm, right?
    b_to_r_in_A_x = (math.cos(theta_a_b)* b_to_r_in_B[0] \
            - math.sin(theta_a_b)*b_to_r_in_B[1])
    b_to_r_in_A_y = (math.sin(theta_a_b) * b_to_r_in_B[0] \
            + math.cos(theta_a_b) * b_to_r_in_B[1])
    b_to_r_in_A = (b_to_r_in_A_x, b_to_r_in_A_y)
    t = ( a_to_r_in_A[0] - b_to_r_in_B[0], a_to_r_in_A[1] - b_to_r_in_B[1] )

    # Part 2: Find goal_in_B
    goal_in_A = (goal[0],goal[1])
    goal_in_B_x = (math.cos(theta_b_a) * goal_in_A[0] \
            - math.sin(theta_b_a) *goal_in_A[1] \
            - t[0]) #should be negative?
    goal_in_B_y = (math.sin(theta_b_a) * goal_in_A[0]
            + math.cos(theta_b_a) * goal_in_A[1]
            - t[1]) #should be negative?
    r_to_goal_in_B = (goal_in_B_x - b_to_r_in_B[0], goal_in_B_y - b_to_r_in_B[1])
    head = math.degrees(math.atan2(r_to_goal_in_B[1], r_to_goal_in_B[0])) - robot.pose_angle.degrees
    actual_head = math.degrees(math.atan2(goal[1],goal[0])) - m_h


    dist = (goal[0]-m_x, goal[1]-m_y)
    delta_t = math.degrees(math.atan2(dist[1], dist[0])) - m_h
    dist = math.sqrt(dist[0]**2 + dist[1]**2)
    print("DISTANCE TO TARGET: {}", dist)

    await robot.turn_in_place(degrees(delta_t)).wait_for_completed()
    #dist = math.sqrt(r_to_goal_in_B[0]**2 + r_to_goal_in_B[1]**2)
    await robot.drive_straight(distance_inches(dist), speed_mmps(40)).wait_for_completed()
    return m_h + delta_t

def get_cube_global_pose(robot, m_x,m_y,m_h, cube_x, cube_y):
    goal = [cube_x,cube_y]
    # Part 1: Figure out Robot's origin + theta offset
    # For consistency, the global frame is A, the one used by the bot is B
    # need tp find angle theta_b_a (radians) that represents angle FROM X_b TO X_a
    theta_a_b = math.radians(m_h - robot.pose_angle.degrees)
    theta_b_a = math.radians(robot.pose_angle.degrees - m_h)

    # need to find vector t (in frame A) that represents the origin of B in A
    # Solution: We have robot pos in both B and A. a_to_r, b_to_r, and t make a triangle.
    # First we must get R_b in the coordinate system of A, then a_to_r - b_to_r = t.
    a_to_r_in_A = (m_x,m_y)
    b_to_r_in_B = (robot.pose.position.x * .03937, robot.pose.position.y * .03937) # in mm, right?
    b_to_r_in_A_x = (math.cos(theta_a_b)* b_to_r_in_B[0] \
            - math.sin(theta_a_b)*b_to_r_in_B[1])
    b_to_r_in_A_y = (math.sin(theta_a_b) * b_to_r_in_B[0] \
            + math.cos(theta_a_b) * b_to_r_in_B[1])
    b_to_r_in_A = (b_to_r_in_A_x, b_to_r_in_A_y)
    t = ( a_to_r_in_A[0] - b_to_r_in_B[0], a_to_r_in_A[1] - b_to_r_in_B[1] )

    # Part 2: Find goal_in_A
    goal_in_B = (goal[0],goal[1])
    goal_in_A_x = (math.cos(theta_a_b) * goal_in_B[0] \
            - math.sin(theta_a_b) *goal_in_B[1] \
            + t[0])
    goal_in_A_y = (math.sin(theta_a_b) * goal_in_B[0]
            + math.cos(theta_a_b) * goal_in_B[1]
            + t[1])
    return goal_in_A_x,goal_in_A_y


class CozmoThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.run_program(run, use_viewer=False)

# class KidnappingThread(threading.Thread):
#     def __init__(self):
#         threading.Thread.__init__(self, daemon=False)
#
#     def is_kidnapped(self):
#         cozmo.run_program(is_kidnapped, use_viewer=False)

if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # # kidnapping thread
    # kidnap_thread = KidnappingThread()
    # kidnap_thread.is_kidnapped()

    # init
    grid = CozGrid(Map_filename)
    gui = GUIWindow(grid)
    gui.start()
