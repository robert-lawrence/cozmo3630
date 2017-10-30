import cozmo

import numpy as np
from cozmo.objects import LightCube

from cozmo.util import degrees, distance_mm, speed_mmps, radians

from cmap import *
from gui import *
from utils import *
import random as rand
import math
from time import sleep

MAX_NODES = 20000

################################################################################
# NOTE:
# Before you start, please familiarize yourself with class Node in utils.py
# In this project, all nodes are Node object, each of which has its own
# coordinate and parent if necessary. You could access its coordinate by node.x
# or node[0] for the x coordinate, and node.y or node[1] for the y coordinate
################################################################################

def step_from_to(node0, node1, limit=75):
    ############################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    if get_dist(node0, node1) > 75.:
        angle = np.arctan2(node1[1]-node0[1],node1[0]-node0[0])
        return Node((node0[0] + 75*math.cos(angle), node0[1] + 75*math.sin(angle)))
    return node1
    ############################################################################


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    node = False
    while node is False:
        x = rand.uniform(0, cmap.width)
        y = rand.uniform(0, cmap.height)
        newnode = Node(tuple((x, y)))
        if cmap.is_inbound(newnode) and not cmap.is_inside_obstacles(newnode):
            node = newnode
    ############################################################################
    return node


def RRT(cmap, start):
    cmap.add_node(start)

    map_width, map_height = cmap.get_size()

    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        rand_node = cmap.get_random_valid_node()
        nearest_node = None
        for node in cmap.get_nodes():
            if nearest_node is None:
                nearest_node = node
            elif get_dist(rand_node,node) < get_dist(rand_node,nearest_node):
                nearest_node = node
        rand_node = step_from_to(nearest_node, rand_node)
        ########################################################################
        sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
    else:
        print("Please try again :-(")

async def go_to_center(robot: cozmo.robot.Robot):
    start_node = Node((100, 75))
    grid_width, grid_height = cmap.get_size()
    center_node = Node((grid_width / 2, grid_height / 2))
    angle = np.arctan2(center_node.y - start_node.y, center_node.x - start_node.x)
    await robot.turn_in_place(radians(angle)).wait_for_completed()
    # sleep(3.0)
    await robot.drive_straight(distance_mm(get_dist(center_node, start_node)), speed_mmps(50)).wait_for_completed()

    target_cube = None
    robot.drive_wheel_motors(-15, 15)
    while target_cube is None:
        target_cube = await robot.world.wait_for_observed_light_cube()
    robot.stop_all_motors()
    return target_cube



async def CozmoPlanning(robot: cozmo.robot.Robot):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global cmap, stopevent

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions

    target_cube = None
    while target_cube is None:
        try:
            target_cube = await robot.world.wait_for_observed_light_cube(timeout=2.0)
        except:
            target_cube = await go_to_center(robot)

    # target_cube_node = Node((target_cube.pose.position.x, target_cube.pose.position.y))
    # await robot.go_to_pose(target_cube.pose).wait_for_completed()
    path_found = await get_updated_path(cmap, robot, target_cube)
    print("Robot Position:")
    print(robot.pose.position.x, robot.pose.position.y, robot.pose_angle.degrees)
    print("Target Cube Position:")
    print(target_cube.pose.position.x, target_cube.pose.position.y, target_cube.pose)
    for node in path_found:
        print("Node:")
        robot_node = Node((robot.pose.position.x, robot.pose.position.y))
        print(node.x, node.y)
        # Todo figure out how to turn to the correct angle
        angle = (np.arctan2(node.y - robot_node.y, node.x - robot_node.x)) - robot.pose_angle.radians
        # print(angle)
        await robot.turn_in_place(radians(angle)).wait_for_completed()
        await robot.drive_straight(distance_mm(get_dist(node, robot_node)), speed_mmps(50)).wait_for_completed()



async def get_updated_path(cmap, robot, target_cube):
    cube_angle_rad = target_cube.pose.rotation.angle_z.radians
    cube_angle_degs = target_cube.pose.rotation.angle_z.degrees
    # print("CUBE ANGLE:")
    # print(cube_angle)
    cube_pos = target_cube.pose.position
    diag1 = math.radians(45) + cube_angle_degs
    diag2 = -1*math.radians(45) + cube_angle_degs
    goal = Node(( cube_pos.x + 50*math.cos(cube_angle_rad), cube_pos.y + 50*math.sin(cube_angle_rad)))

    cube_obstacle = [ Node((cube_pos.x + 50*math.cos(diag1), cube_pos.y + 50*math.sin(diag1))),
            Node((cube_pos.x + 50 * math.cos(diag2), cube_pos.y + 50 * math.sin(diag2))),
            Node((cube_pos.x - 50*math.cos(diag1), cube_pos.y - 50*math.sin(diag1))),
            Node((cube_pos.x - 50*math.cos(diag2), cube_pos.y - 50*math.sin(diag2))) ]
    cmap.add_obstacle(cube_obstacle)

    robot_start = Node((robot.pose.position.x, robot.pose.position.y))
    cmap.set_start(robot_start)
    cmap.add_goal(goal)
    RRT(cmap, cmap.get_start())
    path_found = []
    if cmap.is_solved:
        for goal in cmap._goals:
            cur = goal
            path_found = [cur] + path_found
            while cur.parent is not None:
                path_found = [cur.parent] + path_found
                cur = cur.parent
        # print(path_found)
    return path_found


################################################################################
#                     DO NOT MODIFY CODE BELOW                                 #
################################################################################

class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            sleep(100)
            cmap.reset()
        stopevent.set()


if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    cmap = CozMap("maps/emptygrid.json", node_generator)
    robot_thread = RobotThread()
    robot_thread.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()
