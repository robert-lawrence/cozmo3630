#!/usr/bin/env python3

# Copyright (c) 2016 Anki, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License in the file LICENSE.txt or at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''Tell Cozmo to find a cube, and then drive up to it
This is a test / example usage of the robot.go_to_object call which creates a
GoToObject action, that can be used to drive within a given distance of an
object (e.g. a LightCube).
'''

import asyncio

import cozmo
from cozmo.util import degrees, distance_mm, Pose

import fsmlib


async def go_to_ar_cube(robot: cozmo.robot.Robot, fsm):
    '''The core of the go to object test program'''

    # look around and try to find a cube
    look_around = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)

    cube = None

    try:
        cube = await robot.world.wait_for_observed_light_cube()
        # print("Found cube: %s" % cube)
    except asyncio.TimeoutError:
        print("Didn't find a cube")
    finally:
        look_around.stop()
        fsm.found_cube()
    if cube:
        # Drive to 70mm away from the cube (much closer and Cozmo
        # will likely hit the cube) and then stop.
        # print(cube.pose)
        # fsm.found_cube()
        # robot.say_text("Found cube, going to cube!")
        cube_x = cube.pose.position.x
        cube_y = cube.pose.position.y
        cube_angle = cube.pose.rotation.angle_z.degrees
        pose_x = cube_x - robot.pose.position.x
        pose_y = cube_y - robot.pose.position.y
        pose_x += (-60 + (abs(cube_angle) / 1.5))
        if cube_angle < -90:
            pose_y += ((abs(abs(cube_angle) - 180)) / 1.5)
        elif cube_angle < 0:
            pose_y += ((abs(cube_angle)) / 1.5)
        elif cube_angle > 90:
            pose_y -= ((abs(abs(cube_angle) - 180)) / 1.5)
        else:
            pose_y -= cube_angle / 1.5
        print("robot vals : x-val: %s, y-val: %s", robot.pose.position.x, robot.pose.position.y)
        print("cube vals : x-val: %d, y-val: %d", cube_x, cube_y)
        await robot.go_to_pose(Pose(pose_x, pose_y, 0,
                              angle_z=cube.pose.rotation.angle_z), relative_to_robot=True).wait_for_completed()
        # print(robot.pose)
        if robot.pose.position.x - cube_x < -60 or robot.pose.position.x - cube_x > 60:
            # fsm.lost_cube()
            await fsmlib.trigger(fsm, "lost_cube", robot)
            # robot.say_text("Lost Cube")
            await go_to_ar_cube(robot, fsm)
        if robot.pose.position.y - cube_y < -60 or robot.pose.position.y - cube_y > 60:
            # fsm.lost_cube()
            await fsmlib.trigger(fsm, "lost_cube", robot)
            # robot.say_text("Lost Cube")
            await go_to_ar_cube(robot, fsm)
        # robot.say_text("At the cube")
        await fsmlib.trigger(fsm, "at_cube", robot)
