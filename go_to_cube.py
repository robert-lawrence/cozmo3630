#!/usr/bin/env python3
#!c:/Python35/python3.exe -u
import asyncio
import sys
import cv2
import numpy as np
import cozmo
from cozmo.util import distance_mm, speed_mmps, radians, degrees
import time
import os
from glob import glob

from find_cube import *

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')
def nothing(x):
    pass

YELLOW_LOWER = np.array([9, 135, 101])
YELLOW_UPPER = np.array([179, 215, 255])

GREEN_LOWER = np.array([0,0,0])
GREEN_UPPER = np.array([179, 255, 60])

# Define a decorator as a subclass of Annotator; displays the keypoint
class BoxAnnotator(cozmo.annotate.Annotator):

    cube = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BoxAnnotator.cube is not None:

            #double size of bounding box to match size of rendered image
            BoxAnnotator.cube = np.multiply(BoxAnnotator.cube,2)

            #define and display bounding box with params:
            #msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BoxAnnotator.cube[0]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[1]-BoxAnnotator.cube[2]/2,
                                      BoxAnnotator.cube[2], BoxAnnotator.cube[2])
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BoxAnnotator.cube = None



async def run(robot: cozmo.robot.Robot):

    robot.world.image_annotator.annotation_enabled = False
    robot.world.image_annotator.add_annotator('box', BoxAnnotator)

    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True

    gain,exposure,mode = 390,3,1

    try:
        positions = []
        nones = 0
        while True:
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)   #get camera image
            if event.image is not None:
                image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)

                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure,fixed_gain)

                #find the cube
                cube = find_cube(image, YELLOW_LOWER, YELLOW_UPPER)
                #print(cube)
                BoxAnnotator.cube = cube

                ################################################################
                # Todo: Add Motion Here
                ################################################################
                #need estimated diameter of cube from 5cm away
                # need to turn until x is in the middle of the range. Dimensions are 320x240
                if cube == None:
                    nones = nones + 1
                    if nones > 7:
                        nones = 0
                        await robot.turn_in_place(degrees(40)).wait_for_completed()
                    continue
                nones = 0
                angle = cube[0] - 160
                positions.append((angle,cube[2]))
                if len(positions) > 10:
                    positions.pop(0)
                if len(positions) < 10:
                    continue
                #get avg. position
                avg_x = sum(x[0] for x in positions) / float(len(positions))
                avg_size = sum(x[1] for x in positions) / float(len(positions))
                outliers = sum(0 if abs(x[0]-avg_x) < 20 and abs(x[1]-avg_size) < 20 else 1 for x in positions)
                print(avg_x,avg_size)
                positions = []
                if outliers >= 3:
                    continue

                if (avg_x > 40 or avg_x < -40):
                    await robot.turn_in_place(degrees(10*(-1 if avg_x > 0 else 1))).wait_for_completed()
                    continue
                if (avg_size < 120):
                    dist = 50 if avg_size < 80 else 20
                    await robot.drive_straight(distance_mm(20),speed_mmps(40),False,False,0).wait_for_completed()



    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
    #cv2.destroyAllWindows()


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer = True, force_viewer_on_top = True)
