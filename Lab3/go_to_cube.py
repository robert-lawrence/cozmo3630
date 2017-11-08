#!/usr/bin/env python3
# !c:/Python35/python3.exe -u
import asyncio
import cv2
import numpy as np
import cozmo
import sys

from cozmo.util import distance_mm, speed_mmps, degrees
from .find_cube import *
from .go_to_ar_cube import *

from .fsmlib import *

try:
    from PIL import ImageDraw, ImageFont
except ImportError:
    sys.exit('run `pip3 install --user Pillow numpy` to run this example')


def nothing(x):
    pass


YELLOW_LOWER = np.array([9, 135, 101])
YELLOW_UPPER = np.array([179, 215, 255])

GREEN_LOWER = np.array([0, 0, 0])
GREEN_UPPER = np.array([179, 255, 60])


# Define a decorator as a subclass of Annotator; displays the keypoint
class BoxAnnotator(cozmo.annotate.Annotator):
    cube = None

    def apply(self, image, scale):
        d = ImageDraw.Draw(image)
        bounds = (0, 0, image.width, image.height)

        if BoxAnnotator.cube is not None:
            # double size of bounding box to match size of rendered image
            BoxAnnotator.cube = np.multiply(BoxAnnotator.cube, 2)

            # define and display bounding box with params:
            # msg.img_topLeft_x, msg.img_topLeft_y, msg.img_width, msg.img_height
            box = cozmo.util.ImageBox(BoxAnnotator.cube[0] - BoxAnnotator.cube[2] / 2,
                                      BoxAnnotator.cube[1] - BoxAnnotator.cube[2] / 2,
                                      BoxAnnotator.cube[2], BoxAnnotator.cube[2])
            cozmo.annotate.add_img_box_to_image(image, box, "green", text=None)

            BoxAnnotator.cube = None


async def run(robot: cozmo.robot.Robot):

    # Move lift down and tilt the head up
    robot.move_lift(-3)
    await robot.set_head_angle(degrees(-10)).wait_for_completed()

    robot.world.image_annotator.annotation_enabled = False
    robot.world.image_annotator.add_annotator('box', BoxAnnotator)

    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = True
    robot.camera.enable_auto_exposure = True

    gain, exposure, mode = 390, 3, 1

    fsm = fsmlib.init_fsm()

    try:
        positions = []
        last_angle = 0
        nones = 0

        while True:
            event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)  # get camera image

            # i = Image.new('RGBA', (cozmo.oled_face.SCREEN_WIDTH, cozmo.oled_face.SCREEN_HEIGHT), (0,0,0,0))
            # d = ImageDraw.Draw(i)
            # # draw text, full opacity
            # d.text((10,60), fsm.current, fill=(255,255,255,255))
            #
            # image_data = cozmo.oled_face.convert_image_to_screen_data(i)
            # action = robot.display_oled_face_image(image_data, in_parallel=True)
            #
            # action.wait_for_completed()

            if event.image is not None:
                image = cv2.cvtColor(np.asarray(event.image), cv2.COLOR_BGR2RGB)

                if mode == 1:
                    robot.camera.enable_auto_exposure = True
                else:
                    robot.camera.set_manual_exposure(exposure, fixed_gain)

                if fsm.current == 'search_for_AR_cube':
                    # robot.say_text("Searching for AR cube!")
                    await go_to_ar_cube(robot, fsm)
                    await fsmlib.trigger(fsm, "switch_to_color", robot)


                elif fsm.current == 'go_to_colored_cube':
                    # find the cube
                    cube = find_cube(image, YELLOW_LOWER, YELLOW_UPPER)
                    BoxAnnotator.cube = cube

                    ################################################################
                    # Todo: Add Motion Here
                    ################################################################
                    # need estimated diameter of cube from 5cm away
                    # need to turn until x is in the middle of the range. Dimensions are 320x240
                    if cube == None:
                        nones = nones + 1
                        if nones > 7:
                            nones = 0
                            robot.stop_all_motors()
                            direction = -1 if last_angle > 0 else 1 
                            await robot.turn_in_place(degrees(40*direction)).wait_for_completed()
                        continue
                    nones = 0
                    angle = cube[0] - 160
                    last_angle = angle
                    positions.append((angle, cube[2]))
                    if len(positions) > 10:
                        positions.pop(0)
                    if len(positions) < 10:
                        continue
                    # get avg. position
                    avg_x = sum(x[0] for x in positions) / float(len(positions))
                    avg_size = sum(x[1] for x in positions) / float(len(positions))
                    outliers = sum(0 if abs(x[0] - avg_x) < 20 and abs(x[1] - avg_size) < 20 else 1 for x in positions)
                    print(avg_x, avg_size)
                    if outliers >= 3:
                        continue

                    if (avg_size > 100):
                        robot.stop_all_motors()
                        if (avg_x > 40 or avg_x < -40):
                            await robot.turn_in_place(degrees(10 * (-1 if avg_x > 0 else 1))).wait_for_completed()
                            positions = []
                            continue
                        if (avg_size < 120):
                            dist = 50 if avg_size < 80 else 20
                            await robot.drive_straight(distance_mm(20), speed_mmps(40), False, False,
                                                       0).wait_for_completed()
                            positions = []
                            continue
                    else:
                        print("driving wheels")
                        l_speed = 20
                        r_speed = 20
                        if (positions[-1][0] > 40):
                            l_speed += 20
                        if (positions[-1][0] < -40):
                            r_speed += 20
                        robot.drive_wheel_motors(l_speed,r_speed)
                        await asyncio.sleep(0.1)


                elif fsm.current == "at_colored_cube":
                    continue


    except KeyboardInterrupt:
        print("")
        print("Exit requested by user")
    except cozmo.RobotBusy as e:
        print(e)
        # cv2.destroyAllWindows()


if __name__ == '__main__':
    cozmo.run_program(run, use_viewer=True, force_viewer_on_top=True)
