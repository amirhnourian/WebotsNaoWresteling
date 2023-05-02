# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Demonstrates the gait manager (inverse kinematics + simple ellipsoid path).
"""
from controller import Robot, Motion
import sys
sys.path.append('..')
from utils.running_average import RunningAverage
from utils.finite_state_machine import FiniteStateMachine
from utils.current_motion_manager import CurrentMotionManager
import cv2
from utils.camera import Camera
from utils.gait_manager import GaitManager
from utils.fall_detection import FallDetection
from utils.image_processing import ImageProcessing as IP

# Eve's locate_opponent() is implemented in this module:
# Eve dependencies
print('libraries imported')


class Fatima (Robot):
    SMALLEST_TURNING_RADIUS = 10
    SAFE_ZONE = 0.5
    TIME_BEFORE_DIRECTION_CHANGE = 200  # 8000 ms / 40 ms

    def __init__(self):
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = Camera(self)
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 0
        # Time before changing direction to stop the robot from falling off the ring
        self.counter = 0
        # Eve code implementation
        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.time_step = int(self.getBasicTimeStep())

        self.fsm = FiniteStateMachine(
            states=['CHOOSE_ACTION', 'BLOCKING_MOTION'],
            initial_state='CHOOSE_ACTION',
            actions={
                'CHOOSE_ACTION': self.choose_action,
                'BLOCKING_MOTION': self.pending
            }
        )

        # arm motors for getting up from a side fall
        self.RShoulderRoll = self.getDevice("RShoulderRoll")
        self.LShoulderRoll = self.getDevice("LShoulderRoll")

        self.current_motion = CurrentMotionManager()
        # load motion files
        self.motions = {
            'SideStepLeft': Motion('../motions/SideStepLeftLoop.motion'),
            'SideStepRight': Motion('../motions/SideStepRightLoop.motion'),
            'TurnRight': Motion('../motions/TurnRight20.motion'),
            'TurnLeft': Motion('../motions/TurnLeft20.motion'),
            'Shove': Motion('../motions/Shove.motion'),   
        }
        self.opponent_position = RunningAverage(dimensions=1)
        self.dodging_direction = 'left'
        self.counter = 0
        print('Depemdencies Done')

    def run(self):
        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            t = self.getTime()
            print('time done')
            self.gait_manager.update_theta()
            print('self gait manager init done')
            if 0.3 < t < 2:
                self.start_sequence()
                print('start sequence done')
            elif t > 2:
                self.fall_detector.check()
                print('fall check done')
                self.walk()
                print('walk done')

    def start_sequence(self):
        """At the beginning of the match, the robot walks forwards to move away from the edges."""
        self.gait_manager.command_to_motors(heading_angle=0)

    def walk(self):
        """Walk towards the opponent like a homing missile."""
        self.current_motion = None
        normalized_x = self._get_normalized_opponent_x()
        self.reverse = 1
        self.motions['Shove'].play()
        # We set the desired radius such that the robot walks towards the opponent.
        # If the opponent is close to the middle, the robot walks straight.
        # desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x) if abs(normalized_x) > 1e-3 else None
        if self.opponent_position.average > -0.4 and self.opponent_position.average < 0.4:
            self.heading_angle = 0
            if abs(normalized_x) > 1e-3:
                desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x)
            else:
                desired_radius = None
            self.gait_manager.command_to_motors(desired_radius=desired_radius, heading_angle=self.heading_angle)

        elif self.opponent_position.average < -0.4:
            self.current_motion = self.motions['TurnLeft']
            if self.current_motion is None or self.currentMotion.isOver():
                self.current_motion.play()        

        elif self.opponent_position.average > 0.4:
            self.heading_angle = 3.14
#           self.current_motion.set(self.motions['TurnRight'])
            if abs(normalized_x) > 1e-3:
                desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x)
            else:
                desired_radius = None
        else:
            desired_radius = None
            self.heading_angle = None

#        if self.counter > 1.2*self.TIME_BEFORE_DIRECTION_CHANGE:
#            self.counter = 0
#            self.heading_angle = 0
#            self.reverse = -1
#            if abs(normalized_x) > 1e-3:
#                desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x)
#                print(self.counter)
#            else:
#                desired_radius = None
#
#        elif self.counter > self.TIME_BEFORE_DIRECTION_CHANGE:
#            print(self.counter)
#            self.heading_angle = self.reverse*3.14/2
#            if abs(normalized_x) > 1e-3:
#                desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x)
#            else:
#                desired_radius = None
#        elif abs(normalized_x) > 1e-3:
#            desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x)
#            print(self.counter)
#        else:
#            desired_radius = None

        # TODO: position estimation so that if the robot is close to the edge, it switches dodging direction
        # if self.counter > self.TIME_BEFORE_DIRECTION_CHANGE:
        #     self.heading_angle = - self.heading_angle
        #     self.counter = 0
#        self.counter += 1
#        self.gait_manager.command_to_motors(
#            desired_radius=desired_radius, heading_angle=self.heading_angle)

    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            return 0
        return horizontal_coordinate * 2 / img.shape[1] - 1
    # All functions from Eve

    def choose_action(self):
        if self.opponent_position.average < -0.4:
            self.current_motion.set(self.motions['TurnLeft'])
        elif self.opponent_position.average > 0.4:
            self.current_motion.set(self.motions['TurnRight'])
        else:
            # dodging by alternating between left and right side steps to avoid easily falling off the ring
            if self.dodging_direction == 'left':
                if self.counter < self.NUMBER_OF_DODGE_STEPS:
                    self.current_motion.set(self.motions['SideStepLeft'])
                    self.counter += 1
                else:
                    self.dodging_direction = 'right'
            elif self.dodging_direction == 'right':
                if self.counter > 0:
                    self.current_motion.set(self.motions['SideStepRight'])
                    self.counter -= 1
                else:
                    self.dodging_direction = 'left'
            else:
                return
        self.fsm.transition_to('BLOCKING_MOTION')

    def pending(self):
        # waits for the current motion to finish before doing anything else
        if self.current_motion.is_over():
            self.fsm.transition_to('CHOOSE_ACTION')

    def _get_normalized_opponent_horizontal_position(self):
        """Returns the horizontal position of the opponent in the image, normalized to [-1, 1]
            and sends an annotated image to the robot window."""
        img = self.camera.get_image()
        largest_contour, vertical, horizontal = self.locate_opponent(img)
        output = img.copy()
        if largest_contour is not None:
            cv2.drawContours(output, [largest_contour], 0, (255, 255, 0), 1)
            output = cv2.circle(output, (horizontal, vertical), radius=2,
                                color=(0, 0, 255), thickness=-1)
        self.camera.send_to_robot_window(output)
        if horizontal is None:
            return 0
        return horizontal * 2 / img.shape[1] - 1

    def locate_opponent(self, img):
        """Image processing demonstration to locate the opponent robot in an image."""
        # we suppose the robot to be located at a concentration of multiple color changes (big Laplacian values)
        laplacian = cv2.Laplacian(img, cv2.CV_8U, ksize=3)
        # those spikes are then smoothed out using a Gaussian blur to get blurry blobs
        blur = cv2.GaussianBlur(laplacian, (0, 0), 2)
        # we apply a threshold to get a binary image of potential robot locations
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)
        # the binary image is then dilated to merge small groups of blobs together
        closing = cv2.morphologyEx(
            thresh, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15)))
        # the robot is assumed to be the largest contour
        largest_contour = IP.get_largest_contour(closing)
        if largest_contour is not None:
            # we get its centroid for an approximate opponent location
            vertical_coordinate, horizontal_coordinate = IP.get_contour_centroid(
                largest_contour)
            return largest_contour, vertical_coordinate, horizontal_coordinate
        else:
            # if no contour is found, we return None
            return None, None, None


# create the Robot instance and run main loop
wrestler = Fatima()
wrestler.run()
