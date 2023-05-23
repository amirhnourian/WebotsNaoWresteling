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
# Eve's locate_opponent() is implemented in this module:
from utils.image_processing import ImageProcessing as IP
from utils.fall_detection import FallDetection
from utils.gait_manager import GaitManager
from utils.camera import Camera

# libraries added to fatima
from utils.camera_bottom import CameraBottom
from utils.running_average import RunningAverage
from utils.image_processing import ImageProcessing as IP
from utils.finite_state_machine import FiniteStateMachine
from utils.current_motion_manager import CurrentMotionManager
from utils.border_detection import BorderDetection
import cv2


class Fatima (Robot):
    SMALLEST_TURNING_RADIUS = 0.2
    SAFE_ZONE = 0
    TIME_BEFORE_DIRECTION_CHANGE = 200  # 8000 ms / 40 ms

    def __init__(self):
        # init for fatima
        Robot.__init__(self)
        self.time_step = int(self.getBasicTimeStep())

        self.camera = Camera(self)
        self.fall_detector = FallDetection(self.time_step, self)
        self.gait_manager = GaitManager(self, self.time_step)
        self.heading_angle = 0
        # Time before changing direction to stop the robot from falling off the ring
        self.counter = 0


        # init for motion file generation
        # retrieves the WorldInfo.basicTimeTime (ms) from the world file
        self.time_step = int(self.getBasicTimeStep())

        self.camera = Camera(self)

        # arm motors for getting up from a side fall
        self.RShoulderRoll = self.getDevice("RShoulderRoll")
        self.LShoulderRoll = self.getDevice("LShoulderRoll")

        self.fall_detector = FallDetection(self.time_step, self)
        self.current_motion = CurrentMotionManager()
        # load motion files
        self.motions = {
            'SideStepLeft': Motion('../motions/SideStepLeftLoop.motion'),
            'SideStepRight': Motion('../motions/SideStepRightLoop.motion'),
            'TurnRight': Motion('../motions/TurnRight20.motion'),
            'TurnLeft': Motion('../motions/TurnLeft20.motion'),
            'Forwards50': Motion('../motions/Forwards50.motion'),
            'Shove': Motion('../motions/Shove.motion'),
        }
        self.opponent_position = RunningAverage(dimensions=1)
        self.dodging_direction = 'left'
        self.counter = 0

        # init for border detection
        self.border_detector = BorderDetection(self.time_step, self)
        self.camera_bottom = CameraBottom(self)

        # adjusting head position
        # HeadPitch
        self.HeadPitch = self.getDevice("HeadPitch")
        self.HeadPitch.setPosition(0.25)

        self.t_border_period = 3        
         

    def run(self):
        while self.step(self.time_step) != -1:
            # We need to update the internal theta value of the gait manager at every step:
            t = self.getTime()
            self.gait_manager.update_theta()
            if 0.3 < t < 2:
                self.start_sequence()
            elif t > 2:
                self.fall_detector.check()
                if self.t_border_period < t and self.t_border_period+1 > t:
                    self.border()
                    self.t_border_period += 1
                    print('Time border checked ', self.t_border_period)
                elif self.t_border_period+1 < t:
                    self.t_border_period = t
                self.walk()

    def start_sequence(self):
        """At the beginning of the match, the robot walks forwards to move away from the edges."""
        self.gait_manager.command_to_motors(heading_angle=0)

    def walk(self):
        """Walk towards the opponent like a homing missile."""
        normalized_x = self._get_normalized_opponent_x()
        # We set the desired radius such that the robot walks towards the opponent.
        # If the opponent is close to the middle, the robot walks straight.
        desired_radius = (self.SMALLEST_TURNING_RADIUS / normalized_x) if abs(normalized_x) > 1e-3 else None
        # TODO: position estimation so that if the robot is close to the edge, it switches dodging direction
        if self.counter > self.TIME_BEFORE_DIRECTION_CHANGE:
            self.heading_angle = - self.heading_angle
            self.counter = 0
        self.counter += 1
        self.motions['Shove'].play()
        self.gait_manager.command_to_motors(desired_radius=desired_radius, heading_angle=self.heading_angle)

    def _get_normalized_opponent_x(self):
        """Locate the opponent in the image and return its horizontal position in the range [-1, 1]."""
        img = self.camera.get_image()
        _, _, horizontal_coordinate = IP.locate_opponent(img)
        if horizontal_coordinate is None:
            return 0
        return horizontal_coordinate * 2 / img.shape[1] - 1
    def border(self):
        imgB = self.camera_bottom.get_image()
        output = imgB.copy()
        # print('border function initiated')
        self.border_detector.avoid_line(output)


# create the Robot instance and run main loop
wrestler = Fatima()
wrestler.run()
