from controller import Robot
import sys

sys.path.append('..')
from utils.motion_library import MotionLibrary
from math import pi, sin

class Wrestler (Robot):
    def __init__(self):
    
        super().__init__()
        self.motion_library = MotionLibrary()
        self.time_step = int(self.getBasicTimeStep())
        self.leds = {'right': self.getDevice("Ears/Led/Right"),
                     'left':  self.getDevice("Ears/Led/Left")}
        self.RShoulderPitch = self.getDevice("RShoulderPitch")
        self.LShoulderPitch = self.getDevice("LShoulderPitch")
        self.Sonar1 = self.getDevice("Sonar/Left")
        self.Sonar2 = self.getDevice("Sonar/Right")
        self.Sonar1.enable(self.time_step)
        self.Sonar2.enable(self.time_step)
           
    def run(self):
    
        self.RShoulderPitch.setPosition(0)
        self.LShoulderPitch.setPosition(0) 
        
  
        ts = 0  
        F = 3
              
        while self.step(self.time_step) != -1:  
            self.motion_library.play('Forwards')
       
            ts = (ts + self.time_step)
            d1 = self.Sonar1.getValue()
            d2 = self.Sonar2.getValue()

            t = ts/1000
            position = sin(t * 2.0 * pi * F)

            
            if min(d1,d2) < 0.3:
                self.RShoulderPitch.setPosition(-position)
                self.LShoulderPitch.setPosition(position)  
                                          
            if t > 2.0:
                self.leds['right'].set(0xff0000)
                self.leds['left'].set(0xff0000)            


wrestler = Wrestler()
wrestler.run()
