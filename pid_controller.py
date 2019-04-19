#!/usr/bin/env python
import numpy as np

class PIDControllerThreePoints():
   
    def __init__(self, kp, kd, range_output=None):
        self.x_controller = PIDController(kp, kd, range_output)
        self.y_controller = PIDController(kp, kd, range_output)
        self.z_controller = PIDController(kp, kd, range_output)
    
    def update(self, value, target_value, time):
        x, y, z = value
        x_t, y_t, z_t = target_value
        x += self.x_controller.update(x, x_t, time)
        y += self.y_controller.update(y, y_t, time)
        z += self. z_controller.update(z, z_t, time)
        return (x,y,z)


class PIDController:
    def __init__(self, kp, kd, range_output=None):
        self.kp = kp
        self.kd = kd
        self.range_output = range_output
        self.previousError = 0.0
        self.previousTime = None

    def update(self, value, target_value, time):
        error = target_value - value
        p = self.kp * error
        d = 0
        if(self.previousTime):
            dt = time - self.previousTime
            if dt > 0:
                d = self.kd * (error - self.previousError) / dt
        output = p + d
        self.previousError = error
        if(self.range_output is None):
            return output
        return np.clip(output, self.range_output[0], self.range_output[1])


class PIDControllerTorque:
   
    def __init__(self, kp, kd, range_output=None):
        self.kp = kp
        self.kd = kd
        self.range_output = range_output
        
    def update(self, value, target_value, velocity):
        # :print(value, target_value)
        error = target_value - value
        p = self.kp * error
        d = self.kd * velocity
        output = p - d
        if(self.range_output is None):
            return output
        return np.clip(output, self.range_output[0], self.range_output[1])
