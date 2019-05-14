#!/usr/bin/env python
import numpy as np

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
