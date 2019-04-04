#!/usr/bin/env python
import numpy as np

class PIDControllerThreePoints():
     """Query this function to get the next value to update to. Do small updates

        Args:
        value (tuple):  Tuple of 3 floats that represent current position
        target_value (tuple): Tuple of 3 floats that represents goal position
        time (float): current recorded time, for Integral portion of PID controller

        Returns:
        float. next update value::

        A way you might use this is
        curr_position = ...
        goal_position = ...
        >>> while(curr_position not close to goal position)
        >>> ....curr_position = pid_controller.update(curr_position, goal_position)

        """
       
   
    def __init__(self, kp, ki, kd, range_i, range_output):
        x_controller = PIDController(kp, ki, kd, range_i, range_output)
        y_controller = PIDController(kp, ki, kd, range_i, range_output)
        z_controller = PIDController(kp, ki, kd, range_i, range_output)
    
    def update(self, value, target_value, time):
        x, y, z = value
        x_t, y_t, z_t = target_value
        x += x_controller.update(x, x_t)
        y += y_controller.update(y, y_t)
        z += z_controller.update(z, z_t)
        return (x,y,z)




class PIDController:
    def __init__(self, kp, ki, kd, range_i, range_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.range_i = range_i
        self.range_output = range_output
        self.previousError = 0.0
        self.integral = 0.0
        self.previousTime = None

    def update(self, value, target_value, time):
        
        error = target_value - value
        p = self.kp * error
        d = 0
        i = 0
        if self.previousTime is not None:
            dt = time - self.previousTime
            if dt > 0:
                d = self.kd * (error - self.previousError) / dt
            self.integral += error * dt
            self.integral = clamp(self.integral, self.range_i)
            i = self.ki * self.integral
        output = p + i + d
        self.previousTime = time
        self.previousError = error
        # TODO 70 is a magic number, fix this ASAP.
        return np.clip(output, range_output[0], range_output[1])