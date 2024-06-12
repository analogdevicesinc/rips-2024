################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
from TrajectoryProfileBaseClass import TrajectoryProfileBase
 
 
class TrajectoryProfileSawtoothVelocity(TrajectoryProfileBase):
    def __init__(self, peak_velocity, peak_time):
        self.peak_velocity = peak_velocity
        self.slope = self.peak_velocity/peak_time
        self.t_end = 4*peak_time

    def get_t_end(self):
        return self.t_end
 
    def get_position(self, t):
        if t < 0:
            position = 0
        elif t < (1./4.)*self.t_end:
            position = t*t*self.slope/2.0
        elif t < (2./4.)*self.t_end:
            position = -t*t*self.slope/2.0 + (self.peak_velocity + self.t_end/4.*self.slope)*t - self.peak_velocity*self.t_end/4.
        elif t < (3./4.)*self.t_end:
            position = -t*t*self.slope/2.0 + self.t_end/2.*self.slope*t + self.peak_velocity*self.t_end/4 - self.t_end*self.t_end/8.*self.slope
        elif t <= (4./4.)*self.t_end:
            position = t*t*self.slope/2.0 - (self.peak_velocity + self.t_end*3./4.*self.slope)*t + self.peak_velocity*self.t_end + self.t_end*self.t_end/4.*self.slope
        else:
            position = 0
        return position
 
    def get_velocity(self, t):
        if t < 0:
            velocity = 0
        elif t < (1./4.)*self.t_end:
            velocity = t*self.slope
        elif t < (2./4.)*self.t_end:
            velocity = self.peak_velocity - (t - self.t_end/4.)*self.slope
        elif t < (3./4.)*self.t_end:
            velocity = -(t - self.t_end/2.)*self.slope
        elif t <= (4./4.)*self.t_end:
            velocity = -self.peak_velocity + (t - self.t_end*3./4.)*self.slope
        else:
            velocity = 0
        return velocity
 
    def get_acceleration(self, t):
        if t < 0:
            acceleration = 0
        elif t < (1./4.)*self.t_end:
            acceleration = self.slope
        elif t < (2./4.)*self.t_end:
            acceleration = -self.slope
        elif t < (3./4.)*self.t_end:
            acceleration = -self.slope
        elif t <= (4./4.)*self.t_end:
            acceleration = self.slope
        else:
            acceleration = 0
        return acceleration
 
 
class TrajectoryProfileSawtoothAcceleration(TrajectoryProfileBase):
    def __init__(self, peak_acceleration, peak_time):
        self.peak_accel = peak_acceleration
        self.slope = self.peak_accel/peak_time
        self.t_end = 4*peak_time

    def get_t_end(self):
        return self.t_end

    def get_position(self, t): # TO DO: need to intergrate the velocity and compute the constants for piecewise continuity of position
        if t < 0:
            position = 0
        elif t < (1./4.)*self.t_end:
            position = 0
        elif t < (2./4.)*self.t_end:
            position = 0
        elif t < (3./4.)*self.t_end:
            position = 0
        elif t <= (4./4.)*self.t_end:
            position = 0
        else:
            position = 0
        return position
 
    def get_velocity(self, t):
        if t < 0:
            velocity = 0
        elif t < (1./4.)*self.t_end:
            velocity = t*t*self.slope/2.0
        elif t < (2./4.)*self.t_end:
            velocity = -t*t*self.slope/2.0 + (self.peak_accel + self.t_end/4.*self.slope)*t - self.peak_accel*self.t_end/4.
        elif t < (3./4.)*self.t_end:
            velocity = -t*t*self.slope/2.0 + self.t_end/2.*self.slope*t + self.peak_accel*self.t_end/4 - self.t_end*self.t_end/8.*self.slope
        elif t <= (4./4.)*self.t_end:
            velocity = t*t*self.slope/2.0 - (self.peak_accel + self.t_end*3./4.*self.slope)*t + self.peak_accel*self.t_end + self.t_end*self.t_end/4.*self.slope
        else:
            velocity = 0
        return velocity
 
    def get_acceleration(self, t):
        if t < 0:
            acceleration = 0
        elif t < (1./4.)*self.t_end:
            acceleration = t*self.slope
        elif t < (2./4.)*self.t_end:
            acceleration = self.peak_accel - (t - self.t_end/4.)*self.slope
        elif t < (3./4.)*self.t_end:
            acceleration = -(t - self.t_end/2.)*self.slope
        elif t <= (4./4.)*self.t_end:
            acceleration = -self.peak_accel + (t - self.t_end*3./4.)*self.slope
        else:
            acceleration = 0
        return acceleration
