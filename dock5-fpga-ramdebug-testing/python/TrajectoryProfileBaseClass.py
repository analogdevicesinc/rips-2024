################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
from abc import ABC, abstractmethod
 
 
class TrajectoryProfileBase(ABC):
 
    @abstractmethod
    def get_t_end(self):
        pass
 
    @abstractmethod
    def get_position(self, t):
        pass
 
    @abstractmethod
    def get_velocity(self, t):
        pass
 
    @abstractmethod
    def get_acceleration(self, t):
        pass