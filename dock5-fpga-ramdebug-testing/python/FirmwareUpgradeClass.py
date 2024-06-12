################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
from ubltools.memory.tm01map._tm01_map_latest import MCC
from MccHelpersClass import MccHelpersClass
from FeedForwardParamsClass import FeedForwardParams
from register_helpers import to_register32

class VelocityWithFeedForward(object):
	"""Small class to store the three parameters required for computing the feedforward torque"""
	def __init__(self, interface, feedforward_params=FeedForwardParams(0,0,0)):
		super(VelocityWithFeedForward, self).__init__()
		self.mcchelp = MccHelpersClass(interface)
		self.ff_params = feedforward_params		
		self.mcc_velocity_target = MCC.find("MCC.PID_VELOCITY_TARGET")
		self.mcc_torque_offset = MCC.find("MCC.PID_TORQUE_FLUX_OFFSET")

	def mcc_write(self, velocity_target):
		# Compute the required feedforward torque to compensate for friction and inertia
		velocity_sign = 1 if velocity_target > 0 else (-1 if velocity_target < 0 else 0)
		torque_offset = self.ff_params.fs*velocity_sign + self.ff_params.bv*velocity_target
		# Convert the velocity and torque offset to integer or hexadecimal, as needed for the registers
		velocity_target = int(velocity_target)
		torque_offset = to_register32(int(torque_offset), 0)
		# Publish the new target velocity and torque offset to the chip
		self.mcchelp.mcc_write(self.mcc_torque_offset, torque_offset)
		self.mcchelp.mcc_write(self.mcc_velocity_target, velocity_target)