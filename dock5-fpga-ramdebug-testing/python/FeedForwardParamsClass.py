################################################################################
# Copyright © 2024 Analog Devices, Inc.
################################################################################
class FeedForwardParams(object):
	"""Small class to store the three parameters required for computing the feedforward torque"""
	def __init__(self, fs, bv, J):
		super(FeedForwardParams, self).__init__()
		self.fs = fs # Static friction
		self.bv = bv # Viscous friction
		self.J = J # Inertia