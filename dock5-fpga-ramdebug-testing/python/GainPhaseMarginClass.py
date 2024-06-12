################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import math

# This class is to create multiple functions to calcuate gain and phase crossover freqencies 
# In order to get gain and phase margin by given Motor resistance(R), Inductance(L), dealy(Tau_d), Kp and Ki term.
# For AI - machine learning black box exploration

class GainPhaseMarginClass:

    # Initialise input parameters
    def __init__(self, R, L, Tdelay, PWM_FREQUENCY, Kp, Ki):
        self.R = R
        self.L = L
        self.tau_d = Tdelay/PWM_FREQUENCY
        self.PWM_FREQUENCY = PWM_FREQUENCY
        self.Kp = Kp
        self.Ki = Ki

    # Calculate magnitude value of frequency response(Bode plot)
    def magnitude(self, w):
        
        gain = math.sqrt((self.Kp**2*w**2 + self.Ki**2)/(self.R**2*w**2 + self.L**2*w**4))

        gain_indB = 20*math.log10(gain)

        return gain_indB


    # Calculate phase angle of frequency response(Bode plot)
    def phase_angle(self, w):
        
        phase_angle = math.atan2(self.Kp*w, self.Ki) - math.atan2(self.L*w, self.R) - math.pi/2 - self.tau_d*w
        
        phase_angle_indegree = math.degrees(phase_angle)

        return phase_angle_indegree


    # Calculate gain crossover frequency when bode magnitude is equal to zero 
    def gain_crossover_freq(self):

        w_gc = math.sqrt(((self.Kp**2 - self.R**2) + math.sqrt((self.Kp**2 - self.R**2)**2 + 4*self.L**2*self.Ki**2))/(2*self.L**2))

        return w_gc

    # One of the steps to calculate one of the terms for phase crossover frequency 
    def f_phase_crossover(self, w):

        f_w = math.atan2(self.Kp*w, self.Ki) - math.atan2(self.L*w, self.R) - self.tau_d*w + math.pi/2

        return f_w

    # Calculate phase crossover frequency when bode phase angle is equal to zero 
    # Threshold control how much error of your result
    def phase_crossover_freq(self, threshold = 0.001):
        
        # Find the frequency when the sign change
        sweep_w = 1.0 # Sweep frequency start from 1 rad/s
        f_w = self.f_phase_crossover(sweep_w)

        while f_w > 0:

            sweep_w = sweep_w * 10

            f_w = self.f_phase_crossover(sweep_w)

        w_left = sweep_w/10
        w_right = sweep_w

        print("The phase crossover frequency range has been found beteen", [w_left, w_right])


        # Binary Search to find the phase crossover frequency
        # Initialise variables
        f_w_mid = 1.0
        w_mid = 0.0

        while abs(f_w_mid) >= threshold:

            w_mid = (w_left + w_right) / 2

            f_w_mid = self.f_phase_crossover(w_mid)

            if f_w_mid < 0:

                w_right = w_mid

            elif f_w_mid > 0:

                w_left = w_mid

            else:
                return w_mid

        return w_mid


    # Find gain margin when input frequency is phase crossover frequency
    def gain_margin(self, w_pc):

        gainMargin = abs(self.magnitude(w_pc))

        return gainMargin

    # Find Phase margin when input frequency is gain crossover frequency
    def phase_margin(self, w_gc):

        phase_margin = abs(180 - abs(self.phase_angle(w_gc)))

        return phase_margin


# An simple example to show how to use this class

"""

# Estimate Current Loop - Gain and Phase Margin.
myGainPhaseMargin = GainPhaseMarginClass(Rest, Lest, Tdelay, PWM_FREQUENCY, kp_float, ki_float)
print("#")
print("# Torque/Flux Loop Phase/Gain Margin Estimates.")
print("#")
# Get gain and phase crossover frequencies by given R, L, tau_d, Kp, Ki
w_gc = myGainPhaseMargin.gain_crossover_freq()
w_pc = myGainPhaseMargin.phase_crossover_freq()
# Get Gain and Phase Margin 
gainMargin = myGainPhaseMargin.gain_margin(w_pc)
phaseMargin = myGainPhaseMargin.phase_margin(w_gc)
print("#")
print("# Gain crossover frequency(rad/s) is at ", w_gc)
print("# Phase crossover freqency(rad/s) is at ", w_pc)
print("# Gain margin is ", gainMargin )
print("# Phase margin is ", phaseMargin)
print("#")

"""