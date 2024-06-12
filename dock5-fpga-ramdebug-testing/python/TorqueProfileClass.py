################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import math
from math import sqrt
import time
import csv
import os
from collections import OrderedDict
from datetime import datetime
from turtle import position
from ubltools.memory.tm01map._tm01_map_latest import MCC  # ToDo: Clean this up
from ubltools.helpers.field import Field
from ubltools.helpers.register import Register
import pytrinamic.RAMDebug as RAMDebug
from MccHelpersClass import MccHelpersClass
from TM01SystemUnitsSetup import TM01SystemUnitsSetup
from StimulusCaptureClass import StimulusCaptureClass
from TorqueFluxSystemIDTuningClass import TorqueFluxSystemIDTuningClass

from register_helpers import to_register32
from register_helpers import to_signed_32
from register_helpers import to_signed_16

class TorqueProfileClass:
    
    def __init__(self, interface, TM01Setup, kp_int, ki_int, current_norm_p, current_norm_i, file_path, RAMDEBUG_CAPACITY=8192):
        self.TM01Setup = TM01Setup

        self.filename_prefix = file_path
        now = datetime.now() # current date and time
        date_time = now.strftime("%Y%m%d_%H%M%S")
        self.csvfilename = os.path.splitext(self.filename_prefix)[0]+'_'+date_time+'.csv'
        self.RAMDEBUG_CAPACITY = RAMDEBUG_CAPACITY

        self.my_interface = interface
        self.mcchelp = MccHelpersClass(self.my_interface)
        self.MinTorque = -270
        self.MaxTorque =  270
        self.TorqueStep = 60
        self.kp = kp_int
        self.ki = ki_int
        self.norm_p = current_norm_p
        self.norm_i = current_norm_i

    def detect_critical_current_kest(self, max_torque_codes, Rest, critical_threshold=300000, current_factor_inc=0.01):

        # Variables initialization 
        current_factor = 0
        critical_current_codes = 0
        target_current_codes = 0
        position_difference = 0
        previous_position = 0 
        current_position = 0
        count = 0

        # Sampling Frequency divisors
        PWM_FREQUENCY = 25000
        RAMDEBUG_DIVISOR = 1     # Measurement or Actual data is played at the PWM Rate / RAMDEBUG_DIVISOR
        RAMDEBUG_RATE = PWM_FREQUENCY/RAMDEBUG_DIVISOR
        RAMDEBUG_PERIOD = 1/RAMDEBUG_RATE
        STIMULUS_DIVISOR = 1      # Stimulus or Target data is played at the PWM Rate / STIMULUS_DIVISOR
        STIMULUS_RATE = PWM_FREQUENCY/STIMULUS_DIVISOR
        STIMULUS_PERIOD = 1/STIMULUS_RATE
        # Set at least one pretrigger to not miss the first value of the stimulus
        RAMDEBUG_PRETRIGGER_SAMPLES = 1 # Number of Pretrigger samples.
        # RAMDEBUG_SAMPLES = 1024 # Per-channel sample count
        CAPTURE_REGS = [
            "MCC.PID_TORQUE_FLUX_TARGET",
            "MCC.PID_TORQUE_FLUX_ACTUAL",
            "MCC.FOC_UQ_UD",
            "MCC.PID_VELOCITY_ACTUAL"
        ]
        RAMDEBUG_SAMPLES = int(self.RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS)))       
        myTorqueStimCapture = StimulusCaptureClass(self.my_interface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, STIMULUS_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES)
        time.sleep(1)

        print("# Starting to find critical current codes. Part 1......")
        while True:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(1.5)

            # Get encoder data from current position
            current_position = self.pos_average()
            #print(self.mcchelp.mcc_read(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL))

            # Calculate the absolute difference betweeb previous and current position
            position_difference = abs(current_position - previous_position)
            print("Current factor = ", current_factor , "Current Encoder Position = ", current_position, "Position Difference = ", position_difference)
            if abs(position_difference) > critical_threshold and count != 0:
                print("# Find target current codes are = ", target_current_codes, "Position difference = ", position_difference)
                previous_position = current_position
                break
            else:
                previous_position = current_position
                # Increase current codes by current_factor_inc per loop # 1% per loop
                current_factor = current_factor + current_factor_inc # 0.01
                target_current_codes = int(current_factor*max_torque_codes)
                count = count + 1


            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                raw_data = myTorqueStimCapture.RAMDebug_Setup_Capture_NoStep("MCC.PID_TORQUE_FLUX_TARGET.PID_TORQUE_TARGET", magnitude=target_current_codes)
                # time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break

        critical_current_codes = target_current_codes
        kest_period = RAMDEBUG_PERIOD
        k_est_data = myTorqueStimCapture.GrabExportData(raw_data, True, True)

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)

        return critical_current_codes, k_est_data, kest_period


    def detect_critical_current(self, max_torque_codes, critical_threshold=300000):

        # Variables initialization 
        current_factor = 0
        critical_current_codes = 0
        target_current_codes = 0
        position_difference = 0
        previous_position = 0 
        current_position = 0
        count = 0

        print("# Starting to find critical current codes. Part 1......")
        while True:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(1.5)

            # Get encoder data from current position
            current_position = self.pos_average()
            #print(self.mcchelp.mcc_read(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL))

            # Calculate the absolute difference betweeb previous and current position
            position_difference = abs(current_position - previous_position)
            print("Current factor = ", current_factor , "Current Encoder Position = ", current_position, "Position Difference = ", position_difference)
            if abs(position_difference) > critical_threshold and count != 0:
                print("# Find target current codes are = ", target_current_codes, "Position difference = ", position_difference)
                previous_position = current_position
                break
            else:
                previous_position = current_position
                # Increase current codes by 1% per loop
                current_factor = current_factor + 0.01
                target_current_codes = int(current_factor*max_torque_codes)
                count = count + 1


            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break

        
        critical_current_codes = target_current_codes

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)


        return critical_current_codes

    def detect_critical_current_two_direction(self, max_torque_codes, critical_threshold=700000):

        # Variables initialization 
        current_factor = 0
        critical_current_codes = 0
        target_current_codes = 0
        position_difference = 0
        previous_position = 0 
        current_position = 0
        count = 0

        time.sleep(1)
        print("# Starting to find critical current codes. Part 1......")
        while True:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4)

            # Get encoder data from current position
            current_position = self.pos_average()

            # Calculate the absolute difference betweeb previous and current position
            position_difference = abs(current_position - previous_position)
            print("Current factor = ", current_factor , "Current Encoder Position = ", current_position, "Position Difference = ", position_difference)
            if abs(position_difference) > critical_threshold and count != 0:
                print("# Find target current codes are = ", target_current_codes, "Position difference = ", position_difference)
                previous_position = current_position
                break

            elif count % 2 == 0:
                previous_position = current_position
                # Increase current codes by 1% per loop
                # current_factor = current_factor + 0.01
                target_current_codes = int(current_factor*max_torque_codes)
                count = count + 1
            else:
                previous_position = current_position
                # Increase current codes by 1% per loop
                current_factor = current_factor + 0.01
                target_current_codes = -int(current_factor*max_torque_codes)
                count = count + 1



            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break

        
        critical_current_codes = target_current_codes

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)


        return critical_current_codes




    def detect_end_stop_final(self, critical_current_codes, stop_threshold=1000):
        # Variables initialization 
        position_difference = 0
        previous_position = 0 
        current_position = 0
        FirstEndPosition = 0
        start_time = 0
        end_time = 0
        timeout = 0
        count = 0
        left_end_found = 0

        # # record the initial position from encoder( not sure if we need to scale it atm)
        # initial_position= self.pos_average()

        # # Assign initial position to previous position
        # previous_position = initial_position

        previous_position= self.pos_average()

        # Sleep for a second
        time.sleep(1)
        print("# Starting to detect first end of linear slide. Part 2......")

        start_time = time.time()

        while timeout <= 20:
            # Set flux and velocity registers to zero and motor stop
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4) # Wait motor to be stopped

            # Get encoder data from current position
            current_position = self.pos_average()

            # Calculate the absolute difference between previous and current position
            position_difference = abs(current_position - previous_position)
            print("Current Encoder Position = ", current_position, "Position Difference = ", position_difference)
            
            if abs(position_difference) <= stop_threshold and count != 0:
                print("# First End Found at = ", current_position, "Position difference = ", position_difference)
                previous_position = current_position
                left_end_found = 1
                break
            else:
                previous_position = current_position
                count = count + 1

            # Apply current  to make motor move 
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(critical_current_codes, 0)) # Write to Torque Target Register.
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
            time.sleep(0.15)
            end_time = time.time()
            timeout = end_time - start_time

        if left_end_found == 1:
            # Get first end encoder position
            FirstEndPosition = current_position
        else:
            print("# First End Not Found - Assign a predetermined value to be constrain")
            FirstEndPosition = 0

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)

        # Variables Reset 
        position_difference = 0
        current_position = 0
        SecondEndPosition = 0
        start_time = 0
        end_time = 0
        timeout = 0
        count = 0
        right_end_found = 0


        # Sleep for a second
        time.sleep(1)

        print("# Starting to detect second end of linear slide. Part 3......")
        
        start_time = time.time()
        while timeout <= 120:
            # Set flux and velocity registers to zero and motor stop
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4) # Wait motor to be stopped

            # Get encoder data from current position
            current_position = self.pos_average()

            # Calculate the absolute difference betweeb previous and current position
            position_difference = abs(current_position - previous_position)

            print("Current Encoder Position = ", current_position, "Position Difference = ", position_difference)
            
            if abs(position_difference) <= stop_threshold and count != 0:
                print("# Second End Found at = ", current_position, "Position difference = ", position_difference)
                previous_position = current_position
                right_end_found = 1
                break
            else:
                previous_position = current_position
                count = count + 1

            # Apply current  to make motor move 
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(-critical_current_codes, 0)) # Write to Torque Target Register.
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
            time.sleep(0.15)
            end_time = time.time()
            timeout = end_time - start_time

        if right_end_found == 1:
            # Get second end encoder position
            SecondEndPosition = current_position
        else:
            print("# First End Not Found - Assign a predetermined value to be constrain")
            SecondEndPosition = 0

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)

        max_left_num_of_turn, max_right_num_of_turn, FirstEndPosition, SecondEndPosition = self.set_constrains(left_end_found, right_end_found, FirstEndPosition, SecondEndPosition)

        print("left max turn is ", max_left_num_of_turn, "right max turn is ", max_right_num_of_turn)

        return max_left_num_of_turn, max_right_num_of_turn, FirstEndPosition, SecondEndPosition
    

    def move_to_final(self, target_position, max_left_num_of_turn, max_right_num_of_turn, FirstEndPosition, SecondEndPosition, critical_current_codes, stop_threshold=0.1):

        done = 0 

        if abs(max_left_num_of_turn) == abs(max_right_num_of_turn):
            target_position = target_position
        else:
            target_position = target_position + abs(max_left_num_of_turn - max_right_num_of_turn)/2

        if target_position <= max_left_num_of_turn and target_position >= max_right_num_of_turn:
            # Variables initialization
            target_current_codes = 0
            PositionOffset = 0
            currentPosition = 0 # num of turn
            start_time = 0
            end_time = 0
            timeout = 0 
            
            # Sleep for a second
            time.sleep(1)

            # Maybe think about mapping twice 
            # 1. mapping to get scaling factor for critical current
            # 2. mapping to get actually target current codes
            # target_current_range = [0.5*critical_current_codes, critical_current_codes]
            # offset_range = [0,abs(initial_position-target_position)]
        
            start_time = time.time()
            while timeout < 120:

                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
                self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
                time.sleep(0.4)

                # Find Current position in turn count
                if abs(max_left_num_of_turn) == abs(max_right_num_of_turn):
                    currentPosition = self.mapping([FirstEndPosition, SecondEndPosition],[abs(max_left_num_of_turn - max_right_num_of_turn)/2, -abs(max_left_num_of_turn - max_right_num_of_turn)/2],self.pos_average())
                else:
                    currentPosition = self.mapping([FirstEndPosition, SecondEndPosition],[max_left_num_of_turn, max_right_num_of_turn],self.pos_average())
                
                # Calcuate the position differece between current position and middle position of sldide
                PositionOffset = target_position - currentPosition

                # Figure out the moving direction by the sign of the difference between current position and middle position
                if abs(PositionOffset) <= stop_threshold:
                    # Load is already in the middle
                    # Motor no need to move 
                    print(" Motor is already in the middle ")
                    done = 1
                    break

                elif PositionOffset > stop_threshold:
                    # The sign of target current is positive 
                    target_current_codes = critical_current_codes
                else:
                    # The sign of target current is negative 
                    target_current_codes = -critical_current_codes
                
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.05)
                end_time = time.time()
                timeout = end_time - start_time

            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(5)
        else:
            print(" The input target position is out of boundary, please give a valid value")

        return done


    def move_to(self, target_position, FirstEndPosition, SecondEndPosition, critical_current_codes, stop_threshold=10000):

        done = 0 

        if target_position <= FirstEndPosition and target_position >= SecondEndPosition:
        # Variables initialization
            initial_position = 0 
            target_current_codes = 0
            PositionOffset = 0
            currentPosition = 0
            start_time = 0
            end_time = 0
            timeout = 0 

            initial_position = self.pos_average()

            # Sleep for a second
            time.sleep(1)

            # Maybe think about mapping twice 
            # 1. mapping to get scaling factor for critical current
            # 2. mapping to get actually target current codes
            target_current_range = [0.5*critical_current_codes, critical_current_codes]
            offset_range = [0,abs(initial_position-target_position)]
        
            start_time = time.time()
            while timeout < 120:

                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
                self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
                time.sleep(0.4)

                # Find Current position
                currentPosition = self.pos_average()
                
                # Calcuate the position differece between current position and middle position of sldide
                PositionOffset = target_position - currentPosition

                # Figure out the moving direction by the sign of the difference between current position and middle position
                if abs(PositionOffset) <= stop_threshold:
                    # Load is already in the middle
                    # Motor no need to move 
                    print(" Motor is already in the middle ")
                    done = 1
                    break

                elif PositionOffset > stop_threshold:
                    # The sign of target current is positive 
                    target_current_codes = int(self.mapping(offset_range, target_current_range, PositionOffset))
                    print(" The sign of target current is positive and position offset is ", PositionOffset, "target_current_codes are ", target_current_codes)
                else:
                    # The sign of target current is negative 
                    target_current_codes = -int(self.mapping(offset_range, target_current_range, -PositionOffset))
                    print(" The sign of target current is negative and position offset is ", PositionOffset, "target_current_codes are ", target_current_codes)

                
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
                end_time = time.time()
                timeout = end_time - start_time

            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(5)
        else:
            print(" The input target position is out of boundary, please give a valid value")

        return done
    
    def set_constrains(self, left_end_found, right_end_found, FirstEndPosition, SecondEndPosition, turn_constrains = 5):

        # Variables Reset 
        left_position_difference = 0
        right_position_difference = 0
        currentPosition = 0
        max_left_num_of_turn = 0
        max_right_num_of_turn = 0

        currentPosition = self.pos_average()

        # Sleep for a second
        time.sleep(1)

        if left_end_found == 0 and right_end_found == 0:

            max_left_num_of_turn = turn_constrains
            FirstEndPosition = currentPosition + turn_constrains*2*math.pi*521519
            max_right_num_of_turn = -turn_constrains
            SecondEndPosition = currentPosition -turn_constrains*2*math.pi*521519

        elif left_end_found == 0 and right_end_found != 0:

            right_position_difference = currentPosition - SecondEndPosition
            max_left_num_of_turn = turn_constrains
            FirstEndPosition = currentPosition + turn_constrains*2*math.pi*521519
            max_right_num_of_turn = -abs(right_position_difference)/(2*math.pi*521519)

        elif left_end_found != 0 and right_end_found == 0:

            left_position_difference = currentPosition - FirstEndPosition
            max_right_num_of_turn = -turn_constrains
            max_left_num_of_turn = abs(left_position_difference)/(2*math.pi*521519)
            SecondEndPosition = currentPosition - turn_constrains*2*math.pi*521519
        
        else:
            left_position_difference = currentPosition - FirstEndPosition
            right_position_difference = currentPosition - SecondEndPosition
            max_left_num_of_turn = abs(left_position_difference)/(2*math.pi*521519)
            max_right_num_of_turn = -abs(right_position_difference)/(2*math.pi*521519)
        
        return max_left_num_of_turn, max_right_num_of_turn, FirstEndPosition, SecondEndPosition


    def mapping(self, From, To, input):
        output = (To[1]-To[0])/(From[1]-From[0])*(input-From[0])+ To[0]
        return output

    def move_to_middle(self, FirstEndPosition, SecondEndPosition, target_current_factor, max_torque_codes, stop_threshold=8000):

        # Feedback selection
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000003)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
        self.mcchelp.mcc_write(MCC.PID_TORQUE_COEFF, to_register32(self.kp, self.ki)) # Write Torque PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_FLUX_COEFF,   to_register32(self.kp, self.ki)) # Write Flux PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_P, self.norm_p) # Enable/Disable Torque/Flux P Normalization - 1 = Q0.16, 0 = Q8.8.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_I, self.norm_i) # Enable/Disable Torque/Flux I Normalization - 1 = Q0.16, 0 = Q8.8.

       # Variables initialization 
        current_factor = 0
        target_current_codes = 0
        PositionOffset = 0
        currentPosition = 0
       
        # Find encoder value when motor is in the middle of linear slide 
        SlideMiddlePosition = (FirstEndPosition + SecondEndPosition)/2

        current_factor_range = [0.6*target_current_factor, target_current_factor]
        offset_range = [0,abs((FirstEndPosition - SecondEndPosition)/2)]
    

        while True:

            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4)

            # Find Current position
            currentPosition = self.pos_average()
            
            # Calcuate the position differece between current position and middle position of sldide
            PositionOffset = SlideMiddlePosition - currentPosition

            # Figure out the moving direction by the sign of the difference between current position and middle position
            if abs(PositionOffset) <= stop_threshold:
                # Load is already in the middle
                # Motor no need to move 
                print(" Motor is already in the middle ")
                break

            elif PositionOffset > stop_threshold:
                # The sign of target current is positive 
                current_factor = self.mapping(offset_range, current_factor_range, PositionOffset)
                target_current_codes = int(current_factor*max_torque_codes)
                print(" The sign of target current is positive and position offset is ", PositionOffset, "target_current_codes are ", target_current_codes)
            else:
                # The sign of target current is negative 
                current_factor = self.mapping(offset_range, current_factor_range, -PositionOffset)
                target_current_codes = -int(current_factor*max_torque_codes)
                print(" The sign of target current is negative and position offset is ", PositionOffset, "target_current_codes are ", target_current_codes)

            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)


        



    def detect_end_stop(self, max_torque_codes, start_threshold=300000, stop_threshold=3000): # Note: a velocity_threshold of 0.1 rads/sec ~ 0.1rpm
        # ABN encoder configuration (Init encoder (mode 0))
        # Comment out for test
        # self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000038)
        # self.mcchelp.mcc_write(MCC.ABN_PHI_E_OFFSET, 0x00000000)
        # self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000001)
        # self.mcchelp.mcc_write(MCC.PHI_EXT, 0x00000000)
        # self.mcchelp.mcc_write(MCC.VOLTAGE_EXT, 0x000003E8)
        # time.sleep(1)
        # self.mcchelp.mcc_write(MCC.ABN_COUNT, 0x00000000)
        # time.sleep(1)

        # Feedback selection
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000003)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
        self.mcchelp.mcc_write(MCC.PID_TORQUE_COEFF, to_register32(self.kp, self.ki)) # Write Torque PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_FLUX_COEFF,   to_register32(self.kp, self.ki)) # Write Flux PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_P, self.norm_p) # Enable/Disable Torque/Flux P Normalization - 1 = Q0.16, 0 = Q8.8.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_I, self.norm_i) # Enable/Disable Torque/Flux I Normalization - 1 = Q0.16, 0 = Q8.8.

        # Variables initialization 
        current_factor = 0
        target_current_codes = 0
        position_difference = 0
        previous_position = 0 
        current_position = 0
        count = 0

        time.sleep(1)
        print("# Find target current codes. Part 0......")
        while True:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4)

            # Get encoder data from current position
            current_position = self.pos_average()

            # Calculate the absolute difference betweeb previous and current position
            position_difference = abs(current_position - previous_position)
            print("Current factor = ", current_factor , "Current Encoder Position = ", current_position, "Position Difference = ", position_difference)
            if abs(position_difference) > start_threshold and count != 0:
                print("# Find target current codes are = ", target_current_codes, "Position difference = ", position_difference)
                previous_position = current_position
                break
            else:
                previous_position = current_position
                # Increase current codes by 1% per loop
                current_factor = current_factor + 0.01
                target_current_codes = int(current_factor*max_torque_codes)
                count = count + 1


            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break

        target_current_factor = current_factor

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)



        # Variables initialization 
        position_difference = 0
        previous_position = 0 
        current_position = 0
        FirstEndPosition = 0
        count = 0

        # Calcuate the target current which is going to apply to the motor
        target_current_codes = target_current_codes

        # record the initial position from encoder( not sure if we need to scale it atm)
        initial_position= self.pos_average()

        # Assign initial position to previous position
        previous_position = initial_position

        # Sleep for a second
        time.sleep(1)
        print("# Starting End Stop Detection. Part 1......")
        while True:
            # Set flux and velocity registers to zero and motor stop
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4) # Wait motor to be stopped

            # Get encoder data from current position
            current_position = self.pos_average()

            # Calculate the absolute difference betweeb previous and current position
            position_difference = abs(current_position - previous_position)
            print("Current Encoder Position = ", current_position, "Position Difference = ", position_difference)
            
            if abs(position_difference) <= stop_threshold and count != 0:
                print("# First End Found at = ", current_position, "Position difference = ", position_difference)
                previous_position = current_position
                break
            else:
                previous_position = current_position
                count = count + 1

            # Apply current  to make motor move 
            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break

        # Get first end encoder position
        FirstEndPosition = current_position

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)

        # Variables Reset 
        position_difference = 0
        current_position = 0
        SecondEndPosition = 0
        count = 0

        # Calcuate the target current which is going to apply to the motor(Opposite direction)
        target_current_codes = -target_current_codes

        # Sleep for a second
        time.sleep(1)

        print("# Starting End Stop Detection. Part 2.......")
        while True:
            # Set flux and velocity registers to zero and motor stop
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4) # Wait motor to be stopped

            # Get encoder data from current position
            current_position = self.pos_average()

            # Calculate the absolute difference betweeb previous and current position
            position_difference = abs(current_position - previous_position)

            print("Current Encoder Position = ", current_position, "Position Difference = ", position_difference)
            
            if abs(position_difference) <= stop_threshold and count != 0:
                print("# Second End Found at = ", current_position, "Position difference = ", position_difference)
                previous_position = current_position
                break
            else:
                previous_position = current_position
                count = count + 1

            # Apply current  to make motor move 
            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break

        # Get second end encoder position
        SecondEndPosition = current_position

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)

        return FirstEndPosition, SecondEndPosition, initial_position, target_current_factor



    def detect_end_stop_with_constrains(self, max_torque_codes, start_threshold=300000, stop_threshold=3000): # Note: a velocity_threshold of 0.1 rads/sec ~ 0.1rpm
        # ABN encoder configuration (Init encoder (mode 0))
        # Comment out for test
        # self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000038)
        # self.mcchelp.mcc_write(MCC.ABN_PHI_E_OFFSET, 0x00000000)
        # self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000001)
        # self.mcchelp.mcc_write(MCC.PHI_EXT, 0x00000000)
        # self.mcchelp.mcc_write(MCC.VOLTAGE_EXT, 0x000003E8)
        # time.sleep(1)
        # self.mcchelp.mcc_write(MCC.ABN_COUNT, 0x00000000)
        # time.sleep(1)

        # Feedback selection
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000003)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
        self.mcchelp.mcc_write(MCC.PID_TORQUE_COEFF, to_register32(self.kp, self.ki)) # Write Torque PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_FLUX_COEFF,   to_register32(self.kp, self.ki)) # Write Flux PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_P, self.norm_p) # Enable/Disable Torque/Flux P Normalization - 1 = Q0.16, 0 = Q8.8.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_I, self.norm_i) # Enable/Disable Torque/Flux I Normalization - 1 = Q0.16, 0 = Q8.8.

        # Variables initialization 
        current_factor = 0
        target_current_codes = 0
        position_difference = 0
        previous_position = 0 
        current_position = 0
        count = 0

        time.sleep(1)
        print("# Find target current codes. Part 0......")
        while True:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4)

            # Get encoder data from current position
            current_position = self.pos_average()

            # Calculate the absolute difference betweeb previous and current position
            position_difference = abs(current_position - previous_position)
            print("Current factor = ", current_factor , "Current Encoder Position = ", current_position, "Position Difference = ", position_difference)
            if abs(position_difference) > start_threshold and count != 0:
                print("# Find target current codes are = ", target_current_codes, "Position difference = ", position_difference)
                previous_position = current_position
                break
            elif count % 2 == 0:
                previous_position = current_position
                # Increase current codes by 1% per loop
                current_factor = current_factor + 0.01
                target_current_codes = int(current_factor*max_torque_codes)
                count = count + 1
            else:
                previous_position = current_position
                # Increase current codes by 1% per loop
                current_factor = current_factor + 0.01
                target_current_codes = -int(current_factor*max_torque_codes)
                count = count + 1


            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break

        target_current_factor = current_factor

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)



        # Variables initialization 
        position_difference = 0
        previous_position = 0 
        current_position = 0
        FirstEndPosition = 0
        count = 0

        # Calcuate the target current which is going to apply to the motor
        target_current_codes = target_current_codes

        # record the initial position from encoder( not sure if we need to scale it atm)
        initial_position= self.pos_average()

        # Assign initial position to previous position
        previous_position = initial_position

        # Sleep for a second
        time.sleep(1)
        print("# Starting End Stop Detection. Part 1......")
        while True:
            # Set flux and velocity registers to zero and motor stop
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4) # Wait motor to be stopped

            # Get encoder data from current position
            current_position = self.pos_average()

            # Calculate the absolute difference betweeb previous and current position
            position_difference = abs(current_position - previous_position)
            print("Current Encoder Position = ", current_position, "Position Difference = ", position_difference)
            
            if abs(position_difference) <= stop_threshold and count != 0:
                print("# First End Found at = ", current_position, "Position difference = ", position_difference)
                previous_position = current_position
                break
            else:
                previous_position = current_position
                count = count + 1

            # Apply current  to make motor move 
            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break

        # Get first end encoder position
        FirstEndPosition = current_position

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)

        # Variables Reset 
        position_difference = 0
        current_position = 0
        SecondEndPosition = 0
        count = 0

        # Calcuate the target current which is going to apply to the motor(Opposite direction)
        target_current_codes = -target_current_codes

        # Sleep for a second
        time.sleep(1)

        print("# Starting End Stop Detection. Part 2.......")
        while True:
            # Set flux and velocity registers to zero and motor stop
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4) # Wait motor to be stopped

            # Get encoder data from current position
            current_position = self.pos_average()

            # Calculate the absolute difference betweeb previous and current position
            position_difference = abs(current_position - previous_position)

            print("Current Encoder Position = ", current_position, "Position Difference = ", position_difference)
            
            if abs(position_difference) <= stop_threshold and count != 0:
                print("# Second End Found at = ", current_position, "Position difference = ", position_difference)
                previous_position = current_position
                break
            else:
                previous_position = current_position
                count = count + 1

            # Apply current  to make motor move 
            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break

        # Get second end encoder position
        SecondEndPosition = current_position

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)

        return FirstEndPosition, SecondEndPosition, initial_position, target_current_factor

    """
    ######### Test function for End stop detection #########################
    def detect_end_stop_yk(self, max_torque_codes, start_threshold=300000, stop_threshold=3000): # Note: a velocity_threshold of 0.1 rads/sec ~ 0.1rpm
        # ABN encoder configuration (Init encoder (mode 0))
        # Comment out for test
        # self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000038)
        # self.mcchelp.mcc_write(MCC.ABN_PHI_E_OFFSET, 0x00000000)
        # self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000001)
        # self.mcchelp.mcc_write(MCC.PHI_EXT, 0x00000000)
        # self.mcchelp.mcc_write(MCC.VOLTAGE_EXT, 0x000003E8)
        # time.sleep(1)
        # self.mcchelp.mcc_write(MCC.ABN_COUNT, 0x00000000)
        # time.sleep(1)

        # Feedback selection
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000003)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
        self.mcchelp.mcc_write(MCC.PID_TORQUE_COEFF, to_register32(self.kp, self.ki)) # Write Torque PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_FLUX_COEFF,   to_register32(self.kp, self.ki)) # Write Flux PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_P, self.norm_p) # Enable/Disable Torque/Flux P Normalization - 1 = Q0.16, 0 = Q8.8.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_I, self.norm_i) # Enable/Disable Torque/Flux I Normalization - 1 = Q0.16, 0 = Q8.8.

        # Variables initialization 
        current_factor = 0.0
        target_current_codes = 0
        position_difference = 0
        previous_position = 0 
        current_position = 0
        count = 0
        current_posRMSD = 0
        current_posDiff = 0

        time.sleep(1)
        print("# Find target current codes. Part 0......")
        while True:
            current_factor = current_factor + 0.01
            target_current_codes = int(current_factor*max_torque_codes)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4)

            current_position, current_posRMSD, current_posDiff = self.pos_average_yk()
            # position_difference = abs(current_position) - abs(previous_position)
            position_difference = abs(abs(current_position) - abs(previous_position))
            print("Current Encoder Position = ", current_position, "Position Difference = ", position_difference,"Current Position RMSD Value = ", current_posRMSD, "Current Position difference = ", current_posDiff)
            if abs(position_difference) > start_threshold and count != 0:
                print("# First current factor is = ", current_factor, "Position difference = ", position_difference)
                previous_position = current_position
                break
            else:
                previous_position = current_position
                count = count + 1

            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break


        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)



        # Record the Encoder position and relative step difference
        Encoder_position_f = []
        Step_difference_f = []
        Velocity_f = []

        Encoder_position_b = []
        Step_difference_b = []
        Velocity_b = []

        # Variables initialization 
        target_current_codes = 0
        position_difference = 0
        previous_position = 0 
        current_position = 0
        FirstEndPosition = 0
        count = 0
        current_posRMSD = 0
        current_posDiff = 0

        # Calcuate the target current which applies for the end stop detection
        target_current_codes = int(current_factor*max_torque_codes)

        # record the initial position from encoder( not sure if we need to scale it atm)
        initial_position, current_posRMSD, current_posDiff = self.pos_average_yk()

        previous_position = initial_position

        time.sleep(1)
        print("# Starting End Stop Detection. Part 1......")
        while True:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4)

            current_position, current_posRMSD, current_posDiff = self.pos_average_yk()
            Velocity_f.append(self.vel_data_yk())
            # position_difference = abs(current_position) - abs(previous_position)
            position_difference = abs(abs(current_position) - abs(previous_position))
            Encoder_position_f.append(previous_position)
            Step_difference_f.append(-position_difference)
            print("Current Encoder Position = ", current_position, "Position Difference = ", position_difference,"Current Position RMSD Value = ", current_posRMSD, "Current Position difference = ", current_posDiff)
            if abs(position_difference) <= stop_threshold and count != 0:
                print("# First End Found at = ", current_position, "Position difference = ", position_difference)
                previous_position = current_position
                break
            else:
                previous_position = current_position
                count = count + 1

            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break


        FirstEndPosition = current_position

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)

        # Variables Reset 
        position_difference = 0
        #previous_position = 0 
        current_position = 0
        SecondEndPosition = 0
        target_current_codes = 0
        count = 0
        current_posRMSD = 0
        current_posDiff = 0

        # Calcuate the target current which applies for the end stop detection
        target_current_codes = -int(current_factor*max_torque_codes)

        #previous_position = self.pos_average_yk()

        time.sleep(1)

        print("# Starting End Stop Detection. Part 2.......")
        while True:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.4)

            current_position, current_posRMSD, current_posDiff = self.pos_average_yk()
            Velocity_b.append(self.vel_data_yk())
            # position_difference = abs(current_position) - abs(previous_position)
            position_difference = abs(abs(current_position) - abs(previous_position))
            Encoder_position_b.append(previous_position)
            Step_difference_b.append(-position_difference)
            print("Current Encoder Position = ", current_position, "Position Difference = ", position_difference,"Current Position RMSD Value = ", current_posRMSD, "Current Position difference = ", current_posDiff)
            if abs(position_difference) <= stop_threshold and count != 0:
                print("# Second End Found at = ", current_position, "Position difference = ", position_difference)
                previous_position = current_position
                break
            else:
                previous_position = current_position
                count = count + 1

            if abs(target_current_codes) < max_torque_codes:
                self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_current_codes, 0)) # Write to Torque Target Register.
                self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
                time.sleep(0.1)
            else:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_current_codes)
                break


        SecondEndPosition = current_position

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)

        return FirstEndPosition, SecondEndPosition, initial_position, Encoder_position_f, Step_difference_f, Encoder_position_b, Step_difference_b, Velocity_f, Velocity_b
    """


    def find_static_friction_range_simple(self, max_torque_codes, velocity_threshold=0.2, torque_inc_current=0.01): # Note: a velocity_threshold of 0.1 rads/sec ~ 0.1rpm
        positive_static_friction_torque_codes = 0
        negative_static_friction_torque_codes = 0
        # ABN encoder configuration (Init encoder (mode 0))
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000038)
        self.mcchelp.mcc_write(MCC.ABN_PHI_E_OFFSET, 0x00000000)
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000001)
        self.mcchelp.mcc_write(MCC.PHI_EXT, 0x00000000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT, 0x000003E8)
        time.sleep(1)
        self.mcchelp.mcc_write(MCC.ABN_COUNT, 0x00000000)
        time.sleep(1)

        # Feedback selection
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000003)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
        self.mcchelp.mcc_write(MCC.PID_TORQUE_COEFF, to_register32(self.kp, self.ki)) # Write Torque PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_FLUX_COEFF,   to_register32(self.kp, self.ki)) # Write Flux PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_P, self.norm_p) # Enable/Disable Torque/Flux P Normalization - 1 = Q0.16, 0 = Q8.8.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_I, self.norm_i) # Enable/Disable Torque/Flux I Normalization - 1 = Q0.16, 0 = Q8.8.

        target_torque_codes = 0
        velocity_data = 0
        torque_inc_codes = int(torque_inc_current/self.TM01Setup.get_flux_scaling())
        print("Static Friction Test - Torque Increment = ",torque_inc_current,"A / ",torque_inc_codes," Codes.")
        velocity_limit = velocity_threshold
        time.sleep(1)
        print("# Starting Positive Static Friction Torque Test.")
        while abs(velocity_data) <= velocity_limit:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.2)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_torque_codes, 0)) # Write to Torque Target Register.
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
            if abs(target_torque_codes) >= max_torque_codes:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_torque_codes)
                break
            else:
                print("Target Torque Codes = ",target_torque_codes)
                # time.sleep(0.1) # Wait for loop to settle.
                torque_data, flux_data, velocity_data = self.average_data()
                if abs(velocity_data) >= velocity_limit:
                    print("# Positive Starting Static Friction Torque Found at = ", torque_data, "Amps (", target_torque_codes, "codes) - Velocity = ",velocity_data, "rads/sec.")
                    positive_static_friction_torque_codes_max = target_torque_codes
                    positive_static_friction_velocity_max = velocity_data
                    positive_static_friction_torque_max = torque_data
                    break
                else:
                    target_torque_codes = target_torque_codes + torque_inc_codes

        positive_static_friction_torque_codes = positive_static_friction_torque_codes_max
        positive_static_friction_torque = positive_static_friction_torque_max
        positive_static_friction_velocity = positive_static_friction_velocity_max
        print("# Positive Static Friction Torque Found at = ", positive_static_friction_torque, "Amps (", positive_static_friction_torque_codes, "codes) - Velocity = ",positive_static_friction_velocity, "rads/sec.")


        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)

        target_torque_codes = 0
        velocity_data = 0
        torque_inc_codes = int(torque_inc_current/self.TM01Setup.get_flux_scaling())
        print("Static Friction Test - Torque Increment = ",torque_inc_current,"A / ",torque_inc_codes," Codes.")
        velocity_limit = velocity_threshold
        time.sleep(1)
        print("# Starting Negative Static Friction Torque Test.")
        while abs(velocity_data) <= velocity_limit:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.2)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_torque_codes, 0)) # Write to Torque Target Register.
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
            if abs(target_torque_codes) >= max_torque_codes:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_torque_codes)
                break
            else:
                print("Target Torque Codes = ",target_torque_codes)
                # time.sleep(0.1) # Wait for loop to settle.
                torque_data, flux_data, velocity_data = self.average_data()
                if abs(velocity_data) >= velocity_limit:
                    print("# Negative Starting Static Friction Torque Found at = ", torque_data, "Amps (", target_torque_codes, "codes) - Velocity = ",velocity_data, "rads/sec.")
                    negative_static_friction_torque_codes_min = target_torque_codes
                    negative_static_friction_velocity_min = velocity_data
                    negative_static_friction_torque_min = torque_data
                    break
                else:
                    target_torque_codes = target_torque_codes - torque_inc_codes



        negative_static_friction_torque_codes = negative_static_friction_torque_codes_min
        negative_static_friction_torque = negative_static_friction_torque_min
        negative_static_friction_velocity = negative_static_friction_velocity_min
        print("# Negative Static Friction Torque Found at = ", negative_static_friction_torque, "Amps (", negative_static_friction_torque_codes, "codes) - Velocity = ",negative_static_friction_velocity, "rads/sec.")


        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, 0x00000000)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)

        print("#")
        print("# Static Friction Estimation- Summary.")
        print("#")
        print("# Positive Static Friction Torque Found at = ", positive_static_friction_torque, "Amps (", positive_static_friction_torque_codes, "codes) - Velocity = ",positive_static_friction_velocity, "rads/sec.")
        print("# Negative Static Friction Torque Found at = ", negative_static_friction_torque, "Amps (", negative_static_friction_torque_codes, "codes) - Velocity = ",negative_static_friction_velocity, "rads/sec.")
        print("#")

        return positive_static_friction_torque_codes, negative_static_friction_torque_codes



    def find_static_friction_range(self, max_torque_codes, velocity_threshold=0.1): # Note: a velocity_threshold of 0.1 rads/sec ~ 0.1rpm
        positive_static_friction_torque_codes = 0
        negative_static_friction_torque_codes = 0
        # ABN encoder configuration (Init encoder (mode 0))
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000038)
        self.mcchelp.mcc_write(MCC.ABN_PHI_E_OFFSET, 0x00000000)
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000001)
        self.mcchelp.mcc_write(MCC.PHI_EXT, 0x00000000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT, 0x000003E8)
        time.sleep(1)
        self.mcchelp.mcc_write(MCC.ABN_COUNT, 0x00000000)
        time.sleep(1)

        # Feedback selection
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000003)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
        self.mcchelp.mcc_write(MCC.PID_TORQUE_COEFF, to_register32(self.kp, self.ki)) # Write Torque PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_FLUX_COEFF,   to_register32(self.kp, self.ki)) # Write Flux PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_P, self.norm_p) # Enable/Disable Torque/Flux P Normalization - 1 = Q0.16, 0 = Q8.8.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_I, self.norm_i) # Enable/Disable Torque/Flux I Normalization - 1 = Q0.16, 0 = Q8.8.

        target_torque_codes = 0
        velocity_data = 0
        torque_inc_codes= 1
        velocity_limit = velocity_threshold
        time.sleep(1)
        print("# Starting Positive Static Friction Torque Test.")
        while abs(velocity_data) <= velocity_limit:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.2)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_torque_codes, 0)) # Write to Torque Target Register.
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
            if abs(target_torque_codes) >= max_torque_codes:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_torque_codes)
                break
            else:
                print("Target Torque Codes = ",target_torque_codes)
                # time.sleep(0.1) # Wait for loop to settle.
                torque_data, flux_data, velocity_data = self.average_data()
                if abs(velocity_data) >= velocity_limit:
                    print("# Positive Starting Static Friction Torque Found at = ", torque_data, "Amps (", target_torque_codes, "codes) - Velocity = ",velocity_data, "rads/sec.")
                    positive_static_friction_torque_codes_max = target_torque_codes
                    positive_static_friction_velocity_max = velocity_data
                    positive_static_friction_torque_max = torque_data
                    break
                else:
                    target_torque_codes = target_torque_codes + torque_inc_codes

        while abs(velocity_data) >= velocity_limit:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.2)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_torque_codes, 0)) # Write to Torque Target Register.
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
            if abs(target_torque_codes) >= max_torque_codes:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_torque_codes)
                break
            else:
                print("Target Torque Codes = ",target_torque_codes)
                # time.sleep(0.1) # Wait for loop to settle.
                torque_data, flux_data, velocity_data = self.average_data()
                if abs(velocity_data) <= velocity_limit:
                    print("# Positive Stoping Static Friction Torque Found at = ", torque_data, "Amps (", target_torque_codes, "codes) - Velocity = ",velocity_data, "rads/sec.")
                    positive_static_friction_torque_codes_min = target_torque_codes
                    target_torque_codes = 0
                    positive_static_friction_velocity_min = velocity_data
                    velocity_data = 0.0
                    positive_static_friction_torque_min = torque_data
                    torque_data = 0.0
                    flux_data = 0.0
                    break
                else:
                    target_torque_codes = target_torque_codes - torque_inc_codes

        positive_static_friction_torque_codes = int((positive_static_friction_torque_codes_max + positive_static_friction_torque_codes_min)/2)
        positive_static_friction_torque = (positive_static_friction_torque_max+positive_static_friction_torque_min)/2
        positive_static_friction_velocity = (positive_static_friction_velocity_max+positive_static_friction_velocity_min)/2
        print("# Positive Static Friction Torque Found at = ", positive_static_friction_torque, "Amps (", positive_static_friction_torque_codes, "codes) - Velocity = ",positive_static_friction_velocity, "rads/sec.")


        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(0, 0)) # Write to Torque Target Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
        time.sleep(5)

        print("# Starting Negative Static Friction Torque Test.")

        while abs(velocity_data) <= velocity_limit:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.2)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_torque_codes, 0)) # Write to Torque Target Register.
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
            if abs(target_torque_codes) >= max_torque_codes:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_torque_codes)
                break
            else:
                print("Target Torque Codes = ",target_torque_codes)
                # time.sleep(0.1) # Wait for loop to settle.
                torque_data, flux_data, velocity_data = self.average_data()
                if abs(velocity_data) >= velocity_limit:
                    print("# Negative Starting Static Friction Torque Found at = ", torque_data, "Amps (", target_torque_codes, "codes) - Velocity = ",velocity_data, "rads/sec.")
                    negative_static_friction_torque_codes_min = target_torque_codes
                    negative_static_friction_velocity_min = velocity_data
                    negative_static_friction_torque_min = torque_data
                    break
                else:
                    target_torque_codes = target_torque_codes - torque_inc_codes

        while abs(velocity_data) >= velocity_limit:
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to STOPPED.
            time.sleep(0.2)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_torque_codes, 0)) # Write to Torque Target Register.
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
            if abs(target_torque_codes) >= max_torque_codes:
                print("WARNING: Maximum Torque Target is Exceeded - Target = ", target_torque_codes)
                break
            else:
                print("Target Torque Codes = ",target_torque_codes)
                # time.sleep(0.1) # Wait for loop to settle.
                torque_data, flux_data, velocity_data = self.average_data()
                if abs(velocity_data) <= velocity_limit:
                    print("# Negative Stoping Friction Torque Found at = ", torque_data, "Amps (", target_torque_codes, "codes) - Velocity = ",velocity_data, "rads/sec.")
                    negative_static_friction_torque_codes_max = target_torque_codes
                    target_torque_codes = 0
                    negative_static_friction_velocity_max = velocity_data
                    velocity_data = 0.0
                    negative_static_friction_torque_max = torque_data
                    torque_data = 0.0
                    flux_data = 0.0
                    break
                else:
                    target_torque_codes = target_torque_codes + torque_inc_codes

        negative_static_friction_torque_codes = int((negative_static_friction_torque_codes_max + negative_static_friction_torque_codes_min)/2)
        negative_static_friction_torque = (negative_static_friction_torque_max+negative_static_friction_torque_min)/2
        negative_static_friction_velocity = (negative_static_friction_velocity_max+negative_static_friction_velocity_min)/2
        print("# Negative Static Friction Torque Found at = ", negative_static_friction_torque, "Amps (", negative_static_friction_torque_codes, "codes) - Velocity = ",negative_static_friction_velocity, "rads/sec.")


        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, 0x00000000)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)

        print("#")
        print("# Static Friction Estimation- Summary.")
        print("#")
        print("# Positive Static Friction Torque Found at = ", positive_static_friction_torque, "Amps (", positive_static_friction_torque_codes, "codes) - Velocity = ",positive_static_friction_velocity, "rads/sec.")
        print("# Negative Static Friction Torque Found at = ", negative_static_friction_torque, "Amps (", negative_static_friction_torque_codes, "codes) - Velocity = ",negative_static_friction_velocity, "rads/sec.")
        print("#")

        return positive_static_friction_torque_codes, negative_static_friction_torque_codes





    def run_torque_profile(self):
        # ABN encoder configuration (Init encoder (mode 0))
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000038)
        self.mcchelp.mcc_write(MCC.ABN_PHI_E_OFFSET, 0x00000000)
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000001)
        self.mcchelp.mcc_write(MCC.PHI_EXT, 0x00000000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT, 0x000003E8)
        time.sleep(1)
        self.mcchelp.mcc_write(MCC.ABN_COUNT, 0x00000000)
        time.sleep(1)

        # Feedback selection
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000003)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
        self.mcchelp.mcc_write(MCC.PID_TORQUE_COEFF, to_register32(self.kp, self.ki)) # Write Torque PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_FLUX_COEFF,   to_register32(self.kp, self.ki)) # Write Flux PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_P, self.norm_p) # Enable/Disable Torque/Flux P Normalization - 1 = Q0.16, 0 = Q8.8.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_I, self.norm_i) # Enable/Disable Torque/Flux I Normalization - 1 = Q0.16, 0 = Q8.8.

        self.data = OrderedDict()
        self.data["torque"] = []
        self.data["flux"] = []
        self.data["velocity"] = []
        self.headers = ["torque", "flux", "velocity"]

        for target_torque in range( self.MinTorque, self.MaxTorque, self.TorqueStep ):
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL, 0)
            time.sleep(1)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, to_register32(target_torque, 0)) # Write to Torque Target Register.
            print("Target Torque = ",target_torque)
            time.sleep(10) # Wait for loop to settle.
            torque_data, flux_data, velocity_data = self.average_data()
            self.data["torque"].append(torque_data)
            self.data["flux"].append(flux_data)
            self.data["velocity"].append(velocity_data)

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET, 0x00000000)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)

        data_len = len(self.data[list(self.data)[0]])

        print("Data = ", self.data)

        with open(self.csvfilename, 'w', newline='') as f:
            writer = csv.writer(f, delimiter=";")
            writer.writerow(self.headers)
            for i in range(data_len):
                writer.writerow([self.data[x][i] for x in self.headers])
        print("Written file", self.csvfilename)


    def average_data(self):
        torque_average = 0
        flux_average = 0
        velocity_average = 0
        NumSamples = 64
        for i in range(NumSamples):
            torque_average = torque_average + self.mcchelp.mcc_read(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL)
            flux_average = flux_average + self.mcchelp.mcc_read(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL)
            velocity_average = velocity_average + self.mcchelp.mcc_read(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL)
        torque_average = torque_average/NumSamples
        flux_average = flux_average/NumSamples
        velocity_average = velocity_average/NumSamples
        # torque_scaled = torque_average*self.torque_scaling
        # flux_scaled = flux_average*self.flux_scaling
        # velocity_scaled = velocity_average*self.velocity_scaling
        # voltage_scaled = self.TM01Setup.get_SI_value('MCC.VOLTAGE_EXT.UQ', voltage_average)
        flux_scaled = self.TM01Setup.get_SI_value('MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL', flux_average)
        torque_scaled = self.TM01Setup.get_SI_value('MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL', torque_average)
        velocity_scaled = self.TM01Setup.get_SI_value('MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL', velocity_average)
        print("# Average Torque   = ", torque_scaled," Amps.")
        print("# Flux Torque   = ", flux_scaled," Amps.")
        print("# Average Velocity = ", velocity_scaled," Rads/s.")

        return torque_scaled, flux_scaled, velocity_scaled


    # Calcuate the average position from encoder
    def pos_average(self):
        position_average = 0
        NumSamples = 64
        for i in range(NumSamples):
            position_average = position_average + self.mcchelp.mcc_read(MCC.PID_POSITION_ACTUAL.PID_POSITION_ACTUAL)
        position_average = position_average/NumSamples
        
        return position_average
    
    # Calcuate the average velocity from encoder
    def vel_average(self):
        velocity_average = 0
        NumSamples = 64
        for i in range(NumSamples):
            velocity_average = velocity_average + self.mcchelp.mcc_read(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL)
        velocity_average = velocity_average/NumSamples
        velocity_scaled = self.TM01Setup.get_SI_value('MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL', velocity_average)
        #print("# Average Velocity = ", velocity_scaled," Rads/s.")

        return velocity_scaled
    """
    def pos_average_yk(self):
        position_diff = 0
        position_average = 0
        position_Meansquare_sum = 0
        position_eachRead = 0
        position_array = []
        NumSamples = 64
        for i in range(NumSamples):
            position_eachRead = self.mcchelp.mcc_read(MCC.PID_POSITION_ACTUAL.PID_POSITION_ACTUAL)
            position_array.append(position_eachRead)
            position_average = position_average + position_eachRead
        position_average = position_average/NumSamples
        for position in position_array:
            position_Meansquare_sum = position_Meansquare_sum + (position-position_average)**2
        position_RMSD = sqrt(position_Meansquare_sum/NumSamples)
        position_diff = max(position_array) - min(position_array)
        
        #print("# Average Position = ", position_average)
        return position_average, position_RMSD, position_diff
    
    def vel_data_yk(self):
        velocity_average = 0
        NumSamples = 64
        for i in range(NumSamples):
            velocity_average = velocity_average + self.mcchelp.mcc_read(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL)
        velocity_average = velocity_average/NumSamples
        # torque_scaled = torque_average*self.torque_scaling
        # flux_scaled = flux_average*self.flux_scaling
        # velocity_scaled = velocity_average*self.velocity_scaling
        # voltage_scaled = self.TM01Setup.get_SI_value('MCC.VOLTAGE_EXT.UQ', voltage_average)
        velocity_scaled = self.TM01Setup.get_SI_value('MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL', velocity_average)
        #print("# Average Velocity = ", velocity_scaled," Rads/s.")

        return velocity_scaled
    """

