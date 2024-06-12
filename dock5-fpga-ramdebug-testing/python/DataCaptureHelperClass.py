################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import numpy as np
import math

from TM01SystemUnitsSetup import TM01SystemUnitsSetup
from register_helpers import to_register32
from StimulusCaptureClass import StimulusCaptureClass
from ubltools.memory.tm01map._tm01_map_latest import MCC # ToDo: Clean this up

from biquad_filter_calc import calculate_biquad_filter_coefficients, Tm01, Tmc4671, LowPassFilter, AntiResonanceFilter
from TrajectoryProfileSawtoothClass import TrajectoryProfileSawtoothVelocity, TrajectoryProfileSawtoothAcceleration


class DataCaptureHelperClass: 
    def __init__(self, mySystem, myMotor, myInterface, mcchelp, myMotorTune): 
        self.mySystem = mySystem
        self.myMotor = myMotor
        self.myInterface = myInterface
        self.mcchelp = mcchelp
        self.myMotorTune = myMotorTune

        self.PWM_FREQUENCY = self.mySystem.get_pwm_frequency()

        # End stop detect parameters
        self.system_id_start_point = 0
        self.max_left_num_of_turn = 0
        self.max_right_num_of_turn = 0
        self.LeftEndPosition = 0
        self.RightEndPosition = 0

    def zero_stop_motor(self):
        # Set the Motor to STOPPER and Zero all inputs and outputs.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000) # Motor Stop.
        self.myMotor.zero_voltageext()
        self.myMotor.zero_torquefluxtarget()
        self.myMotor.zero_torquefluxactual()
        self.myMotor.zero_velocitytarget()
        self.myMotor.zero_velocityactual()
        self.myMotor.zero_positiontarget()
        self.myMotor.zero_positionactual()

    def zero_stop_motor_not_position(self):
        # Set the Motor to STOPPER and Zero all inputs and outputs.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000) # Motor Stop.
        self.myMotor.zero_voltageext()
        self.myMotor.zero_torquefluxtarget()
        self.myMotor.zero_torquefluxactual()
        self.myMotor.zero_velocitytarget()
        self.myMotor.zero_velocityactual()

    def SetCurrentVoltageLimits(self, target_voltageext_codes, max_motor_current_TM01Units): 
        self.mcchelp.mcc_write(MCC.PID_UQ_UD_LIMITS, to_register32(target_voltageext_codes,target_voltageext_codes))
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_LIMITS, to_register32(max_motor_current_TM01Units, max_motor_current_TM01Units)) 
        self.mcchelp.mcc_write(MCC.PID_VELOCITY_LIMIT, 0x03E80000) # Default Value.

    def SetCurrentLoopLimits(self, max_motor_current_TM01Units):
        # Update Limits - Before Velocity Loop Testing.
        self.mcchelp.mcc_write(MCC.PID_UQ_UD_LIMITS, 0x3FFF3FFF) # Remove Limit.
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_LIMITS, to_register32(max_motor_current_TM01Units,max_motor_current_TM01Units)) 
        self.mcchelp.mcc_write(MCC.PID_VELOCITY_LIMIT, 0x7FFFFFFF) # Remove Limit.

    def SetCurrentVelocityLoopLimits(self, max_motor_current_TM01Units, max_motor_velocity_TM01Units):
        # Update Limits - Before Velocity Loop Testing.
        self.mcchelp.mcc_write(MCC.PID_UQ_UD_LIMITS, 0x3FFF3FFF) # Remove Limit.
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_LIMITS, to_register32(max_motor_current_TM01Units,max_motor_current_TM01Units)) 
        self.mcchelp.mcc_write(MCC.PID_VELOCITY_LIMIT, int(max_motor_velocity_TM01Units))

    def Estimate_MotorMaxV(self, UserVM, K_est, Guardband=2):
        # Based on Back-EMF limits of UserVM, Max Velocity = UserVM/K_est. The factor of 2 is extra guardband. 
        max_sawtooth_velocity = (UserVM / K_est) / Guardband
        return max_sawtooth_velocity

    def Estimate_MaxVKpKi(self, UserMotorImax, max_sawtooth_velocity, current_loop_bandwidth, dominant_loop_corner_ratio=1.0):
        # Initial guess of Kp_v using supply voltage, motor constant K and maximum current.
        Kp_v = UserMotorImax / max_sawtooth_velocity
        Ki_v = 0.0
        vel_kp_float =  Kp_v 
        vel_ki_float =  Ki_v 
        vel_ki_float_dt = vel_ki_float/self.PWM_FREQUENCY
        vel_fn_vf = current_loop_bandwidth / dominant_loop_corner_ratio
        return vel_kp_float, vel_ki_float, vel_ki_float_dt, vel_fn_vf

    def ConfigureCurrentPIRegisters(self, kp_int, ki_int, current_norm_p, current_norm_i): 
        self.mcchelp.mcc_write(MCC.PID_TORQUE_COEFF, to_register32(kp_int, ki_int)) # Write Torque PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_FLUX_COEFF,   to_register32(kp_int, ki_int)) # Write Flux PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_P, current_norm_p) # Enable/Disable Torque/Flux P Normalization - 1 = Q0.16, 0 = Q8.8.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_I, current_norm_i) # Enable/Disable Torque/Flux I Normalization - 1 = Q0.16, 0 = Q8.8.
        self.mcchelp.mcc_write(MCC.VELOCITY_CONFIG.MOVING_AVRG_FILTER_SAMPLES, 0) # Enable Velocity Moving Average Filter with 8-samples.

    def ConfigureVelocityPIRegisters(self, vel_kp_float, vel_ki_float_dt, vel_fn_vf):
        f_s = self.PWM_FREQUENCY  # configure you targets sampling rate
        down_sampling_factor = 1  # configure you targets down-sampling factor
        f_vf = vel_fn_vf
        if f_vf <= 45.0:
            f_pb = 45.0
            print("Note: Target Velocity Filter Passband = ", f_vf,"Hz. Actual Velocity Filter Passband is set to ",f_pb,"Hz to avoid filter stability issues." )
        else:
            f_pb = f_vf
            print("Velocity Filter Passband is set to ",f_pb,"Hz." )
        filter_specification = LowPassFilter(f_p=f_pb, d_p=1.0)
        # select if you like a bode plot to show up or not
        plot_bode = False
        result = calculate_biquad_filter_coefficients(
            chip_type=Tm01,  # you may also input Tm01 or Tmc4671 here
            f_s=f_s,
            down_sampling_factor=down_sampling_factor,
            filter_spec=filter_specification
        )
        print(result.normalized_coefficients)
        print("A1 = ",result.normalized_coefficients.a_1)
        print("A2 = ",result.normalized_coefficients.a_2)
        print("B0 = ",result.normalized_coefficients.b_0)
        print("B1 = ",result.normalized_coefficients.b_1)
        print("B2 = ",result.normalized_coefficients.b_2)

        kp_velocity_int, ki_velocity_int, velocity_norm_p, velocity_norm_i = self.myMotorTune.ScaleVelocity_PICoeffs(vel_kp_float, vel_ki_float_dt)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Disable Motion Mode while Configuring Coefficients.

        # Prior to any Velocity Testing, set the Velocity Feedback Filters - currently set to the approximate fn of the closed current loop (467Hz).
        self.mcchelp.mcc_write(MCC.BIQUAD_V_ENABLE.BIQUAD_V_ENABLE, 0x00000001) # Enable Velocity BiQuad Filter.
        self.mcchelp.mcc_write(MCC.BIQUAD_T_ENABLE.BIQUAD_T_ENABLE, 0x00000000) # Disable Torque BiQuad Filter.
        self.mcchelp.mcc_write(MCC.VELOCITY_CONFIG.MOVING_AVRG_FILTER_SAMPLES, 0) # Enable Velocity Moving Average Filter with 0-samples - allowd range is 0 to 7 ( meaning 1 to 8 samples.).
        self.mcchelp.mcc_write(MCC.BIQUAD_V_A_1.BIQUAD_V_A_1, result.normalized_coefficients.a_1)
        self.mcchelp.mcc_write(MCC.BIQUAD_V_A_2.BIQUAD_V_A_2, result.normalized_coefficients.a_2)
        self.mcchelp.mcc_write(MCC.BIQUAD_V_B_0.BIQUAD_V_B_0, result.normalized_coefficients.b_0)
        self.mcchelp.mcc_write(MCC.BIQUAD_V_B_1.BIQUAD_V_B_1, result.normalized_coefficients.b_1)
        self.mcchelp.mcc_write(MCC.BIQUAD_V_B_2.BIQUAD_V_B_2, result.normalized_coefficients.b_2)
        self.mcchelp.mcc_write(MCC.PID_CONFIG.VEL_SCALE, self.mySystem.VEL_SCALE)
        self.mcchelp.mcc_write(MCC.PID_CONFIG.VEL_SMPL, self.mySystem.VEL_SAMPL)

        self.mcchelp.mcc_write(MCC.PID_CONFIG.VELOCITY_NORM_P, velocity_norm_p) # Select Velocity Kp Normalization Range.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.VELOCITY_NORM_I, velocity_norm_i) # Select Position Kp Normalization Range.
        self.mcchelp.mcc_write(MCC.PID_VELOCITY_COEFF, to_register32(kp_velocity_int, ki_velocity_int)) # Write Velocity PI Coefficients.


    def Estimate_PositionKp(self, zeta_v=1.65, zeta_i=1.0):
        pos_kp_float = self.myMotorTune.CalculatePosition_PICoeffs(zeta_v, zeta_i)
        return pos_kp_float

    def Estimate_MaxAcceleration(self, J_est, K_est, max_motor_current):
        max_acceleration = (3.0/2.0)*(K_est*max_motor_current)/J_est
        return max_acceleration
        
    def ConfigurePositionPIRegisters(self, pos_kp_float):
        kp_position_int, position_norm_p = self.myMotorTune.ScalePosition_PICoeffs(pos_kp_float)
        self.mcchelp.mcc_write(MCC.PID_CONFIG.POSITION_NORM_P, position_norm_p) # Select Position Kp Normalization Range.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.POSITION_NORM_I, 0)               # Set Position Ki Normalization Range to 0
        self.mcchelp.mcc_write(MCC.PID_POSITION_COEFF, to_register32(kp_position_int, 0)) # Write Position PI Coefficients.
        self.mcchelp.mcc_write(MCC.PID_CONFIG.POS_SMPL, self.mySystem.POS_SAMPL)

    def create_velocity_ramp_RAMDebug_Instance(self, peak_time, CAPTURE_REGS, RAMDEBUG_CAPACITY, max_sawtooth_velocity, system_id_range=10, target_sawtooth_velocity=None, max_acceleration=None, range_turns=None):

        print(f"in function, PWM frequency is {self.PWM_FREQUENCY}")
        # Derive RAMDEBUG and peak_time parameters to maximise use of available memory.
        RAMDEBUG_SAMPLES = int(RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS))) # Per-channel sample count = Total RAMDEBUG Capacity / (4bytes per channel x number of channels ).
        # peak_time = RAMDEBUG_SAMPLES * RAMDEBUG_DIVISOR / self.PWM_FREQUENCY / 4
        RAMDEBUG_DIVISOR = int(peak_time / RAMDEBUG_SAMPLES * self.PWM_FREQUENCY * 4)
        RAMDEBUG_RATE = self.PWM_FREQUENCY/RAMDEBUG_DIVISOR
        RAMDEBUG_PERIOD = 1/RAMDEBUG_RATE
        STIMULUS_DIVISOR = RAMDEBUG_DIVISOR      # Stimulus or Target data is played at the PWM Rate / STIMULUS_DIVISOR
        STIMULUS_RATE = self.PWM_FREQUENCY/STIMULUS_DIVISOR
        STIMULUS_PERIOD = 1/STIMULUS_RATE
        RAMDEBUG_PRETRIGGER_SAMPLES = 1 # Ignore this for now.

        # Derive Sawtooth peak velocity based on system_id_range and peak_time.
        velocity_scaling = self.mySystem.get_velocity_scaling()
        if target_sawtooth_velocity:
            if range_turns: 
                if peak_time * target_sawtooth_velocity > range_turns: 
                    sawtooth_velocity = target_sawtooth_velocity / (peak_time * target_sawtooth_velocity / range_turns)
                else: 
                    sawtooth_velocity = target_sawtooth_velocity
            else: 
                sawtooth_velocity = target_sawtooth_velocity
            
        else:
            distance_turns = system_id_range # 4.431539826510507 # turns
            distance_rads = distance_turns * 2 *math.pi # Rads
            sawtooth_velocity = min(distance_rads/peak_time, max_sawtooth_velocity)  

        ramp_acceleration = sawtooth_velocity/peak_time

        if max_acceleration: # Is specifed as an aurgement.
            if ramp_acceleration > max_acceleration:
                print("# WARNING: Max Acceleration Exceeded.")
                print("# Sawtooth Velocity to ",sawtooth_velocity,"rads/sec.")
                print("# Ramp Acceleration = ",ramp_acceleration,"rads/sec^2.")
                print("# Max Acceleration = ",max_acceleration,"rads/sec^2.")
                sawtooth_velocity = max_acceleration*peak_time
                print("# Setting Sawtooth Velocity to ",sawtooth_velocity,"rads/sec.")
            else:
                print("# Sawtooth Velocity to ",sawtooth_velocity,"rads/sec.")
                print("# Ramp Acceleration = ",ramp_acceleration,"rads/sec^2.")
                print("# Max Acceleration = ",max_acceleration,"rads/sec^2.")
        else:
            print("# Sawtooth Velocity to ",sawtooth_velocity,"rads/sec.")
            print("# Ramp Acceleration = ",ramp_acceleration,"rads/sec^2.")

        print(f"in function, sawtooth_velocity is {sawtooth_velocity}, scaling is {velocity_scaling}")
        sawtooth_velocity_codes = int(sawtooth_velocity/velocity_scaling)

        # Derive Sawtooth trajecory based on sawtooth_velocity_codes and peak_time.
        my_trajectory = TrajectoryProfileSawtoothVelocity(peak_velocity=sawtooth_velocity_codes, peak_time=peak_time)

        # Configure and create a RAMDebug Object.
        my_capture = StimulusCaptureClass(self.myInterface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, STIMULUS_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES)

        return RAMDEBUG_DIVISOR, sawtooth_velocity_codes, my_trajectory, my_capture, RAMDEBUG_PERIOD
