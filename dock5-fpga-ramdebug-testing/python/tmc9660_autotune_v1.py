################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import argparse
import logging
import sys
import time
from types import ModuleType
import serial
import csv
import os
import math
import numpy as np
from numpy.fft import *
from datetime import datetime

from pytrinamic.connections.connection_manager import ConnectionManager
from pytrinamic.connections.serial_tmcl_interface import SerialTmclInterface
from pytrinamic.tmcl import TMCLReplyStatusError
import pytrinamic.RAMDebug as RAMDebug

from motorDataFileFunctionsClass import motorDataFileFunctionsClass
from MotorConfigClass import MotorConfigClass
from TM01SystemUnitsSetup import TM01SystemUnitsSetup
from MccHelpersClass import MccHelpersClass
from StimulusCaptureClass import StimulusCaptureClass
from TorqueFluxSystemIDTuningClass import TorqueFluxSystemIDTuningClass
from DataCaptureHelperClass import DataCaptureHelperClass
from StimulusReaderClass import StimulusReaderClass
from MotorDataFFTClass import MotorDataFFTClass
from TorqueProfileClass import TorqueProfileClass
from UQProfileClass import UQProfileClass
from biquad_filter_calc import calculate_biquad_filter_coefficients, Tm01, Tmc4671, LowPassFilter, AntiResonanceFilter
from register_helpers import to_register32
from TrajectoryProfileSawtoothClass import TrajectoryProfileSawtoothVelocity, TrajectoryProfileSawtoothAcceleration
from FeedForwardParamsClass import FeedForwardParams
from GainPhaseMarginClass import GainPhaseMarginClass

from collections import OrderedDict

import pathlib
MY_SCRIPT_DIR = pathlib.Path(__file__).parent
# This trick is necessary to get the import working from a different folder
# that isn't a subfolder
sys.path.insert(1, str(MY_SCRIPT_DIR / '../../UBL'))
from MemoryMap import TM01

# Grab the address blocks we care about
MCC      = TM01.MCC

### Commandline arguments ######################################################
parser = argparse.ArgumentParser(description="Start a Torque Loop System ID and Auto-Tuning run.")

parser.add_argument("port", help="Serial connection port")
parser.add_argument('--target-motor', default="Stepper",help='Target Motor Type, select between Stepper, BLDC or DC')

parser.add_argument("--plot", default=True, action=argparse.BooleanOptionalAction, help="Plot the captured data")
parser.add_argument('--plot-disappear', default=True, help='Automatically Close any plots after 5 seconds.')
parser.add_argument('--repeated-times', default="1", help="Enable multiple itterations of velocity ramp system ID test.")

parser.add_argument('--end-stop-detect', action="store_true", help="Enable End Stop Detection.")
parser.add_argument("--distance-turns", default="0.0", help="Number of motor turns/rotations used for velocity system ID when end stop detection is disabled. Zero means no motion constraint. ")

# Optional arguments
parser.add_argument("--input", help="Input file of the generated stimulus. If present, we will not capture the Input channel but calculate the data instead, leaving more space for capturing other channels.")

parser.add_argument("--plot-stimulus", action="store_true", help="Show a grpah of the stimulus data generated")
parser.add_argument("--damping-factor", default="0.707", help="Control Loop Damping Factor.")
parser.add_argument("--shunt-resistance", default="0.005", help="H-Bridge Shunt Resistance.")
parser.add_argument("--shunt-op-amp-gain", default="20.0", help="Shunt Amplifier Gain.")
parser.add_argument("--poles", default="50", help="Number of Pole Pairs in Motor.")
parser.add_argument("--maximum-current", default="1.0", help="Maximum Motor Current (A).")
parser.add_argument("--abn-encoder-resolution", default="4096", help="Resolution of ABN Encoder.")
parser.add_argument("--abn-encoder-direction", default="0", help="Direction of ABN Encoder (0,1).")
parser.add_argument("--current-loop-test-channel", default="Flux", help="Current Channel to Test (Flux or Torque).")
parser.add_argument("--tuning-method", default="2", help="Select the Current Loop Kp/Ki Tuning Method.")
parser.add_argument("--current-loop-gm", default="12.0", help="Current Loop Kp/Ki Tuning Method #3  Gain Margin Target (degrees).")
parser.add_argument("--current-loop-pm", default="60.0", help="Current Loop Kp/Ki Tuning Method #3  Phase Margin Target (degrees).")
parser.add_argument("--current-loop-bw", default="1000.0", help="Current Loop Kp/Ki Tuning Method #5  Target Loop Bandwidth (Hz).")

parser.add_argument("--ud-output", default="ud_output.csv", help="UD Output file path")
parser.add_argument("--torque-output", default="torque_output.csv", help="Torque Output file path")
parser.add_argument("--velocity-output", default="velocity_output.csv", help="Velocity Output file path")
parser.add_argument("--offset-output", default="offset_output.csv", help="Path to the results")
parser.add_argument("--systemID-output", default="systemID_summary.csv", help="Path to the systemID summary results")

parser.add_argument('--no-timeout', action="store_true", help="Disable timeout on waiting for bootloader reply, for debugging.")

# Debug flags
parser.add_argument('-v', '--verbose', action="count", default=0, help="Verbosity level")

args = parser.parse_args()

if args.verbose == 0:
    log_level = logging.ERROR
elif args.verbose == 1:
    log_level = logging.WARNING
elif args.verbose == 2:
    log_level = logging.INFO
elif args.verbose >= 3:
    log_level = logging.DEBUG
    # Create local logger
logging.basicConfig(stream=sys.stdout, level=log_level)

# Magic Numbers.
#
# Magic Numbers for testing the UD limit based on the Max Motor Current.
target_motor_current_percentage = 75 # Scaling used to guardband maximum operating current - Percentage (0 to 100%).
max_allowed_ud_codes = 1500 # Max UD allowed when searching for max current UD limit - Codes.
ud_sweep_inc_volts = 0.1 # UD test sweep voltage limit - Volts.
#
# Magic Numbers for controlling the Current Loop Tuning.
select_current_tuning_approach = int(args.tuning_method) # PI Tuning Approach used in Current Tuning Computation.
CurrentLoopGm=float(args.current_loop_gm) # Current Loop Gain Margin Target, used with select_current_tuning_approach=3
CurrentLoopPm=float(args.current_loop_pm) # Current Loop Phase Margin Target, used with select_current_tuning_approach=3
CurrentLoopBW=float(args.current_loop_bw) # Current Loop Bandwidth Target, used with select_current_tuning_approach=5
zeta_i= float(args.damping_factor) # Not the perferred method (select_current_tuning_approach=1) for setting the Current Kp/Ki parameters. Also used in Position Kp Calculations.
#
# Velocity System ID Magic Numbers.
static_friction_position_change_threshold = 0.5 # rads

#
# Position Loop Magic Numbers.
zeta_v = 1.65 # math.sqrt(2) - currently only used as an input for setting the Position Kp coefficient.
#

print("##########################################################################################################################")
print("# Torque Loop - System Identification and Auto Tuning System.                                                            #")
print("##########################################################################################################################")

print("#")
print(" Step 1: Configure System.")
print("#")

print("#### Setting up the System Constants. ####")
### Constants ##################################################################
# PWM Calculations.
RAMDEBUG_CAPACITY = 8192 # 16384
PWM_FREQUENCY = 25000 # Decimal - 25kHz
CORE_CLOCK = 120000000 # Decimal - 120MHz
PWM_COUNT_DEC = int(CORE_CLOCK/PWM_FREQUENCY)-1
PWM_COUNT_HEX = '0x{0:08X}'.format(PWM_COUNT_DEC)
print("PWM_COUNT - DEC ", PWM_COUNT_DEC)
print("PWM_COUNT - HEX ", PWM_COUNT_HEX)

print("#### Done with Setting up the System Constants. ####")

### Connection setup ###########################################################
myInterface = SerialTmclInterface(args.port)

# my_interface.enable_debug(True)
print(myInterface)

### Motor setup ################################################################

# Note: These will be moved to Parameters.
#
SystemPWMFrequency=PWM_FREQUENCY
UserVM=24                                           # Max System Power Supply Voltage. Measured VM = 24.436V
UserRshunt=float(args.shunt_resistance)             # H-Bridge Shunt Resistor Value - Ohms - Estimated Rshunt = 0.0075 Ohms. ( V1.2 Boards = 0.005, V1.1 Board = 0.011)
UserShuntOpAmpGain=float(args.shunt_op_amp_gain)    # V/V ( V1.2 Boards = 20, V1.1 Board = 10)
UserMotorType=args.target_motor                     # Type of Motor Connected.
UserMotorR=0.5                                      # Nominal Motor Winding Resistance.
UserMotorL=0.0006                                   # Nominal Motor Winding Inductance.
UserMotorPoles=int(args.poles)                      # Number of Poles in Motor.
UserMotorVmax=1.4                                   # Maximum Allowed Motor Voltage.
UserMotorImax=float(args.maximum_current)           # Maximum Allowed Motor Current.
UserEnableABN=True                                  # Enable ABN Encoder.
UserABNResolution=int(args.abn_encoder_resolution)  # Resolution of ABN Encoder.
UserABNDirection=int(args.abn_encoder_direction)    # Direction of ABN encoder.

print("############### System / User Inputs. ################")
print("#")
print(f"# Voltage Scaling = {SystemPWMFrequency} Hz.")
print(f"# System Power Voltage = {UserVM} Volts.")
print(f"# H-Bridge Shunt Resistor = {UserRshunt} Ohms.")
print(f"# Shunt OpAmp Gain = {UserShuntOpAmpGain}.")
print("#")
print(f"# Motor Type = {UserMotorType}.")
print(f"# Motor Winding Nominal Resistance = {UserMotorR} Ohms.")
print(f"# Motor Winding Nominal Inductance = {UserMotorL} Henries.")
print(f"# Number of Poles in Motor = {UserMotorPoles} Poles.")
print(f"# Maximum Motor Voltage = {UserMotorVmax} Volts.")
print(f"# Maximum Motor Current = {UserMotorImax} Amps.")
print("#")
print(f"# Enable ABN Encoder     = {UserEnableABN}.")
print(f"# ABN Encoder Resolution = {UserABNResolution} Lines.")
print(f"# ABN Encoder Direction  = {UserABNDirection}.")
print("#")
print("######################################################")

target_motor = args.target_motor
mcchelp = MccHelpersClass(myInterface)
mySystem = TM01SystemUnitsSetup(
    SystemPWMFrequency=SystemPWMFrequency,
    UserVM=UserVM,
    UserRshunt=UserRshunt,
    UserShuntOpAmpGain=UserShuntOpAmpGain,
    UserMotorType=UserMotorType,
    UserMotorR=UserMotorR,
    UserMotorL=UserMotorL,
    UserMotorPoles=UserMotorPoles,
    UserMotorVmax=UserMotorVmax,
    UserMotorImax=UserMotorImax
)

mySystem.SetupVelocityPIParams()
mySystem.SetupPositionPIParams()
myMotor = MotorConfigClass(myInterface, mySystem, UserEnableABN, UserABNResolution, UserABNDirection)
myMotor.config_motor(UserMotorType)

# 
# Restrict App/Axis Mode Velocity Meter changes - Stay in Mode 0 where velocity meter is based on period mode. 
# TODO: Find a better place to put this code.
#
try:    # TODO: Replace try/except with a check to see which mode the part is in and do not send command in register mode.
    myInterface.set_axis_parameter(137, 0, 0x07FFFFFFF) # 137 = Hardcoded address for VelocityMeterSwitchThreshold parameter.
except TMCLReplyStatusError as e:
    print("# WARNING: TM01 is in Register Mode, ignoring Axis Parameter Set.")
#
#
#

myMotorTune = TorqueFluxSystemIDTuningClass(mySystem)
myDataCapture = DataCaptureHelperClass(mySystem=mySystem, myMotor=myMotor, myInterface=myInterface, mcchelp=mcchelp, myMotorTune=myMotorTune)

max_motor_current = UserMotorImax
max_motor_current_TM01Units = int(max_motor_current/mySystem.get_flux_scaling())
target_voltageext_codes, current_limit_codes = myMotor.set_allowed_voltageext_level(max_motor_current, target_motor_current_percentage, ud_inc_volts=ud_sweep_inc_volts , max_ud_codes=max_allowed_ud_codes)

print("Target UD Codes = ", target_voltageext_codes)
print("Maximum Test Current Codes = ", current_limit_codes)
print("Maximum Motor Current Codes =", max_motor_current_TM01Units)

myDataCapture.SetCurrentVoltageLimits(target_voltageext_codes=target_voltageext_codes, max_motor_current_TM01Units=max_motor_current_TM01Units)

print("#")
print(" Step 2: Capture UD/FLUX data for Motor System ID.")
print("#")


# Set at least one pretrigger to not miss the first value of the stimulus
RAMDEBUG_PRETRIGGER_SAMPLES = 1 # STIMULUS_DIVISOR # Per-channel sample count

# Set at least the length of the stimulus + 1 (for the above pretrigger sample)
# to see the full stimulus data
#
# NOTE: 
# RAMDebug available memory is expanded to 32,768 Bytes in total using the NOBL_ROM Workflow ( from the default 2048 Bytes).
# Each sample is 4-bytes so there is sufficient space for 8192 Samples. 
#

# Define the Capture Regs.

CAPTURE_REGS = [
            "MCC.VOLTAGE_EXT",           # <- This should be the stimulated register
            "MCC.PID_TORQUE_FLUX_ACTUAL",
            "MCC.PID_POSITION_ACTUAL"
        ]
RAMDEBUG_SAMPLES = int(RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS))) # Per-channel sample count = Total RAMDEBUG Capacity / (4bytes per channel x number of channels ).
print("########################################### RAMDEBUG_SAMPLES = ", RAMDEBUG_SAMPLES)
# Sampling Frequency divisors
measure_time = 0.0546 # 16384/(4*len(CAPTURE_REGS))/PWM_FREQUENCY
print("########################################### measure_time = ", measure_time)
RAMDEBUG_DIVISOR = max(int(measure_time * PWM_FREQUENCY / RAMDEBUG_SAMPLES), 1)      # Measurement or Actual data is played at the PWM Rate / RAMDEBUG_DIVISOR
print("########################################### RAMDEBUG_DIVISOR = ", RAMDEBUG_DIVISOR)
RAMDEBUG_RATE = PWM_FREQUENCY/RAMDEBUG_DIVISOR
RAMDEBUG_PERIOD = 1/RAMDEBUG_RATE
STIMULUS_DIVISOR = max(int(measure_time * PWM_FREQUENCY / RAMDEBUG_SAMPLES), 1)      # Measurement or Actual data is played at the PWM Rate / RAMDEBUG_DIVISOR
STIMULUS_RATE = PWM_FREQUENCY/STIMULUS_DIVISOR
STIMULUS_PERIOD = 1/STIMULUS_RATE

print("################ STARTING UD CAPTURE ##################")
mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000028) # VOLTAGE_EXT Mode
myUDStimCapture = StimulusCaptureClass(myInterface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, STIMULUS_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES)

ud_step_max = target_voltageext_codes
ud_step_start = 0 # int(target_voltageext_codes/2)
mcchelp.mcc_write(MCC.VOLTAGE_EXT, ud_step_start)
time.sleep(1)
raw_data = myUDStimCapture.RAMDebug_Setup_Capture_WithStep("MCC.VOLTAGE_EXT.UD", magnitude=ud_step_max, step_start=ud_step_start)

# Zero the UD/UQ voltages.
mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000028) # VOLTAGE_EXT Mode
myMotor.zero_voltageext()
myMotor.zero_torquefluxtarget()
print("################ COMPLETED UD CAPTURE ##################")

data = myUDStimCapture.GrabExportData(raw_data, args.plot, args.plot_disappear, args.input, args.ud_output)

print("#")
print("# Done with Capture of UD/FLUX data for Motor System ID.")
print("#")


print("#")
print(" Step 3: Estimate Motor Rest and Lest and Estimate Torque/Flux Loop PI values.")
print("#")


Rest, Lest, Tdelay = myMotorTune.estimate_R_L(data, "MCC.VOLTAGE_EXT.UD",  "MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL", period=RAMDEBUG_PERIOD)
motor_f3dB =  myMotorTune.freq3dB

# def estimate_R_L_delay(self, data, period=None, max_delay=4): 
print(" Returned Estimated Motor R = ", Rest)
print(" Returned Estimated Motor L = ", Lest)
print(" Returned Estimated Loop Delay = ", Tdelay)
print(" Estimated Motor Transfer Function 3dB Corner Frequency = ", motor_f3dB,"Hz.")

kp_float, ki_float, ki_float_dt = myMotorTune.CalculateTorqueFlux_PICoeffs(Rest, Lest, zeta_i, Tdelay=Tdelay, use_approach=select_current_tuning_approach, Gm=CurrentLoopGm, Pm=CurrentLoopPm, loop_bw=CurrentLoopBW)
kp_int, ki_int, current_norm_p, current_norm_i = myMotorTune.ScaleTorqueFlux_PICoeffs(kp_float, ki_float_dt)

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

myMotor.zero_voltageext()
myMotor.zero_torquefluxtarget()

print("#")
print(" Done with Estimation of Motor Rest and Lest and Estimate Torque/Flux Loop PI values.")
print("#")

time.sleep(5)

print("#")
print(" Step 4: Set Current Loop Tuning Parameters and run Coarse Static Friction Torque Test.")
print("#")

# Update Limits - Before Velocity Loop System ID and  Loop Tuning.
myDataCapture.SetCurrentLoopLimits(max_motor_current_TM01Units=max_motor_current_TM01Units)

# ABN encoder configuration (Init encoder (mode 0))
myMotor.init_abn_encoder(voltage_codes=target_voltageext_codes)

myDataCapture.zero_stop_motor()
time.sleep(1)

mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
myDataCapture.ConfigureCurrentPIRegisters(kp_int=kp_int, ki_int=ki_int, current_norm_p=current_norm_p, current_norm_i=current_norm_i)
# Update Limits - Before Velocity Loop System ID and  Loop Tuning.
myDataCapture.SetCurrentLoopLimits(max_motor_current_TM01Units=max_motor_current_TM01Units)

# Load Torque Profile Class and get set position change threshold scaling in TM01 Units.
myTorqueProfile = TorqueProfileClass(myInterface, mySystem, kp_int, ki_int, current_norm_p, current_norm_i, "../capture_data/torque_profile.csv", RAMDEBUG_CAPACITY=RAMDEBUG_CAPACITY)
static_friction_position_threshold_codes = int(static_friction_position_change_threshold/mySystem.get_position_scaling())
print("#")
print("Static friction threshold = ", static_friction_position_change_threshold, " rads - ", static_friction_position_threshold_codes, " codes.")
print("#")

# Run static friction test. The detect_critical_current_kest() method combines the static friction torque threshold and Motor K estimation.
static_friction_current_codes, kest_data, kest_period = myTorqueProfile.detect_critical_current_kest(current_limit_codes, Rest, critical_threshold=static_friction_position_threshold_codes)
K_est_init =  myMotorTune.estimate_K(kest_data, Rest, period=kest_period)

print("# Motor Constant, K dynamical Estimate (initial measurement) = ", K_est_init)
K_est = K_est_init

if args.end_stop_detect: 
    print("#")
    print("# Running Endstop Detection Test.")
    print("#")
    max_left_num_of_turn, max_right_num_of_turn, LeftEndPosition, RightEndPosition = myTorqueProfile.detect_end_stop_final(static_friction_current_codes)
    
    print("# Left  Turn Rotations    (TM01 Units) = ", max_left_num_of_turn)
    print("# Right  Turn Rotations   (TM01 Units) = ", max_right_num_of_turn)
    print("# Left End Stop Position  (TM01 Units) = ", LeftEndPosition)
    print("# Right End Stop Position (TM01 Units) = ", RightEndPosition)
    print("#")
    print("# Selected Position Scaling Factor (TM01 Chip Units per rads )= ", mySystem.POSITION_SCALING_FACTOR)
    print("# Selected Position Scaling Factor (rads per TM01 Chip Unit) = ", mySystem.PositionScaling)
    print("#")
    max_left_num_of_turn_rotations  = max_left_num_of_turn
    max_right_num_of_turn_rotations = max_right_num_of_turn
    LeftEndPosition_rads = LeftEndPosition * mySystem.PositionScaling
    RightEndPosition_rads = RightEndPosition * mySystem.PositionScaling
    print("# Left  Turn Rotations (Rotations) = ", max_left_num_of_turn_rotations)
    print("# Right  Turn Rotations (Rotations) = ", max_right_num_of_turn_rotations)
    print("# Left End Stop Position (Rads )  = ", LeftEndPosition_rads)
    print("# Right End Stop Position (Rads )  = ", RightEndPosition_rads)
    print("#")
    
    range_turns = (max_left_num_of_turn - max_right_num_of_turn)
    system_id_start_point = -0.3*range_turns
    system_id_range = 0.6*range_turns
    
    print("# Total Range (Turns) = ", range_turns)
    print("# System ID Start Point (Turns) = ", system_id_start_point)
    print("# System ID Range (Turns) = ", system_id_range)
    print("#")

    done = myTorqueProfile.move_to_final(system_id_start_point, max_left_num_of_turn, max_right_num_of_turn, LeftEndPosition, RightEndPosition, static_friction_current_codes)

else:
    print("#")
    print("# Bypassing Endstop Detection Test.")
    print("#")
    range_turns = float(args.distance_turns)
    system_id_start_point = -0.4*range_turns
    system_id_range = 0.8*range_turns

    print("# Total Range (Turns) = ", range_turns)
    print("# System ID Start Point (Turns) = ", system_id_start_point)
    print("# System ID Range (Turns) = ", system_id_range)
    print("#")

print("################ COMPLETED Static Friction Test ##################")

print("#")
print(" Step 5: Estimate Motor Constant and approximate Velocity Kp value for Velocity System ID Ramp Test.")
print("#")

if mySystem.MotorType == "Stepper":
    MaxVGuardband = 2
elif mySystem.MotorType == "BLDC":
    MaxVGuardband = 8
else:
    MaxVGuardband = 2

repeated_times = int(float(args.repeated_times))
print(f"#################Start repeatability test, repeated times = {repeated_times}################")
# save system ID result from the repeatability test in data_summary
headers = ["Run", "Status", "Motor Type", "Max Current" , "Poles", "Inertial Load" "Encoder Resolution", "Encoder Direction", "Constraints Enabled", "Constraints Defaults", "R", "L", "Zeta_i", "Motor Bandwidth (Hz)", "Kp_i", "Ki_i", "Ki_i_dt", "Gain crossover frequency(rad/s)", "Phase crossover freqency(rad/s)", "Gain Margin", "Phase Margin", "Current Loop Bandwidth (Hz)", "Critial Current", "RAMDebug samples", "RAMDebug divisor", "K_est", "J_est", "b_est", "fs_est", "Max Velocity (rads/sec)", "Mech_BW (rads/sec)", "Kp_v", "Ki_v", "Ki_v_dt", "f_vf (Hz)", "Kp_p"]
data_summary = []

for i_run in range(repeated_times): 

    status = "Pass"
    print(f" ### Run {i_run} ###")

    if args.end_stop_detect: 
        done = myTorqueProfile.move_to_final(system_id_start_point, max_left_num_of_turn, max_right_num_of_turn, LeftEndPosition, RightEndPosition, static_friction_current_codes)
        print("Move the load to the initial point. ")

    # Zero all inputs and outputs.
    myDataCapture.zero_stop_motor_not_position()
    # Initial guess of Kp_v using supply voltage, motor constant K and maximum current.
    # Based on Back-EMF limits of UserVM, Max Velocity = UserVM/K_est. The factor of 2 is extra guardband. 
    max_sawtooth_velocity =  myDataCapture.Estimate_MotorMaxV(UserVM, K_est, Guardband=MaxVGuardband)
    vel_kp_float, vel_ki_float, vel_ki_float_dt, vel_fn_vf = myDataCapture.Estimate_MaxVKpKi(UserMotorImax, max_sawtooth_velocity, motor_f3dB)
    target_sawtooth_velocity = max_sawtooth_velocity

    print("#")
    print("################ Velocity Loop System ID - Initial Coarse Loop Parameters ##################")
    print("# Estimated Initial Motor Constant (K) = ", K_est)
    print("# Estimated Initial Max Velocity = ", max_sawtooth_velocity )
    print("# Initial Velocity Kp Estimate = ", vel_kp_float)
    print("# Initial Velocity Ki Estimate = ", vel_ki_float)
    print("# Initial Velocity Ki.dt Estimate = ", vel_ki_float_dt)
    print("# Initial Velocity Filter Corner(Hz) = ", vel_fn_vf)
    print("#")

    myDataCapture.ConfigureVelocityPIRegisters(vel_kp_float, vel_ki_float_dt, vel_fn_vf)

    # Update Limits - Before Velocity Loop Testing.
    myDataCapture.SetCurrentLoopLimits(max_motor_current_TM01Units)

    print("#")
    print(" Step 6: Perform inital Velocity Ramp Test to improve K estimate.")
    print("#")

    myDataCapture.zero_stop_motor_not_position()# Switch to Stopped.

    # No offset during the first run. 
    static_friction_codes = 0
    viscous_friction_codes = 0
    inertia_codes = 0
    my_feedforwardparams = FeedForwardParams(static_friction_codes, viscous_friction_codes, inertia_codes)

    # Create Velocity Ramp and RAMDebug Capture Object.
    initial_divisor = 32
    peak_time = initial_divisor * 4e-5 * 1024 / 4
    # RAMDEBUG_DIVISOR = initial_divisor

    CAPTURE_REGS = [
        "MCC.FOC_UQ_UD", 
        "MCC.PIDIN_TORQUE_FLUX_TARGET",
        "MCC.PIDIN_VELOCITY_TARGET",
        "MCC.PID_VELOCITY_ACTUAL"
        ]
    RAMDEBUG_SAMPLES = int(RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS))) # Per-channel sample count = Total RAMDEBUG Capacity / (4bytes per channel x number of channels ).
        
    if args.end_stop_detect: 
        RAMDEBUG_DIVISOR, sawtooth_velocity_codes, my_trajectory, my_kstimulus_capture, RAMDEBUG_PERIOD = myDataCapture.create_velocity_ramp_RAMDebug_Instance(peak_time, CAPTURE_REGS, RAMDEBUG_CAPACITY, max_sawtooth_velocity, system_id_range )
    else:
        RAMDEBUG_DIVISOR, sawtooth_velocity_codes, my_trajectory, my_kstimulus_capture, RAMDEBUG_PERIOD = myDataCapture.create_velocity_ramp_RAMDebug_Instance(peak_time, CAPTURE_REGS, RAMDEBUG_CAPACITY, max_sawtooth_velocity, system_id_range, target_sawtooth_velocity=target_sawtooth_velocity, range_turns=range_turns )

    print("#")
    print(" Capture the data for K-Estimate")
    print("#")

    print("#")
    print("# Velocity System ID - 1st Ramp RAMDEBUG_DIVISOR = ", RAMDEBUG_DIVISOR)
    print("#")

    # Register Dump.
    # register_file = "./register_dump_broken2.txt"
    # mcchelp.read_all_Registers(register_file)

    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000002) # Switch to Velocity Mode. 01 for torque mode, 03 for position mode
    raw_data = my_kstimulus_capture.RAMDebug_Capture_Velocity_Profile(my_trajectory, my_feedforwardparams, use_firmware_upgrade_method=False)
    myDataCapture.zero_stop_motor_not_position()# Switch to Stopped.
    print("#")
    print(" Data captured for K-Estimate. Stopping the motor")
    print("#")

    data = my_kstimulus_capture.GrabExportData(raw_data, args.plot, args.plot_disappear, output=args.velocity_output)
    K_est_updated =  myMotorTune.estimate_K(data, Rest, period=RAMDEBUG_PERIOD)
    print("# Motor Constant, K Coarse Measurement Estimate = ", K_est_init)
    print("# Updated Motor Constant, K dynamical Measurement Estimate = ", K_est_updated)
    K_est = K_est_updated

    # Update max_velocity_estimate based on updated K_est
    max_sawtooth_velocity =  myDataCapture.Estimate_MotorMaxV(UserVM, K_est, Guardband=MaxVGuardband)
    vel_kp_float, vel_ki_float, vel_ki_float_dt, vel_fn_vf = myDataCapture.Estimate_MaxVKpKi(UserMotorImax, max_sawtooth_velocity, motor_f3dB)
    target_sawtooth_velocity = max_sawtooth_velocity

    print("#")
    print("################ Velocity Loop System ID - Initial Fine-Tuned K_est Loop Parameters ##################")
    print("# Estimated Initial Motor Constant (K) = ", K_est)
    print("# Estimated Initial Max Velocity = ", max_sawtooth_velocity )
    print("# Initial Velocity Kp Estimate = ", vel_kp_float)
    print("# Initial Velocity Ki Estimate = ", vel_ki_float)
    print("# Initial Velocity Ki.dt Estimate = ", vel_ki_float_dt)
    print("# Initial Velocity Filter Corner(Hz) = ", vel_fn_vf)
    print("#")

    myDataCapture.ConfigureVelocityPIRegisters(vel_kp_float, vel_ki_float_dt, vel_fn_vf)

    omega, iq = myMotorTune.extract_data_for_sawtooth(data, "MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL", "MCC.PIDIN_TORQUE_FLUX_TARGET.PIDIN_TORQUE_TARGET", period=RAMDEBUG_PERIOD)
    omega_target, _ = myMotorTune.extract_data_for_sawtooth(data, "MCC.PIDIN_VELOCITY_TARGET.PIDIN_VELOCITY_TARGET", "MCC.PIDIN_TORQUE_FLUX_TARGET.PIDIN_TORQUE_TARGET", period=RAMDEBUG_PERIOD)

    divisor_updated_factor = myMotorTune.find_divisor_mod_factor(omega, omega_target, RAMDEBUG_DIVISOR/PWM_FREQUENCY, PWM_FREQUENCY, omega_eval_ratio=0.1, omega_eval_window_size=10, plot_enable=False, plot_disappear=args.plot_disappear)

    print("#")
    print(" Capture the data for velocity loop system ID")
    print("#")

    CAPTURE_REGS = [
        "MCC.PIDIN_TORQUE_FLUX_TARGET",           # <- This should be the stimulated register
        "MCC.PID_TORQUE_FLUX_ACTUAL",
        "MCC.PIDIN_VELOCITY_TARGET",
        "MCC.PID_VELOCITY_ACTUAL"
        ]
    RAMDEBUG_SAMPLES = int(RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS))) # Per-channel sample count = Total RAMDEBUG Capacity / (4bytes per channel x number of channels ).
    # RAMDEBUG_DIVISOR = int(divisor_updated_factor * initial_divisor)
    peak_time *= divisor_updated_factor

    print("#")
    print("# Velocity System ID Acceleration reduction scaling = ", divisor_updated_factor)
    print("# Velocity System ID - acceleration_based RAMDEBUG_DIVISOR = ", int(divisor_updated_factor * initial_divisor))
    print("#")

    my_stimulus_capture = StimulusCaptureClass(myInterface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, RAMDEBUG_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES)

    # Update RAMDebug DIVISOR based on the ratio between target and actual velocity. 
    myDataCapture.ConfigureVelocityPIRegisters(vel_kp_float, vel_ki_float_dt, vel_fn_vf)
    if args.end_stop_detect: 
        RAMDEBUG_DIVISOR, sawtooth_velocity_codes, my_trajectory, my_stimulus_capture, RAMDEBUG_PERIOD = myDataCapture.create_velocity_ramp_RAMDebug_Instance(peak_time, CAPTURE_REGS, RAMDEBUG_CAPACITY, max_sawtooth_velocity, system_id_range )
    else:
        RAMDEBUG_DIVISOR, sawtooth_velocity_codes, my_trajectory, my_stimulus_capture, RAMDEBUG_PERIOD = myDataCapture.create_velocity_ramp_RAMDebug_Instance(peak_time, CAPTURE_REGS, RAMDEBUG_CAPACITY, max_sawtooth_velocity, system_id_range, target_sawtooth_velocity=target_sawtooth_velocity, range_turns=range_turns )

    print("#")
    print("# Velocity System ID - 2nd Ramp RAMDEBUG_DIVISOR = ", RAMDEBUG_DIVISOR)
    print("#")

    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000002) # Switch to Velocity Mode. 01 for torque mode, 03 for position mode
    raw_data = my_stimulus_capture.RAMDebug_Capture_Velocity_Profile(my_trajectory, my_feedforwardparams, use_firmware_upgrade_method=False)
    myDataCapture.zero_stop_motor_not_position()# Switch to Stopped.

    data = my_stimulus_capture.GrabExportData(raw_data, plot_enable=args.plot, plot_disappear=args.plot_disappear, output=args.velocity_output)

    # Update velocity and torque data. 
    omega, iq = myMotorTune.extract_data_for_sawtooth(data, "MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL", "MCC.PIDIN_TORQUE_FLUX_TARGET.PIDIN_TORQUE_TARGET", period=RAMDEBUG_PERIOD)
    omega_FFT_filtered, iq_FFT_filtered = myMotorTune.filter_FFT(omega, iq, timestep=RAMDEBUG_PERIOD, zero_noisy_freq_comp=1, fraction_target_x=0.05, fraction_target_y=0.05, zero_high_freq_comp=1, expansion_factor=0.15)
    
    J_est_filtered, b_est_filtered, fs_est_filtered = myMotorTune.estimate_J_b_fs_sawtooth(omega_FFT_filtered, iq_FFT_filtered, K_est, delta_t=RAMDEBUG_PERIOD, bin_length = int(len(omega_FFT_filtered)/8), butterworth_corner_freq_ratio=0.05, plot_enable=False, plot_disappear=args.plot_disappear)

    if J_est_filtered < 0.0:
        print("# WARNING: J_est_filtered < 0, using absolute value.")
        J_est  = abs(J_est_filtered)
        status = "Fail"
    else:
        J_est  = J_est_filtered

    if b_est_filtered < 0.0:
        print("# WARNING: b_est_filtered < 0, using absolute value.")
        b_est  = abs(b_est_filtered)
        status = "Fail"
    else:
        b_est  = b_est_filtered

    if fs_est_filtered < 0.0:
        print("# WARNING: fs_est_filtered < 0, using absolute value.")
        fs_est  = abs(fs_est_filtered)
        status = "Fail"
    else:
        fs_est  = fs_est_filtered

    max_acceleration = myDataCapture.Estimate_MaxAcceleration(J_est, K_est, max_motor_current)
    omega_max = myMotorTune.GetMaxVelocity(Rest, Lest, K_est, UserMotorPoles, b_est, UserVM)
    omega_max_rpm = (omega_max/(2*math.pi))*60.0
    omega_max_TM01Units = int(omega_max/mySystem.VelocityScaling)

    myDataCapture.zero_stop_motor_not_position()# Switch to Stopped.
    print("#")
    print("################ Velocity Loop System ID Parameters ##################")
    print("# RAMDEBUG DIVISOR = ", RAMDEBUG_DIVISOR)
    print("# Estimated Motor Constant (K) = ", K_est)
    print("# Estimated Inertia (J) = ", J_est)
    print("# Estimated Friction (b) = ", b_est)
    print("# Estimated Static Friction (fs) = ", fs_est)
    print("# Estimated Max Velocity (omega_max) = ", omega_max, " rads/sec (", omega_max_rpm," RPM ,", omega_max_TM01Units," TM01 Chip Units )." )
    print("# Estimated Max Acceleration = ", max_acceleration, " rads/sec^2")

    print("# Zeta_I = ", myMotorTune.zeta_est)
    print("#")

    #
    #
    #
    #   End of Velocity System ID.
    #
    #
    #
    #

    print("#")
    print(" Step 7: Compute Velocity Loop PI and Filter Parameters.")
    print("#")


    print ("#")
    print("Root-Locus Velocity Loop Kp/Ki Tuning.")
    print("#")
    vel_fn_vf, vel_kp_float, vel_ki_float, vel_ki_float_dt = myMotorTune.CalculateVelocity_PICoeffs_RLocus(Rest, Lest, Tdelay, K_est, J_est, b_est, kp_float, ki_float)

    pos_kp_float = myDataCapture.Estimate_PositionKp(zeta_v, zeta_i)

    data_row = [i_run, status, args.target_motor, UserMotorImax, UserMotorPoles, "", UserABNResolution, UserABNDirection, system_id_range, Rest, Lest, zeta_i, motor_f3dB, kp_float, ki_float, ki_float_dt, w_gc, w_pc, gainMargin, phaseMargin, myMotorTune.freq_natural, static_friction_current_codes * mySystem.get_flux_scaling(), RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, K_est, J_est_filtered, b_est_filtered, fs_est_filtered, max_sawtooth_velocity, myMotorTune.wn_mech, vel_kp_float, vel_ki_float, vel_ki_float_dt, vel_fn_vf, pos_kp_float]
    data_summary.append(data_row)

    print ("#")
    print("Update Velcoity PI, Filter Values.")
    print("#")
    myDataCapture.ConfigureVelocityPIRegisters(vel_kp_float, vel_ki_float_dt, vel_fn_vf)

    print("#")
    print("################ Position Loop Calculated PI Parameters ##################")
    print("#")
    print("# Position Kp Estimate = ", pos_kp_float)

    myDataCapture.ConfigurePositionPIRegisters(pos_kp_float)

    print("#")
    print(" Done with Estimation of Velocity System ID and Computation of Velocity/Position Loop PI values.")
    print("#")

    print("#")
    print(" Step 8: Run a Closed Loop Velocity Step Test (without Feedforward Torque Offset Compensation).")
    print("#")
    myDataCapture.zero_stop_motor()# Switch to Stopped.

    # ABN encoder configuration (Init encoder (mode 0))
    myMotor.init_abn_encoder(voltage_codes=target_voltageext_codes)

    myDataCapture.zero_stop_motor()# Switch to Stopped.
    time.sleep(1)

    # Update Limits - Before Velocity Loop Testing - ignore velocity loop limits for now..
    myDataCapture.SetCurrentLoopLimits(max_motor_current_TM01Units)

    # Set the Feedforward paramaters to zero.

    static_friction_codes = 0
    viscous_friction_codes = 0
    inertia_codes = 0
    my_feedforwardparams = FeedForwardParams(static_friction_codes, viscous_friction_codes, inertia_codes)

    # Setup Velocity RAMDebug Capture Class & Trajectory.
    CAPTURE_REGS = [
        "MCC.PIDIN_TORQUE_FLUX_TARGET",           # <- This should be the stimulated register
        "MCC.PID_TORQUE_FLUX_ACTUAL",
        "MCC.PIDIN_VELOCITY_TARGET",
        "MCC.PID_VELOCITY_ACTUAL"
        ]

    # Create Velocity Ramp and RAMDebug Capture Object.
    print(f"max acceleration is {max_acceleration}, RAMDebug divisor is {RAMDEBUG_DIVISOR}, capacity is {RAMDEBUG_CAPACITY}, range is {system_id_range}")
    if args.end_stop_detect: 
        RAMDEBUG_DIVISOR, sawtooth_velocity_codes, my_trajectory, myVelocityStimCapture, RAMDEBUG_PERIOD = myDataCapture.create_velocity_ramp_RAMDebug_Instance(peak_time, CAPTURE_REGS, RAMDEBUG_CAPACITY, max_sawtooth_velocity, system_id_range, max_acceleration=max_acceleration )
    else:
        RAMDEBUG_DIVISOR, sawtooth_velocity_codes, my_trajectory, myVelocityStimCapture, RAMDEBUG_PERIOD = myDataCapture.create_velocity_ramp_RAMDebug_Instance(peak_time, CAPTURE_REGS, RAMDEBUG_CAPACITY, max_sawtooth_velocity, system_id_range, target_sawtooth_velocity=target_sawtooth_velocity, range_turns=range_turns, max_acceleration=max_acceleration )

    # peak_time, sawtooth_velocity_codes, my_trajectory, myVelocityStimCapture, RAMDEBUG_PERIOD = myDataCapture.create_velocity_ramp_RAMDebug_Instance(RAMDEBUG_DIVISOR, CAPTURE_REGS, RAMDEBUG_CAPACITY, max_sawtooth_velocity, system_id_range=system_id_range, max_acceleration=max_acceleration )
    print(f"configured max velociy is {sawtooth_velocity_codes}")


    print("#")
    print(" Capture the data for non-compensated Closed Loop Velocity.")
    print("#")

    time.sleep(1)
    print("################ STARTING VELOCITY CAPTURE ##################")
    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000002) # Switch to Velocity Mode. 01 for torque mode, 03 for position mode
    raw_data = myVelocityStimCapture.RAMDebug_Capture_Velocity_Profile(my_trajectory, my_feedforwardparams, use_firmware_upgrade_method=False)
    myDataCapture.zero_stop_motor()# Switch to Stopped.
    time.sleep(1)

    data = myVelocityStimCapture.GrabExportData(raw_data, args.plot, args.plot_disappear, args.input, args.velocity_output)

    print("################ COMPLETED VELOCITY CAPTURE ##################")


output_summary_fname = os.path.splitext(args.systemID_output)[0] + '_' + args.target_motor + '.csv'
myFileIO = motorDataFileFunctionsClass(output_summary_fname)
myFileIO.write_summary(headers, data_summary)


print("#")
print(" Step 9: Run a Closed Loop Velocity Ramp Test (with Feedforward Torque Offset Compensation).")
print("#")

myDataCapture.zero_stop_motor()# Switch to Stopped.

# Configure Feedforward Parameters.
velocity_scaling = mySystem.get_velocity_scaling()
current_scaling = mySystem.get_flux_scaling()

static_friction_codes = fs_est / (3/2*K_est) / current_scaling
viscous_friction_codes = b_est / (3/2*K_est) / current_scaling * velocity_scaling
inertia_codes = J_est / (3/2*K_est) / current_scaling * velocity_scaling
my_feedforwardparams = FeedForwardParams(static_friction_codes, viscous_friction_codes, inertia_codes)

print("#")
print("# Static Friction Compensation Value = ", static_friction_codes)
print("# Dynamic Friction Compensation Value = ", viscous_friction_codes)
print("# Inertia Compensation Value = ", inertia_codes)
print("#")

# Setup Velocity RAMDebug Capture Class & Trajectory.
# RAMDEBUG_DIVISOR = 128
CAPTURE_REGS = [
    "MCC.PIDIN_TORQUE_FLUX_TARGET",           # <- This should be the stimulated register
    "MCC.PID_TORQUE_FLUX_ACTUAL",
    "MCC.PIDIN_VELOCITY_TARGET",
    "MCC.PID_VELOCITY_ACTUAL"
    ]
# Create Velocity Ramp and RAMDebug Capture Object.
if args.end_stop_detect: 
    RAMDEBUG_DIVISOR, sawtooth_velocity_codes, my_trajectory, myVelocityStimCapture, RAMDEBUG_PERIOD = myDataCapture.create_velocity_ramp_RAMDebug_Instance(peak_time, CAPTURE_REGS, RAMDEBUG_CAPACITY, max_sawtooth_velocity, system_id_range, max_acceleration=max_acceleration )
else:
    RAMDEBUG_DIVISOR, sawtooth_velocity_codes, my_trajectory, myVelocityStimCapture, RAMDEBUG_PERIOD = myDataCapture.create_velocity_ramp_RAMDebug_Instance(peak_time, CAPTURE_REGS, RAMDEBUG_CAPACITY, max_sawtooth_velocity, system_id_range, target_sawtooth_velocity=target_sawtooth_velocity, range_turns=range_turns, max_acceleration=max_acceleration )

# peak_time, sawtooth_velocity_codes, my_trajectory, myVelocityStimCapture, RAMDEBUG_PERIOD = myDataCapture.create_velocity_ramp_RAMDebug_Instance(RAMDEBUG_DIVISOR, CAPTURE_REGS, RAMDEBUG_CAPACITY, max_sawtooth_velocity, system_id_range=system_id_range, max_acceleration=max_acceleration)

print("#")
print(" Capture the data for Compensated Closed Loop Velocity.")
print("#")

time.sleep(1)
print("################ STARTING VELOCITY CAPTURE ##################")
mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000002) # Switch to Velocity Mode. 01 for torque mode, 03 for position mode
raw_data = myVelocityStimCapture.RAMDebug_Capture_Velocity_Profile(my_trajectory, my_feedforwardparams, use_firmware_upgrade_method=False)
myDataCapture.zero_stop_motor()# Switch to Stopped.
time.sleep(1)

data = myVelocityStimCapture.GrabExportData(raw_data, args.plot, args.plot_disappear, args.input, args.velocity_output)

print("################ COMPLETED VELOCITY CAPTURE ##################")

# Close the connection.
myInterface.close()
print("#")
print(" Done with Velocity Closed Loop Test.")
print("#")
print("Done.")
