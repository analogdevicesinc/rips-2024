################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import argparse
import logging
import sys
import time
import serial
import csv
import os
import math
import numpy as np
from datetime import datetime

import matplotlib.pyplot as plot

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
from StimulusReaderClass import StimulusReaderClass
from MotorDataFFTClass import MotorDataFFTClass
from TorqueProfileClass import TorqueProfileClass
from UQProfileClass import UQProfileClass

from register_helpers import to_register32

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

# Optional arguments
parser.add_argument("--input", help="Input file of the generated stimulus. If present, we will not capture the Input channel but calculate the data instead, leaving more space for capturing other channels.")
parser.add_argument("--sine-csv-file", default="sine_flux_input.csv", help="Multi-Tone Sine Flux Input file path")

parser.add_argument("--plot-stimulus", action="store_true", help="Show a grpah of the stimulus data generated")
parser.add_argument("--damping-factor", default="1.0", help="Control Loop Damping Factor.")
parser.add_argument("--shunt-resistance", default="0.005", help="H-Bridge Shunt Resistance.")
parser.add_argument("--shunt-op-amp-gain", default="20.0", help="Shunt Amplifier Gain.")
parser.add_argument("--poles", default="50", help="Number of Pole Pairs in Motor.")
parser.add_argument("--maximum-current", default="1.0", help="Maximum Motor Current (A).")
parser.add_argument("--abn-encoder-resolution", default="4096", help="Resolution of ABN Encoder.")
parser.add_argument("--abn-encoder-direction", default="0", help="Direction of ABN Encoder (0,1).")
parser.add_argument("--current-loop-test-channel", default="Flux", help="Current Channel to Test (Flux or Torque).")
parser.add_argument("--tuning-method", default="2", help="Select the Current Loop Kp/Ki Tuning Method.")
parser.add_argument("--gain-margin", default="12.0", help="Current Loop Kp/Ki Tuning Method #3 - Gain Margin Target.")
parser.add_argument("--phase-margin", default="60.0", help="Current Loop Kp/Ki Tuning Method #3 - Phase Margin Target.")

parser.add_argument("--ud-output", default="ud_output.csv", help="UD Output file path")
parser.add_argument("--torque-output", default="torque_output.csv", help="Torque Output file path")
parser.add_argument("--prbs-ud-output", default="prbs_ud_output.csv", help="PRBS UD Output file path")
parser.add_argument("--prbs-flux-output", default="prbs_flux_output.csv", help="PRBS Flux Output file path")
parser.add_argument("--sine-flux-output", default="sine_flux_output.csv", help="Sine Flux Output file path")

parser.add_argument("--enable-prbs-test", action="store_true", help="Enable PRBS Testing")
parser.add_argument("--enable-flux-loop-test", action="store_true", help="Enable Flux Loop Testing from maximum-current/10 to maximum-current")
parser.add_argument("--enable-sine-test", action="store_true", help="Enable Multi-Tone Sine Testing")


parser.add_argument('--no-timeout',  default=True, help="Disable timeout on waiting for bootloader reply, for debugging.")

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
my_interface = SerialTmclInterface(args.port)

# my_interface.enable_debug(True)
print(my_interface)

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
mcchelp = MccHelpersClass(my_interface)
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
myMotor = MotorConfigClass(my_interface, mySystem, UserEnableABN, UserABNResolution, UserABNDirection)
myMotor.config_motor(UserMotorType)

# 
# Restrict App/Axis Mode Velocity Meter changes - Stay in Mode 0 where velocity meter is based on period mode. 
# TODO: Find a better place to put this code.
#
try:    # TODO: Replace try/except with a check to see which mode the part is in and do not send command in register mode.
    my_interface.set_axis_parameter(137, 0, 0x07FFFFFFF) # 137 = Hardcoded address for VelocityMeterSwitchThreshold parameter.
except TMCLReplyStatusError as e:
    print("# WARNING: TM01 is in Register Mode, ignoring Axis Parameter Set.")
#
#
#

max_motor_current = UserMotorImax
target_motor_current_percentage = 75
target_voltageext_codes, current_limit_codes = myMotor.set_allowed_voltageext_level(max_motor_current, target_motor_current_percentage, ud_inc_volts=0.1, max_ud_codes=2000)

print("Target UD Codes = ", target_voltageext_codes)
print("Maximum Current Codes = ", current_limit_codes)


print("#")
print(" Step 2: Capture UD/FLUX data for Motor System ID.")
print("#")

# Sampling Frequency divisors
RAMDEBUG_DIVISOR = 1      # Measurement or Actual data is played at the PWM Rate / RAMDEBUG_DIVISOR
RAMDEBUG_RATE = PWM_FREQUENCY/RAMDEBUG_DIVISOR
RAMDEBUG_PERIOD = 1/RAMDEBUG_RATE
STIMULUS_DIVISOR = 1      # Stimulus or Target data is played at the PWM Rate / STIMULUS_DIVISOR
STIMULUS_RATE = PWM_FREQUENCY/STIMULUS_DIVISOR
STIMULUS_PERIOD = 1/STIMULUS_RATE

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
            "MCC.PID_TORQUE_FLUX_ACTUAL"
        ]
RAMDEBUG_SAMPLES = int(RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS)))

print("################ STARTING UD CAPTURE ##################")
mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000028) # VOLTAGE_EXT Mode
myUDStimCapture = StimulusCaptureClass(my_interface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, STIMULUS_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES)

ud_step_max = target_voltageext_codes
ud_step_start = 0 # int(target_voltageext_codes/2)
mcchelp.mcc_write(MCC.VOLTAGE_EXT, ud_step_start)
time.sleep(1)
raw_data = myUDStimCapture.RAMDebug_Setup_Capture_WithStep("MCC.VOLTAGE_EXT.UD", magnitude=ud_step_max, step_start=ud_step_start)

# Zero the UD/UQ voltages.
mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000028) # VOLTAGE_EXT Mode
myMotor.zero_voltageext()
myMotor.zero_torquefluxtarget()
myMotor.zero_torquefluxactual()
myMotor.zero_velocitytarget()
myMotor.zero_velocityactual()
myMotor.zero_positiontarget()
myMotor.zero_positionactual()
print("################ COMPLETED UD CAPTURE ##################")

data = myUDStimCapture.GrabExportData(raw_data, args.plot, args.plot_disappear, args.input, args.ud_output)

print("#")
print("# Done with Capture of UD/FLUX data for Motor System ID.")
print("#")


print("#")
print(" Step 3: Estimate Motor Rest and Lest and Estimate Torque/Flux Loop PI values.")
print("#")
myMotorTune = TorqueFluxSystemIDTuningClass(mySystem)
Rest, Lest, Tdelay = myMotorTune.estimate_R_L(data, "MCC.VOLTAGE_EXT.UD",  "MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL", period=RAMDEBUG_PERIOD)

# def estimate_R_L_delay(self, data, period=None, max_delay=4): 
print(" Returned Estimated Motor R = ", Rest)
print(" Returned Estimated Motor L = ", Lest)
print(" Returned Estimated Loop Delay = ", Tdelay)
print(" Estimated Motor Transfer Function 3dB Corner Frequency = ", myMotorTune.freq3dB,"Hz.")
input_zeta_i = float(args.damping_factor)
input_tuning_method = int(args.tuning_method)
input_gain_margin = float(args.gain_margin)
input_phase_margin = float(args.phase_margin) 
zeta_i = input_zeta_i
gain_margin = input_gain_margin
phase_margin = input_phase_margin
kp_float, ki_float, ki_float_dt = myMotorTune.CalculateTorqueFlux_PICoeffs(Rest, Lest, zeta_i, Tdelay=Tdelay, use_approach=input_tuning_method, Gm=gain_margin, Pm=phase_margin)
kp_int, ki_int, current_norm_p, current_norm_i = myMotorTune.ScaleTorqueFlux_PICoeffs(kp_float, ki_float_dt)

myMotor.zero_voltageext()
myMotor.zero_torquefluxtarget()
myMotor.zero_torquefluxactual()
myMotor.zero_velocitytarget()
myMotor.zero_velocityactual()
myMotor.zero_positiontarget()
myMotor.zero_positionactual()

print("#")
print(" Done with Estimation of Motor Rest and Lest and Estimate Torque/Flux Loop PI values.")
print("#")

time.sleep(5)

print("#")
print(" Step 5: Run Flux/Torque Loop Test.")
print("#")

### Connection setup ###########################################################
# my_interface = SerialTmclInterface(args.port)

# my_interface.enable_debug(True)
print(my_interface)
mcchelp = MccHelpersClass(my_interface)
myMotor = MotorConfigClass(my_interface, mySystem, UserEnableABN, UserABNResolution, UserABNDirection)
myMotor.zero_voltageext()
myMotor.zero_torquefluxtarget()
myMotor.zero_torquefluxactual()
myMotor.zero_velocitytarget()
myMotor.zero_velocityactual()
myMotor.zero_positiontarget()
myMotor.zero_positionactual()

# ABN encoder configuration (Init encoder (mode 0))
myMotor.init_ext_encoder(voltage_codes=target_voltageext_codes)

myMotor.zero_voltageext()
myMotor.zero_torquefluxtarget()
myMotor.zero_torquefluxactual()
myMotor.zero_velocitytarget()
myMotor.zero_velocityactual()
myMotor.zero_positiontarget()
myMotor.zero_positionactual()
time.sleep(1)

# ===== ABN encoder test drive =====

# Feedback selection

mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Mode, No Ramp
mcchelp.mcc_write(MCC.PID_TORQUE_COEFF, to_register32(kp_int, ki_int)) # Write Torque PI Coefficients.
mcchelp.mcc_write(MCC.PID_FLUX_COEFF,   to_register32(kp_int, ki_int)) # Write Flux PI Coefficients.
mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_P, current_norm_p) # Enable/Disable Torque/Flux P Normalization - 1 = Q0.16, 0 = Q8.8.
mcchelp.mcc_write(MCC.PID_CONFIG.CURRENT_NORM_I, current_norm_i) # Enable/Disable Torque/Flux I Normalization - 1 = Q0.16, 0 = Q8.8.
#
mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_LIMITS, to_register32(current_limit_codes, current_limit_codes)) # Write Torque/Flux Limit Values - 2.8A/Current Scale(RMS).
#
mcchelp.mcc_write(MCC.VELOCITY_CONFIG.MOVING_AVRG_FILTER_SAMPLES, 7) # Enable Velocity Moving Average Filter with 8-samples.

expected_tests =[ "Flux", "Torque"]
if args.current_loop_test_channel in expected_tests:
    current_loop_test = args.current_loop_test_channel
else:
    print("Current Loop Test Name (",args.current_loop_test_channel,") is invalid. Using Flux instead.")
    current_loop_test = "Flux"

if current_loop_test == "Flux":
    # Sampling Frequency divisors
    RAMDEBUG_DIVISOR = 1      # Measurement or Actual data is played at the PWM Rate / RAMDEBUG_DIVISOR
    RAMDEBUG_RATE = PWM_FREQUENCY/RAMDEBUG_DIVISOR
    RAMDEBUG_PERIOD = 1/RAMDEBUG_RATE
    STIMULUS_DIVISOR = 1      # Stimulus or Target data is played at the PWM Rate / STIMULUS_DIVISOR
    STIMULUS_RATE = PWM_FREQUENCY/STIMULUS_DIVISOR
    STIMULUS_PERIOD = 1/STIMULUS_RATE
    # Set at least one pretrigger to not miss the first value of the stimulus
    RAMDEBUG_PRETRIGGER_SAMPLES = 1 # Number of Pretrigger samples.
    CAPTURE_REGS = [
        "MCC.PID_TORQUE_FLUX_TARGET",           # <- This should be the stimulated register
        "MCC.PID_TORQUE_FLUX_ACTUAL",
        "MCC.FOC_UQ_UD_LIMITED"
    ]
    RAMDEBUG_SAMPLES = int(RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS)))
    myTorqueStimCapture = StimulusCaptureClass(my_interface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, STIMULUS_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES)
    time.sleep(1)
    id_current_step = int(current_limit_codes/15)
    raw_data = myTorqueStimCapture.RAMDebug_Setup_Capture_WithStep("MCC.PID_TORQUE_FLUX_TARGET.PID_FLUX_TARGET", id_current_step)
else:
    myMotor.init_abn_encoder(voltage_codes=target_voltageext_codes)
    # Sampling Frequency divisors
    RAMDEBUG_DIVISOR = 256      # Measurement or Actual data is played at the PWM Rate / RAMDEBUG_DIVISOR
    RAMDEBUG_RATE = PWM_FREQUENCY/RAMDEBUG_DIVISOR
    RAMDEBUG_PERIOD = 1/RAMDEBUG_RATE
    STIMULUS_DIVISOR = 256      # Stimulus or Target data is played at the PWM Rate / STIMULUS_DIVISOR
    STIMULUS_RATE = PWM_FREQUENCY/STIMULUS_DIVISOR
    STIMULUS_PERIOD = 1/STIMULUS_RATE
    # Set at least one pretrigger to not miss the first value of the stimulus
    RAMDEBUG_PRETRIGGER_SAMPLES = 1 # Number of Pretrigger samples.
    CAPTURE_REGS = [
        "MCC.PID_TORQUE_FLUX_TARGET",           # <- This should be the stimulated register
        "MCC.PID_TORQUE_FLUX_ACTUAL",
        "MCC.PID_VELOCITY_ACTUAL"
    ]
    RAMDEBUG_SAMPLES = int(RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS)))

    # Additional Test that Tianyu added - need to create and additional command line option for this.
    #
    myStimulusTorqueReaderClass = StimulusReaderClass(tcml_interface=my_interface, plot_stimulus=args.plot_stimulus, csv_file="../stimulus_data/torque_ramp_200.csv", no_timeout=args.no_timeout, verbose=args.verbose)
    myStimulusTorqueReaderClass.GenerateStimulusData()
    #

    myTorqueStimCapture = StimulusCaptureClass(my_interface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, STIMULUS_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES)
    time.sleep(1)
    iq_current_step = int(current_limit_codes/15)
    raw_data = myTorqueStimCapture.RAMDebug_Setup_Capture_WithStep("MCC.PID_TORQUE_FLUX_TARGET.PID_TORQUE_TARGET", iq_current_step)
    myMotor.init_ext_encoder(voltage_codes=target_voltageext_codes)


# Zero all inputs and outputs.
mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000) # Motor Stop.
myMotor.zero_voltageext()
myMotor.zero_torquefluxtarget()
myMotor.zero_torquefluxactual()
myMotor.zero_velocitytarget()
myMotor.zero_velocityactual()
myMotor.zero_positiontarget()
myMotor.zero_positionactual()
print("################ COMPLETED UD CAPTURE ##################")

data = myTorqueStimCapture.GrabExportData(raw_data, args.plot, False, args.input, args.torque_output)



if args.enable_flux_loop_test:

    print(" ####### Loop Through Flux Targets #############")

    # ABN encoder configuration (Init encoder (mode 0))
    myMotor.init_ext_encoder(voltage_codes=target_voltageext_codes)

    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001) # Swotch to Torque Mode.
    myMotor.zero_voltageext()
    myMotor.zero_torquefluxtarget()
    myMotor.zero_torquefluxactual()
    myMotor.zero_velocitytarget()
    myMotor.zero_velocityactual()
    myMotor.zero_positiontarget()
    myMotor.zero_positionactual()

    # Sampling Frequency divisors
    RAMDEBUG_DIVISOR = 1      # Measurement or Actual data is played at the PWM Rate / RAMDEBUG_DIVISOR
    RAMDEBUG_RATE = PWM_FREQUENCY/RAMDEBUG_DIVISOR
    RAMDEBUG_PERIOD = 1/RAMDEBUG_RATE
    STIMULUS_DIVISOR = 1      # Stimulus or Target data is played at the PWM Rate / STIMULUS_DIVISOR
    STIMULUS_RATE = PWM_FREQUENCY/STIMULUS_DIVISOR
    STIMULUS_PERIOD = 1/STIMULUS_RATE
    # Set at least one pretrigger to not miss the first value of the stimulus
    RAMDEBUG_PRETRIGGER_SAMPLES = 1 # Number of Pretrigger samples.
    CAPTURE_REGS = [
        "MCC.PID_TORQUE_FLUX_TARGET",           # <- This should be the stimulated register
        "MCC.PID_TORQUE_FLUX_ACTUAL",
        "MCC.FOC_UQ_UD_LIMITED"
    ]
    RAMDEBUG_SAMPLES = int(RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS)))

    myFluxLoopCapture = StimulusCaptureClass(my_interface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, STIMULUS_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES)

    for x in range(10, 0,-1):
        output = "../capture_data/motor_flux_current.csv"
        id_current_step = int(current_limit_codes/x)
        output_file = os.path.splitext(output)[0]+'_'+str(id_current_step)+'codes.csv'
        print("Output Filename = ", output_file)

        raw_data = myFluxLoopCapture.RAMDebug_Setup_Capture_WithStep("MCC.PID_TORQUE_FLUX_TARGET.PID_FLUX_TARGET", id_current_step)
        data = myFluxLoopCapture.GrabExportData(raw_data, args.plot, args.plot_disappear, args.input, output_file)

    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000) # Switched to Stopped.
    myMotor.zero_torquefluxtarget()
    myMotor.zero_torquefluxactual()
    myMotor.zero_velocitytarget()
    myMotor.zero_velocityactual()
    myMotor.zero_positiontarget()
    myMotor.zero_positionactual()

else:
    pass


if args.enable_prbs_test:

    print("################ UD PRBS TEST ##################")

    myMotor.init_ext_encoder(voltage_codes=target_voltageext_codes)
    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to Torque Mode, No Ramp
    print("#")
    print(" Step 6: Capture PRBS UD data to validate Motor System ID.")
    print("#")

    # Sampling Frequency divisors
    RAMDEBUG_DIVISOR = 1      # Measurement or Actual data is played at the PWM Rate / RAMDEBUG_DIVISOR
    RAMDEBUG_RATE = PWM_FREQUENCY/RAMDEBUG_DIVISOR
    RAMDEBUG_PERIOD = 1/RAMDEBUG_RATE
    STIMULUS_DIVISOR = 1      # Stimulus or Target data is played at the PWM Rate / STIMULUS_DIVISOR
    STIMULUS_RATE = PWM_FREQUENCY/STIMULUS_DIVISOR
    STIMULUS_PERIOD = 1/STIMULUS_RATE

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
                "MCC.FOC_UQ_UD_LIMITED",           # <- This should be the stimulated register
                "MCC.PID_TORQUE_FLUX_ACTUAL"
            ]
    RAMDEBUG_SAMPLES = int(RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS)))

    print("################ STARTING UD CAPTURE ##################")
    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000009) # PRBS UD Mode
    mcchelp.mcc_write(MCC.PRBS_AMPLITUDE.PRBS_AMPLITUDE, int(target_voltageext_codes*0.1))
    time.sleep(1)
    myPRBSUDStimCapture = StimulusCaptureClass(my_interface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, STIMULUS_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES)

    raw_data = myPRBSUDStimCapture.RAMDebug_Setup_CaptureNoTrigger()

    # Zero all inputs and outputs.
    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000) # Motor Stop.
    myMotor.zero_voltageext()
    myMotor.zero_torquefluxtarget()
    myMotor.zero_torquefluxactual()
    myMotor.zero_velocitytarget()
    myMotor.zero_velocityactual()
    myMotor.zero_positiontarget()
    myMotor.zero_positionactual()

    data_prbs_ud = myPRBSUDStimCapture.GrabExportData(raw_data, args.plot, False, args.input, args.prbs_ud_output)


    print("################ FLUX PRBS TEST ##################")

    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to Torque Mode, No Ramp
    print("#")
    print(" Step 7: Capture PRBS FLUX data to validate Motor System ID.")
    print("#")

    # ABN encoder configuration (Init encoder (mode 0))
    myMotor.init_ext_encoder(voltage_codes=target_voltageext_codes)


    # Sampling Frequency divisors
    RAMDEBUG_DIVISOR = 1      # Measurement or Actual data is played at the PWM Rate / RAMDEBUG_DIVISOR
    RAMDEBUG_RATE = PWM_FREQUENCY/RAMDEBUG_DIVISOR
    RAMDEBUG_PERIOD = 1/RAMDEBUG_RATE
    STIMULUS_DIVISOR = 1      # Stimulus or Target data is played at the PWM Rate / STIMULUS_DIVISOR
    STIMULUS_RATE = PWM_FREQUENCY/STIMULUS_DIVISOR
    STIMULUS_PERIOD = 1/STIMULUS_RATE

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
        "MCC.PIDIN_TORQUE_FLUX_TARGET",           # <- This should be the stimulated register
        "MCC.PID_TORQUE_FLUX_ACTUAL",
        "MCC.FOC_UQ_UD_LIMITED"
        ]
    RAMDEBUG_SAMPLES = int(RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS)))

    print("################ STARTING UD CAPTURE ##################")
    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000004) # PRBS Flux Mode = 4, PRBS Torque Mode = 5. 
    mcchelp.mcc_write(MCC.PRBS_AMPLITUDE.PRBS_AMPLITUDE, int(current_limit_codes*0.1))
    time.sleep(1)
    myPRBSUDStimCapture = StimulusCaptureClass(my_interface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, STIMULUS_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES)
    
    raw_data = myPRBSUDStimCapture.RAMDebug_Setup_CaptureNoTrigger()
    
    # Zero all inputs and outputs.
    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000) # Motor Stop.
    myMotor.zero_voltageext()
    myMotor.zero_torquefluxtarget()
    myMotor.zero_torquefluxactual()
    myMotor.zero_velocitytarget()
    myMotor.zero_velocityactual()
    myMotor.zero_positiontarget()
    myMotor.zero_positionactual()

    data_prbs_flux = myPRBSUDStimCapture.GrabExportData(raw_data, args.plot, False, args.input, args.prbs_flux_output)

    myFFTClass = MotorDataFFTClass()
    myFFTClass.fft(data_prbs_ud,RAMDEBUG_PERIOD)
    myFFTClass.fft(data_prbs_flux,RAMDEBUG_PERIOD)

else:
    pass


if args.enable_sine_test:

    print("################ STARTING MULTI-TONE FLUX SINE TEST ##################")

    myStimulusFluxReaderClass = StimulusReaderClass(tcml_interface=my_interface, plot_stimulus=args.plot_stimulus, csv_file=args.sine_csv_file, no_timeout=args.no_timeout, verbose=args.verbose)
    myStimulusFluxReaderClass.GenerateStimulusData()

    # ABN encoder configuration (Init encoder (mode 0))
    myMotor.init_ext_encoder(voltage_codes=target_voltageext_codes)


    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)     # Switch to Stopped.
    myMotor.zero_voltageext()
    myMotor.zero_torquefluxtarget()
    myMotor.zero_torquefluxactual()
    myMotor.zero_velocitytarget()
    myMotor.zero_velocityactual()
    myMotor.zero_positiontarget()
    myMotor.zero_positionactual()

    print("#")
    print(" Step 7: Capture MULTI-TONE FLUX SINE data to validate Motor System ID.")
    print("#")

    # Sampling Frequency divisors
    RAMDEBUG_DIVISOR = 1      # Measurement or Actual data is played at the PWM Rate / RAMDEBUG_DIVISOR
    RAMDEBUG_RATE = PWM_FREQUENCY/RAMDEBUG_DIVISOR
    RAMDEBUG_PERIOD = 1/RAMDEBUG_RATE
    STIMULUS_DIVISOR = 1      # Stimulus or Target data is played at the PWM Rate / STIMULUS_DIVISOR
    STIMULUS_RATE = PWM_FREQUENCY/STIMULUS_DIVISOR
    STIMULUS_PERIOD = 1/STIMULUS_RATE

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
        "MCC.PIDIN_TORQUE_FLUX_TARGET",           # <- This should be the stimulated register
        "MCC.PID_TORQUE_FLUX_ACTUAL",
        "MCC.FOC_UQ_UD_LIMITED",
        "MCC.PID_VELOCITY_ACTUAL"
        ]
    RAMDEBUG_SAMPLES = int(RAMDEBUG_CAPACITY/(4*len(CAPTURE_REGS)))

    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001) # Switch to Torque Mode.
    myMotor.zero_voltageext()
    myMotor.zero_torquefluxtarget()
    myMotor.zero_torquefluxactual()
    myMotor.zero_velocitytarget()
    myMotor.zero_velocityactual()
    myMotor.zero_positiontarget()
    myMotor.zero_positionactual()

    mySineFluxStimCapture = StimulusCaptureClass(my_interface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, STIMULUS_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES)
    
    stimilus_target_register = "MCC.PID_TORQUE_FLUX_TARGET"
    raw_data = mySineFluxStimCapture.RAMDebug_Setup_Capture(stim_register0=stimilus_target_register, stim_scalar0=1)
   
    print(len(raw_data))
    data = mySineFluxStimCapture.GrabExportData(raw_data, args.plot, False, args.input, args.sine_flux_output)

    mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000) # Switched to Stopped.
    myMotor.zero_torquefluxtarget()
    myMotor.zero_torquefluxactual()
    myMotor.zero_velocitytarget()
    myMotor.zero_velocityactual()
    myMotor.zero_positiontarget()
    myMotor.zero_positionactual()

else:
    pass


# myTorqueProfile = TorqueProfileClass(my_interface, kp_int, ki_int, "../capture_data/torque_profile.csv")
# myTorqueProfile.run_torque_profile()

# myUQProfile = UQProfileClass(my_interface, UserMotorPoles, Rest, Lest, "../capture_data/uq_profile.csv")
# K_est = myUQProfile.run_uq_profile()
# print("# Motor Constant, K Estimate = ", K_est)

# myStimulusTorqueReaderClass = StimulusReaderClass(tcml_interface=my_interface, plot_stimulus=args.plot_stimulus, csv_file=args.torque_csv_file, no_timeout=args.no_timeout, verbose=args.verbose)
# myStimulusTorqueReaderClass.GenerateStimulusData()


print("Testing Complete - Re-Initializing Encoder and Switching to ABN Mode.")

myMotor.init_abn_encoder(voltage_codes=target_voltageext_codes)

print("Done.")
