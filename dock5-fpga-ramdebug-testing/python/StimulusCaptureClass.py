################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
from datetime import datetime
import os
import time
from sys import setprofile
import matplotlib.pyplot as plot

from pytrinamic.connections.connection_manager import ConnectionManager
from pytrinamic.connections.serial_tmcl_interface import SerialTmclInterface
from pytrinamic.tmcl import TMCLCommand, TMCLReplyChecksumError
import pytrinamic.RAMDebug as RAMDebug

from MccHelpersClass import MccHelpersClass
from motorDataFileFunctionsClass import motorDataFileFunctionsClass
from PlotHelperClass import PlotHelperClass
from register_helpers import to_register32
from collections import OrderedDict
from FirmwareUpgradeClass import VelocityWithFeedForward

import pathlib
MY_SCRIPT_DIR = pathlib.Path(__file__).parent
# This trick is necessary to get the import working from a different folder
# that isn't a subfolder
sys.path.insert(1, str(MY_SCRIPT_DIR / '../../UBL'))
from MemoryMap import TM01

# Grab the address blocks we care about
MCC      = TM01.MCC

class StimulusCaptureClass:
    
    def __init__(self, interface, RAMDEBUG_SAMPLES, RAMDEBUG_DIVISOR, STIMULUS_DIVISOR, CAPTURE_REGS, RAMDEBUG_PERIOD, RAMDEBUG_PRETRIGGER_SAMPLES):
        self.my_interface = interface
        self.RAMDEBUG_SAMPLES = RAMDEBUG_SAMPLES
        self.RAMDEBUG_DIVISOR = RAMDEBUG_DIVISOR
        self.STIMULUS_DIVISOR = STIMULUS_DIVISOR
        self.CAPTURE_REGS = CAPTURE_REGS
        self.RAMDEBUG_PERIOD = RAMDEBUG_PERIOD
        self.RAMDEBUG_PRETRIGGER_SAMPLES = RAMDEBUG_PRETRIGGER_SAMPLES

        ### Constants ##################################################################
        self.GP_STIMULUS_FSM = 25
        self.GP_STIMULUS_DIVISOR = 26
        self.GP_STIMULUS_SET_CHANNEL_0 = 27
        self.GP_STIMULUS_SET_CHANNEL_1 = 28
        self.GP_STIMULUS_SET_CHANNEL_SCALER_0 = 29
        self.GP_STIMULUS_SET_CHANNEL_SCALER_1 = 30

        self.STIMULUS_PARTITION = 1

        self.mcchelp = MccHelpersClass(self.my_interface)

    def RAMDebug_Setup_Capture(self, stim_register0=None, stim_register1=None, stim_scalar0:int=1, stim_scalar1:int=1):

        print("#")
        print("# Flash Stimulus / Capture: Only one stimulus channel is currently supported.")
        print("# Stimulus Register = ", stim_register0)
        print("# Stimulus Scalar   = ", str(stim_scalar0))
        print("#")

        ### Connection setup ###########################################################

        print(self.my_interface)

        self.size_idx = self.my_interface.send(65, 0, 0, 0).value
        self.sector_idx = self.my_interface.send(65, 0, 0, 1).value
        self.partition_count = self.my_interface.send(65, 0, 0, 2).value
        print(f"Flash chip size:   0x{1<<self.size_idx if self.size_idx else 1<<24:08X}")
        print(f"Flash sector size: 0x{1<<self.sector_idx:08X}")
        print(f"Flash partitions:  {self.partition_count}")

        for i in range(self.partition_count):
            self.partition_start = self.my_interface.send(65, 1, i, 4).value
            self.partition_size  = self.my_interface.send(65, 1, i, 5).value
            self.partition_end   = self.partition_start + self.partition_size
            print(f"Partition {i}: 0x{self.partition_start:08X}-0x{self.partition_end:08X}")

        ### RAMDebug capture setup #####################################################
        MOTOR = 0 # The motor number used in the system - we only have one motor

        # Create a RAMDebug object. This represents a measurement
        measurement = RAMDebug.RAMDebug(self.my_interface)

        # Setup the capture settings
        measurement.set_sample_count(self.RAMDEBUG_SAMPLES) # This is per-channel samples
        measurement.set_divider(self.RAMDEBUG_DIVISOR) # Measurement frequency divider

        # Add the channels to the measurement
        for reg in self.CAPTURE_REGS:
            mcc_reg = MCC.find(reg)
            reg_addr = self.mcchelp.get_reg_addr(mcc_reg)[1]
            is_signed = mcc_reg.SIGNED
            measurement.set_channel(RAMDebug.Channel.register(MOTOR, reg_addr, signed=is_signed))

        # Setup the trigger
        # Set at least one pretrigger to not miss the first value of the stimulus
        # Note that RAMDebug checks the trigger via a greater-than comparison.
        # A change of values between 0 and 1 therefore requires a trigger threshold of 0
        measurement.set_pretrigger_samples(self.RAMDEBUG_PRETRIGGER_SAMPLES) # per-channel samples
        measurement.set_trigger(
            RAMDebug.Channel.global_parameter(MOTOR, self.GP_STIMULUS_FSM),
            RAMDebug.RAMDebug_Trigger.TRIGGER_DUAL_EDGE_UNSIGNED,
            2
            )

        # Start the measurement
        measurement.start_measurement(strict=False)

        # Wait for RAMDebug to be ready to trigger
        print("Waiting for pretrigger")
        while measurement.is_pretriggering():
            pass

        # Start the stimulus state machine

        mcc_target0 = MCC.find(stim_register0)
        self.my_interface.set_global_parameter(self.GP_STIMULUS_SET_CHANNEL_0, 0, self.mcchelp.get_reg_addr(mcc_target0)[1])
        # mcc_target1 = MCC.find(stim_register1)
        # self.my_interface.set_global_parameter(self.GP_STIMULUS_SET_CHANNEL_1, 0, self.mcchelp.get_reg_addr(mcc_target1)[1])
        self.my_interface.set_global_parameter(self.GP_STIMULUS_SET_CHANNEL_SCALER_0, 0, stim_scalar0)
        self.my_interface.set_global_parameter(self.GP_STIMULUS_SET_CHANNEL_SCALER_1, 0, stim_scalar1)
        self.my_interface.set_global_parameter(self.GP_STIMULUS_DIVISOR, 0, self.STIMULUS_DIVISOR)
        self.my_interface.set_global_parameter(self.GP_STIMULUS_FSM, 0, self.STIMULUS_PARTITION)

        # Wait for the measurement to be done
        print("Waiting for measurement completion")
        while not measurement.is_measurement_done():
            pass

        # Download the measurement data
        print("Downloading measurement data")
        raw_data = measurement.get_samples()

        return raw_data

    def RAMDebug_Setup_Capture_NoStep(self, register, magnitude):
        ### RAMDebug capture setup #####################################################
        MOTOR = 0 # The motor number used in the system - we only have one motor

        print("Step Target Register = ", register)        
        mcc_target = MCC.find(register)
        self.mcchelp.mcc_write(mcc_target, magnitude) # Set the Target Register Value to Step-Start Value (default=0).
    
        # Create a RAMDebug object. This represents a measurement
        measurement = RAMDebug.RAMDebug(self.my_interface)

        # Setup the capture settings
        measurement.set_sample_count(self.RAMDEBUG_SAMPLES) # This is per-channel samples
        measurement.set_divider(self.RAMDEBUG_DIVISOR) # Measurement frequency divider

        # Add the channels to the measurement
        for reg in self.CAPTURE_REGS:
            mcc_reg = MCC.find(reg)
            reg_addr = self.mcchelp.get_reg_addr(mcc_reg)[1]
            is_signed = mcc_reg.SIGNED
            measurement.set_channel(RAMDebug.Channel.register(MOTOR, reg_addr, signed=is_signed))

        # Setup the trigger - Unconditional in this case.
        # Is there a way to use the pretrigger function to better align the Step timing with the start of capture.
        measurement.set_pretrigger_samples(self.RAMDEBUG_PRETRIGGER_SAMPLES) # per-channel samples
        measurement.set_trigger(
            RAMDebug.Channel.register(MOTOR, address=self.mcchelp.get_reg_addr(mcc_target)[1]), # needs signed argument
            RAMDebug.RAMDebug_Trigger.TRIGGER_UNCONDITIONAL, # depends on start_step vs magnitude and register signedness
            2
            )
       
        # Start the measurement
        measurement.start_measurement(strict=False)

        # Wait for RAMDebug to be ready to trigger
        print("Waiting for pretrigger")
        while measurement.is_pretriggering():
            pass

        # Wait for the measurement to be done
        print("Waiting for measurement completion")
        while not measurement.is_measurement_done():
            pass

        self.mcchelp.mcc_write(mcc_target, 0)
        # Download the measurement data
        print("Downloading measurement data")
        raw_data = measurement.get_samples()

        return raw_data

    def RAMDebug_Setup_Capture_WithVelocityStep(self, magnitude, *, step_start=0, delay=4, feedforward_params=None, use_firmware_upgrade_method=False):
        ### RAMDebug capture setup #####################################################
        MOTOR = 0 # The motor number used in the system - we only have one motor
        mcc_velocity_target = MCC.find("MCC.PID_VELOCITY_TARGET")
        mcc_torque_offset = MCC.find("MCC.PID_TORQUE_FLUX_OFFSET")
        if use_firmware_upgrade_method:
            velocity_writer = VelocityWithFeedForward(self.my_interface, feedforward_params)
            velocity_writer.mcc_write(0)
            velocity_writer.mcc_write(step_start)
        else:
            self.mcchelp.mcc_write(mcc_velocity_target, 0) # Set the Target Register Value to Step-Start Value (default=0).
            self.mcchelp.mcc_write(mcc_torque_offset, 0) # Set the Target Register Value to Step-Start Value (default=0).
            self.mcchelp.mcc_write(mcc_velocity_target, step_start) # Set the Target Register Value to Step-Start Value (default=0).
    
        # Create a RAMDebug object. This represents a measurement
        measurement = RAMDebug.RAMDebug(self.my_interface)

        # Setup the capture settings
        measurement.set_sample_count(self.RAMDEBUG_SAMPLES) # This is per-channel samples
        measurement.set_divider(self.RAMDEBUG_DIVISOR) # Measurement frequency divider

        # Add the channels to the measurement
        for reg in self.CAPTURE_REGS:
            mcc_reg = MCC.find(reg)
            reg_addr = self.mcchelp.get_reg_addr(mcc_reg)[1]
            is_signed = mcc_reg.SIGNED
            measurement.set_channel(RAMDebug.Channel.register(MOTOR, reg_addr, signed=is_signed))

        # Setup the trigger - Unconditional in this case.
        # Is there a way to use the pretrigger function to better align the Step timing with the start of capture.
        measurement.set_pretrigger_samples(self.RAMDEBUG_PRETRIGGER_SAMPLES) # per-channel samples
        measurement.set_trigger(
            RAMDebug.Channel.register(MOTOR, address=self.mcchelp.get_reg_addr(mcc_velocity_target)[1]), # needs signed argument
            RAMDebug.RAMDebug_Trigger.TRIGGER_UNCONDITIONAL, # depends on start_step vs magnitude and register signedness
            2
            )
       
        # Start the measurement
        measurement.start_measurement(strict=False)

        # Wait for RAMDebug to be ready to trigger
        print("Waiting for pretrigger")
        while measurement.is_pretriggering():
            pass

        # Write the Register Step Value: Note that the timing is not synchronized.
        #
        time.sleep(delay*self.RAMDEBUG_PERIOD)
        if use_firmware_upgrade_method:
            velocity_writer.mcc_write(magnitude)
        else:
            self.mcchelp.mcc_write(mcc_torque_offset, 0) # Set the Target Register Value to Step-Start Value (default=0).
            self.mcchelp.mcc_write(mcc_velocity_target, magnitude) # Set the Target Register Value to Step-Start Value (default=0).
        
        # Wait for the measurement to be done
        print("Waiting for measurement completion")
        while not measurement.is_measurement_done():
            pass

        if use_firmware_upgrade_method:
            velocity_writer.mcc_write(0)
        else:
            self.mcchelp.mcc_write(mcc_torque_offset, 0) # Set the Target Register Value to Step-Start Value (default=0).
            self.mcchelp.mcc_write(mcc_velocity_target, 0) # Set the Target Register Value to Step-Start Value (default=0).

        # Download the measurement data
        print("Downloading measurement data")
        raw_data = measurement.get_samples()

        return raw_data


    def RAMDebug_Setup_Capture_WithStep(self, register, magnitude, *, step_start=0, idle_value=0):
        ### RAMDebug capture setup #####################################################
        MOTOR = 0 # The motor number used in the system - we only have one motor

        if magnitude == step_start:
            raise ValueError("Step start and magnitude must not be equal")

        print("Step Target Register = ", register)        
        mcc_target = MCC.find(register)
        self.mcchelp.mcc_write(mcc_target, step_start) # Set the Target Register Value to Step-Start Value (default=0).
    
        # Create a RAMDebug object. This represents a measurement
        measurement = RAMDebug.RAMDebug(self.my_interface)

        if measurement.MAX_ELEMENTS < self.RAMDEBUG_SAMPLES:
            raise RuntimeError(f"Too many samples requested! Requested {self.RAMDEBUG_SAMPLES}, have {measurement.MAX_ELEMENTS}")

        # Setup the capture settings
        measurement.set_sample_count(self.RAMDEBUG_SAMPLES) # This is per-channel samples
        measurement.set_divider(self.RAMDEBUG_DIVISOR) # Measurement frequency divider

        # Add the channels to the measurement
        for reg in self.CAPTURE_REGS:
            mcc_reg = MCC.find(reg)
            reg_addr = self.mcchelp.get_reg_addr(mcc_reg)[1]
            is_signed = mcc_reg.SIGNED
            measurement.set_channel(RAMDebug.Channel.register(MOTOR, reg_addr, signed=is_signed))

        # Setup the trigger - Unconditional in this case.
        # Is there a way to use the pretrigger function to better align the Step timing with the start of capture.
        trigger_type = RAMDebug.RAMDebug_Trigger.TRIGGER_DUAL_EDGE_SIGNED if mcc_target.SIGNED else RAMDebug.RAMDebug_Trigger.TRIGGER_DUAL_EDGE_UNSIGNED
        # The triggering mechanism checks for value > threshold.
        # To ensure we always have a valid threshold, place it just below the upper value
        trigger_threshold = max(magnitude, step_start) - 1

        measurement.set_pretrigger_samples(self.RAMDEBUG_PRETRIGGER_SAMPLES) # per-channel samples
        measurement.set_trigger(
            RAMDebug.Channel.register(MOTOR, address=self.mcchelp.get_reg_addr(mcc_target)[1]), # needs signed argument
            trigger_type,
            trigger_threshold
            )
       
        # Start the measurement
        measurement.start_measurement(strict=False)

        # Wait for RAMDebug to be ready to trigger
        print("Waiting for pretrigger")
        while measurement.is_pretriggering():
            pass

        # Write the Register Step Value
        self.mcchelp.mcc_write(mcc_target, magnitude)

        # Wait for the measurement to be done
        print("Waiting for measurement completion")
        while not measurement.is_measurement_done():
            pass

        self.mcchelp.mcc_write(mcc_target, idle_value)

        # Download the measurement data
        print("Downloading measurement data")
        raw_data = measurement.get_samples()

        return raw_data


    def RAMDebug_Setup_Capture_WithSquareWave(self, register, magnitude ):
        ### RAMDebug capture setup #####################################################
        MOTOR = 0 # The motor number used in the system - we only have one motor

        print("Step Target Register = ", register)        
        mcc_target = MCC.find(register)
        self.mcchelp.mcc_write(mcc_target, 0) # Set the Target Register Value to Step-Start Value (default=0).
    
        # Create a RAMDebug object. This represents a measurement
        measurement = RAMDebug.RAMDebug(self.my_interface)

        # Setup the capture settings
        measurement.set_sample_count(self.RAMDEBUG_SAMPLES) # This is per-channel samples
        measurement.set_divider(self.RAMDEBUG_DIVISOR) # Measurement frequency divider

        # Add the channels to the measurement
        for reg in self.CAPTURE_REGS:
            mcc_reg = MCC.find(reg)
            reg_addr = self.mcchelp.get_reg_addr(mcc_reg)[1]
            is_signed = mcc_reg.SIGNED
            measurement.set_channel(RAMDebug.Channel.register(MOTOR, reg_addr, signed=is_signed))

        # Setup the trigger - Unconditional in this case.
        # Is there a way to use the pretrigger function to better align the Step timing with the start of capture.
        measurement.set_pretrigger_samples(self.RAMDEBUG_PRETRIGGER_SAMPLES) # per-channel samples
        measurement.set_trigger(
            RAMDebug.Channel.register(MOTOR, address=self.mcchelp.get_reg_addr(mcc_target)[1]), # needs signed argument
            RAMDebug.RAMDebug_Trigger.TRIGGER_UNCONDITIONAL, # depends on start_step vs magnitude and register signedness
            2
            )
       
        # Start the measurement
        measurement.start_measurement(strict=False)

        # Wait for RAMDebug to be ready to trigger
        print("Waiting for pretrigger")
        while measurement.is_pretriggering():
            pass

        # Write the Register Square Value: Note that the timing is not synchronized.
        #
        delay_samples = int(self.RAMDEBUG_SAMPLES/5)
        # 1. +ve Step.
        # time.sleep(delay_samples*self.RAMDEBUG_PERIOD)
        self.mcchelp.mcc_write(mcc_target, int(magnitude*1.0)) # Set the Target Register Value to the input Magnitude Value.
        # 2. +ve Step.
        time.sleep(delay_samples*self.RAMDEBUG_PERIOD)
        self.mcchelp.mcc_write(mcc_target, int(magnitude*0.75)) # Set the Target Register Value to the input Magnitude Value.
        # 3. -ve Step.
        time.sleep(delay_samples*self.RAMDEBUG_PERIOD)
        self.mcchelp.mcc_write(mcc_target, int(magnitude*(-1.0))) # Set the Target Register Value to the input Magnitude Value.
        # 4. -ve Step.
        time.sleep(delay_samples*self.RAMDEBUG_PERIOD)
        self.mcchelp.mcc_write(mcc_target, int(magnitude*(-0.75))) # Set the Target Register Value to the input Magnitude Value.
        # 5. zero Step.
        time.sleep(delay_samples*self.RAMDEBUG_PERIOD)
        self.mcchelp.mcc_write(mcc_target, 0) # Set the Target Register Value to the input Magnitude Value.


        # Wait for the measurement to be done
        print("Waiting for measurement completion")
        while not measurement.is_measurement_done():
            pass

        self.mcchelp.mcc_write(mcc_target, 0)
        # Download the measurement data
        print("Downloading measurement data")
        raw_data = measurement.get_samples()

        return raw_data






    def RAMDebug_Setup_CaptureNoTrigger(self):
        ### RAMDebug capture setup #####################################################
        MOTOR = 0 # The motor number used in the system - we only have one motor

        # Create a RAMDebug object. This represents a measurement
        measurement = RAMDebug.RAMDebug(self.my_interface)

        # Setup the capture settings
        measurement.set_sample_count(self.RAMDEBUG_SAMPLES) # This is per-channel samples
        measurement.set_divider(self.RAMDEBUG_DIVISOR) # Measurement frequency divider

        # Add the channels to the measurement
        for reg in self.CAPTURE_REGS:
            mcc_reg = MCC.find(reg)
            reg_addr = self.mcchelp.get_reg_addr(mcc_reg)[1]
            is_signed = mcc_reg.SIGNED
            measurement.set_channel(RAMDebug.Channel.register(MOTOR, reg_addr, signed=is_signed))

        # Start the measurement
        measurement.start_measurement(strict=False)

        # Wait for the measurement to be done
        print("Waiting for measurement completion")
        while not measurement.is_measurement_done():
            pass

        # Download the measurement data
        print("Downloading measurement data")
        raw_data = measurement.get_samples()

        return raw_data

    def GrabExportData(self, raw_data, plot_enable=False, plot_disappear=True,input=None,output=None):

        # ToDo: Add this info parameter to RAMDebug class in pytrinamic
        trigger_prescaler = self.my_interface.send(142, 10, 0, 4).value
            
        # Grab the captured data
        data = OrderedDict()

        # If we have an input file, grab the stimulus data from it
        if input:
            # Read the stimulus input file
            myReadFileIO = motorDataFileFunctionsClass(input)
            myReadFileIO.read()
            raw_stimulus = myReadFileIO.read_data_list

            # RAMDebug trigger happens when stimulus is just starting, it takes another stimulus cycle
            # for the first stimulus value to be written -> Prepend a value
            raw_stimulus.insert(0, 0)

            stimulus = []

            # Scale stimulus up to PWM frequency by repeating each value
            for value in raw_stimulus:
                stimulus += [value]*self.STIMULUS_DIVISOR
    
            # Scale down to RAMDebug frequency
            # This sampling of the PWM frequency scale values must check for the exact trigger point
            # relative to the RAMDebug frequency in order to grab the right samples.
            stimulus = stimulus[(-1-trigger_prescaler)%self.RAMDEBUG_DIVISOR::self.RAMDEBUG_DIVISOR]
            
            # Prepend pretrigger samples
            stimulus = [0]*self.RAMDEBUG_PRETRIGGER_SAMPLES + stimulus

            # If needed, pad the stimulus data
            if len(stimulus) < len(raw_data[0]):
                # We have less stimulus samples than RAMDebug samples -> repeat the last sample
                stimulus += [stimulus[-1]]*(len(raw_data[0]) - len(stimulus))

            # Split the stimulus register data into its respective fields
            for field in myReadFileIO.register.get_fields():
                values = list([field.read(x) for x in stimulus])
                data["Input:" + field.get_name()] = values

        # Split each captured register into the corresponding fields
        for reg_name, values in zip(self.CAPTURE_REGS, raw_data):
            # Grab the MCC memory map register object
            reg = MCC.find(reg_name)
            fields = reg.get_fields()
            for field in fields:
                data[field.get_name()] = list([field.read(x) for x in values])

        if output:
            # Create the Filename with Data and time stamp.
            now = datetime.now() # current date and time
            date_time = now.strftime("%Y%m%d_%H%M%S")
            print("date and time:",date_time)
            output_file = os.path.splitext(output)[0]+'_'+date_time+'.csv'
            print("Output Filename = ", output_file)

            # Write the data to a file.
            myFileIO = motorDataFileFunctionsClass(output_file)
            myFileIO.write(data, self.RAMDEBUG_PERIOD)

        # Plot the Data.
        myDataPlot = PlotHelperClass()
        myDataPlot.PlotData(data, plot_enable, plot_disappear)

        return data

    
    def RAMDebug_Capture_Velocity_Profile(self, trajectory, feedforward_params, use_firmware_upgrade_method):
        ### RAMDebug capture setup #####################################################
        MOTOR = 0 # The motor number used in the system - we only have one motor
        mcc_velocity_target = MCC.find("MCC.PID_VELOCITY_TARGET")
        mcc_torque_offset = MCC.find("MCC.PID_TORQUE_FLUX_OFFSET")
        if use_firmware_upgrade_method:
            velocity_writer = VelocityWithFeedForward(self.my_interface, feedforward_params)
            velocity_writer.mcc_write(0)
        else:
            self.mcchelp.mcc_write(mcc_velocity_target, 0) # Set the Target Register Value to Step-Start Value (default=0).
            self.mcchelp.mcc_write(mcc_torque_offset, 0) # Set the Target Register Value to Step-Start Value (default=0).
    
        # Create a RAMDebug object. This represents a measurement
        measurement = RAMDebug.RAMDebug(self.my_interface)

        # Setup the capture settings
        measurement.set_sample_count(self.RAMDEBUG_SAMPLES) # This is per-channel samples
        measurement.set_divider(self.RAMDEBUG_DIVISOR) # Measurement frequency divider

        # Add the channels to the measurement
        for reg in self.CAPTURE_REGS:
            mcc_reg = MCC.find(reg)
            reg_addr = self.mcchelp.get_reg_addr(mcc_reg)[1]
            is_signed = mcc_reg.SIGNED
            measurement.set_channel(RAMDebug.Channel.register(MOTOR, reg_addr, signed=is_signed))

        # Setup the trigger - Unconditional in this case.
        # Is there a way to use the pretrigger function to better align the Step timing with the start of capture.
        measurement.set_pretrigger_samples(self.RAMDEBUG_PRETRIGGER_SAMPLES) # per-channel samples
        measurement.set_trigger(
            RAMDebug.Channel.register(MOTOR, address=self.mcchelp.get_reg_addr(mcc_velocity_target)[1]), # needs signed argument
            RAMDebug.RAMDebug_Trigger.TRIGGER_UNCONDITIONAL, # depends on start_step vs magnitude and register signedness
            2
            )

       
        # Start the measurement
        measurement.start_measurement(strict=False)

        # Wait for RAMDebug to be ready to trigger
        print("Waiting for pretrigger")
        while measurement.is_pretriggering():
            pass

        # Publish the profile
        t_now = 0
        t_ref = time.time_ns()*1e-9 # in seconds
        while t_now < trajectory.get_t_end():
            # Get the target velocity
            target_velocity_codes = trajectory.get_velocity(t_now)
            if use_firmware_upgrade_method:
                velocity_writer.mcc_write(target_velocity_codes)
            else:
                # Get the target acceleration
                target_acceleration_codes = trajectory.get_acceleration(t_now)
                # Compute the required feedforward torque to compensate for friction and inertia
                velocity_sign = 1 if target_velocity_codes > 0 else (-1 if target_velocity_codes < 0 else 0)
                target_torque_offset_codes = feedforward_params.fs*velocity_sign + feedforward_params.bv*target_velocity_codes + feedforward_params.J*target_acceleration_codes
                # Convert the velocity and torque offset to integer or hexadecimal, as needed for the registers
                target_velocity_codes = int(target_velocity_codes)
                target_torque_offset_codes = to_register32(int(target_torque_offset_codes), 0)
                # Publish the new target velocity and torque offset to the chip
                self.mcchelp.mcc_write(mcc_torque_offset, target_torque_offset_codes)
                self.mcchelp.mcc_write(mcc_velocity_target, target_velocity_codes)
            # Update the timestamp for the next iteration
            t_now = time.time_ns()*1e-9 - t_ref

        # Make sure the profile ends on a null value for the velocity and torque offset registers
        if use_firmware_upgrade_method:
            velocity_writer.mcc_write(0)
        else:
            self.mcchelp.mcc_write(mcc_torque_offset, 0)
            self.mcchelp.mcc_write(mcc_velocity_target, 0)

        # Wait for the measurement to be done
        print("Waiting for measurement completion")
        while not measurement.is_measurement_done():
            pass

        # Download the measurement data
        print("Downloading measurement data")
        raw_data = measurement.get_samples()

        return raw_data
    """
    def RAMDebug_Capture_Velocity_Profile_from_stimulus(self):
        ### RAMDebug capture setup #####################################################
        MOTOR = 0 # The motor number used in the system - we only have one motor

        # Create a RAMDebug object. This represents a measurement
        measurement = RAMDebug.RAMDebug(self.my_interface)

        # Setup the capture settings
        measurement.set_sample_count(self.RAMDEBUG_SAMPLES) # This is per-channel samples
        measurement.set_divider(self.RAMDEBUG_DIVISOR) # Measurement frequency divider

        # Add the channels to the measurement
        for reg in self.CAPTURE_REGS:
            mcc_reg = MCC.find(reg)
            reg_addr = self.mcchelp.get_reg_addr(mcc_reg)[1]
            is_signed = mcc_reg.SIGNED
            measurement.set_channel(RAMDebug.Channel.register(MOTOR, reg_addr, signed=is_signed))

        # Setup the trigger
        # Set at least one pretrigger to not miss the first value of the stimulus
        # Note that RAMDebug checks the trigger via a greater-than comparison.
        # A change of values between 0 and 1 therefore requires a trigger threshold of 0
        measurement.set_pretrigger_samples(self.RAMDEBUG_PRETRIGGER_SAMPLES) # per-channel samples
        if self._use_legacy_aps:
            trigger_channel = RAMDebug.Channel.axis_parameter(MOTOR, self.AP_STIMULUS_FSM)
        else:
            trigger_channel = RAMDebug.Channel.global_parameter(MOTOR, self.GP_STIMULUS_FSM)

        measurement.set_trigger(
            trigger_channel,
            RAMDebug.RAMDebug_Trigger.TRIGGER_RISING_EDGE_UNSIGNED,
            2
            )

        # Start the measurement
        measurement.start_measurement(strict=False)

        # Wait for RAMDebug to be ready to trigger
        print("Waiting for pretrigger")
        while measurement.is_pretriggering():
            pass

        # Start the stimulus state machine
        if self._use_legacy_aps:
            self.my_interface.set_axis_parameter(self.AP_STIMULUS_DIVISOR, 0, self.STIMULUS_DIVISOR)
            self.my_interface.set_axis_parameter(self.AP_STIMULUS_FSM, 0, self.STIMULUS_ADDRESS)
        else:
            self.my_interface.set_global_parameter(self.GP_STIMULUS_DIVISOR, 0, self.STIMULUS_DIVISOR)
            self.my_interface.set_global_parameter(self.GP_STIMULUS_FSM, 0, self.STIMULUS_ADDRESS)

        # Wait for the measurement to be done
        print("Waiting for measurement completion")
        while not measurement.is_measurement_done():
            pass

        # Download the measurement data
        print("Downloading measurement data")
        raw_data = measurement.get_samples()

        return raw_data
    """
