################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import time
import csv
import os
from collections import OrderedDict
from datetime import datetime
from tkinter import N
from MccHelpersClass import MccHelpersClass
from TM01SystemUnitsSetup import TM01SystemUnitsSetup

from register_helpers import to_register32
from register_helpers import to_signed_32
from register_helpers import to_signed_16

import pathlib
import sys
MY_SCRIPT_DIR = pathlib.Path(__file__).parent
# This trick is necessary to get the import working from a different folder
# that isn't a subfolder
sys.path.insert(1, str(MY_SCRIPT_DIR / '../../UBL'))
from MemoryMap import TM01

# Grab the address blocks we care about
MCC      = TM01.MCC


class UQProfileClass:
    
    def __init__(self, interface, TM01Setup, poles, Rest, Lest, file_path):
        self.TM01Setup = TM01Setup

        self.filename_prefix = file_path
        now = datetime.now() # current date and time
        date_time = now.strftime("%Y%m%d_%H%M%S")
        self.csvfilename = os.path.splitext(self.filename_prefix)[0]+'_'+date_time+'.csv'

        self.my_interface = interface
        self.mcchelp = MccHelpersClass(self.my_interface)
        self.NumPoles = poles
        self.Rest = Rest
        self.Lest = Lest
        self.MinUQ =  -1350
        self.MaxUQ =  1350
        self.UQStep = 300

        self.SF_Search_UQ_Start = 0
        self.SF_Search_UQ_End   = 250
        self.SF_Search_UQ_Step  = 5

        # self.voltage_scaling = 0.000732421875 # Volts/code.
        # self.torque_flux_scaling = 0.0003814697265625 # Amps/code.
        # self.velocity_scaling = 4.5716189973850531342955386244061e-6 # rads/sec per code.



    def run_uq_profile_torque(self, target_torque_codes):
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
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000001)     # Switch to Torque Modet Mode, No Ramp
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL, 0)

        self.data = OrderedDict()
        self.data["uq"] = []
        self.data["torque"] = []
        self.data["flux"] = []        
        self.data["velocity"] = []
        self.data["kest"] = []
        self.headers = ["uq", "torque", "flux", "velocity", "kest"]

        loop_num = 0
        kest_sum = 0
        
        self.mcchelp.mcc_write(MCC.FOC_UQ_UD_LIMITED.UQ, 0)
        self.mcchelp.mcc_write(MCC.FOC_UQ_UD_LIMITED.UD, 0)
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL, 0)
        self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL, 0)
        time.sleep(1)

        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET.PID_FLUX_TARGET,                     0 ) # Write to Flux Target Register.
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET.PID_TORQUE_TARGET, target_torque_codes ) # Write to Torque Target Register.
        print("# Target Torque Current = ",target_torque_codes, "codes,", self.TM01Setup.get_SI_value('MCC.PID_TORQUE_FLUX_TARGET.PID_TORQUE_TARGET', target_torque_codes), "Amps")
        time.sleep(10) # Wait for loop to settle.
        uq_data, torque_data, flux_data, velocity_data = self.average_data()
        if velocity_data == 0.0:
            print("Skipping itteration due to 0 Velocity data.")
            K_est = 0.0
        else:
            kest_data =  ((uq_data-(self.Rest*torque_data))/velocity_data) - (self.NumPoles*self.Lest*flux_data)
            print("# Kest = ", kest_data)
            K_est = kest_data
            print("#")

        
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0) # Write to UQ Register.
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET.PID_FLUX_TARGET,   0 ) # Write to Flux Target Register.
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET.PID_TORQUE_TARGET, 0 ) # Write to Torque Target Register.
        
        return K_est






    def run_uq_profile_single(self, target_uq):
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
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000038)     # Switch to Voltage_Ext Mode, No Ramp
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0 ) # Write to UD Register.
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0) # Write to UQ Register.

        self.data = OrderedDict()
        self.data["uq"] = []
        self.data["torque"] = []
        self.data["flux"] = []        
        self.data["velocity"] = []
        self.data["kest"] = []
        self.headers = ["uq", "torque", "flux", "velocity", "kest"]

        loop_num = 0
        kest_sum = 0
        
        self.mcchelp.mcc_write(MCC.FOC_UQ_UD_LIMITED.UQ, 0)
        self.mcchelp.mcc_write(MCC.FOC_UQ_UD_LIMITED.UD, 0)
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL, 0)
        self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL, 0)
        time.sleep(1)

        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0 ) # Write to UD Register.
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, target_uq) # Write to UQ Register.
        print("# Target Voltage = ",target_uq, "codes,", self.TM01Setup.get_SI_value('MCC.VOLTAGE_EXT.UQ', target_uq), "Volts")
        time.sleep(10) # Wait for loop to settle.
        uq_data, torque_data, flux_data, velocity_data = self.average_data()
        if velocity_data == 0.0:
            print("Skipping itteration due to 0 Velocity data.")
        else:
            kest_data =  ((uq_data-(self.Rest*torque_data))/velocity_data) - (self.NumPoles*self.Lest*flux_data)
            print("# Kest = ", kest_data)
            print("#")

        K_est = kest_data
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0) # Write to UQ Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)
        
        return K_est



    def run_uq_profile(self):
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
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000038)     # Switch to Voltage_Ext Mode, No Ramp
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0 ) # Write to UD Register.
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0) # Write to UQ Register.

        self.data = OrderedDict()
        self.data["uq"] = []
        self.data["torque"] = []
        self.data["flux"] = []        
        self.data["velocity"] = []
        self.data["kest"] = []
        self.headers = ["uq", "torque", "flux", "velocity", "kest"]

        loop_num = 0
        kest_sum = 0
        for target_uq in range( self.MinUQ, self.MaxUQ+1, self.UQStep ):
            self.mcchelp.mcc_write(MCC.FOC_UQ_UD_LIMITED.UQ, 0)
            self.mcchelp.mcc_write(MCC.FOC_UQ_UD_LIMITED.UD, 0)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL, 0)
            time.sleep(1)
            self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0 ) # Write to UD Register.
            self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, target_uq) # Write to UQ Register.
            print("# Itteration: ", loop_num)
            # print("# Target Voltage = ",target_uq*self.voltage_scaling)
            print("# Target Voltage = ",target_uq, "codes,", self.TM01Setup.get_SI_value('MCC.VOLTAGE_EXT.UQ', target_uq), "Volts")
            time.sleep(10) # Wait for loop to settle.
            uq_data, torque_data, flux_data, velocity_data = self.average_data()
            if velocity_data == 0.0:
                print("Skipping itteration due to 0 Velocity data.")
            else:
                kest_data =  ((uq_data-(self.Rest*torque_data))/velocity_data) - (self.NumPoles*self.Lest*flux_data)
                self.data["uq"].append(uq_data)
                self.data["torque"].append(torque_data)
                self.data["flux"].append(flux_data)
                self.data["velocity"].append(velocity_data)
                self.data["kest"].append(kest_data)
                loop_num = loop_num +1
                kest_sum = kest_sum + kest_data
                print("# Kest = ", kest_data)
                print("#")

        K_est = kest_sum/loop_num
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0) # Write to UQ Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)

        data_len = len(self.data[list(self.data)[0]])

        # print("Data = ", self.data)

        with open(self.csvfilename, 'w', newline='') as f:
            writer = csv.writer(f, delimiter=";")
            writer.writerow(self.headers)
            for i in range(data_len):
                writer.writerow([self.data[x][i] for x in self.headers])
        print("Written file", self.csvfilename)

        return K_est

    def average_data(self):
        voltage_average = 0
        torque_average = 0
        flux_average = 0
        velocity_average = 0
        NumSamples = 512
        for i in range(NumSamples):
            voltage_average = voltage_average + self.mcchelp.mcc_read(MCC.FOC_UQ_UD_LIMITED.UQ)
            flux_average = flux_average + self.mcchelp.mcc_read(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL)
            torque_average = torque_average + self.mcchelp.mcc_read(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL)
            velocity_average = velocity_average + self.mcchelp.mcc_read(MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL)
        voltage_average = voltage_average/NumSamples
        flux_average = flux_average/NumSamples
        torque_average = torque_average/NumSamples
        velocity_average = velocity_average/NumSamples

        voltage_scaled = self.TM01Setup.get_SI_value('MCC.VOLTAGE_EXT.UQ', voltage_average)
        flux_scaled = self.TM01Setup.get_SI_value('MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL', flux_average)
        torque_scaled = self.TM01Setup.get_SI_value('MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL', torque_average)
        velocity_scaled = self.TM01Setup.get_SI_value('MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL', velocity_average)

        print("# Average Voltage = ", voltage_average, "codes,", voltage_scaled," Volts.")
        print("# Average Torque   = ", torque_average, "codes,", torque_scaled," Amps.")
        print("# Average Flux     = ", flux_average, "codes,", flux_scaled," Amps.")
        print("# Average Velocity = ", velocity_average, "codes,", velocity_scaled," Rads/s.")

        return voltage_scaled, torque_scaled, flux_scaled, velocity_scaled


    def find_static_friction(self):
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
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000038)     # Switch to Voltage_Ext Mode, No Ramp
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0 ) # Write to UD Register.
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0) # Write to UQ Register.

        self.data = OrderedDict()
        self.data["uq"] = []
        self.data["torque"] = []
        self.data["flux"] = []        
        self.data["velocity"] = []
        self.headers = ["uq", "torque", "flux", "velocity"]

        loop_num = 0

        for target_uq in range( self.SF_Search_UQ_Start, self.SF_Search_UQ_End+1, self.SF_Search_UQ_Step ):
            self.mcchelp.mcc_write(MCC.FOC_UQ_UD_LIMITED.UQ, 0)
            self.mcchelp.mcc_write(MCC.FOC_UQ_UD_LIMITED.UD, 0)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL, 0)
            self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL, 0)
            time.sleep(1)
            self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0 ) # Write to UD Register.
            self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, target_uq) # Write to UQ Register.
            print("# Itteration: ", loop_num)
            # print("# Target Voltage = ",target_uq*self.voltage_scaling)
            print("# Target Voltage = ", target_uq, "codes,", self.TM01Setup.get_SI_value('MCC.VOLTAGE_EXT.UQ', target_uq), "Volts.")
            time.sleep(0.1) # Wait for loop to settle.
            uq_data, torque_data, flux_data, velocity_data = self.average_data()
            self.data["uq"].append(uq_data)
            self.data["torque"].append(torque_data)
            self.data["flux"].append(flux_data)
            self.data["velocity"].append(velocity_data)
            loop_num = loop_num +1
            print("#")

        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0) # Write to UQ Register.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)

        data_len = len(self.data[list(self.data)[0]])

        # print("Data = ", self.data)

        with open(self.csvfilename, 'w', newline='') as f:
            writer = csv.writer(f, delimiter=";")
            writer.writerow(self.headers)
            for i in range(data_len):
                writer.writerow([self.data[x][i] for x in self.headers])
        print("Written file", self.csvfilename)

