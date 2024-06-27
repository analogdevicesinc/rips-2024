################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################

import time
from MccHelpersClass import MccHelpersClass
from TM01SystemUnitsSetup import TM01SystemUnitsSetup

from register_helpers import to_register32

import pathlib
MY_SCRIPT_DIR = pathlib.Path(__file__).parent
# This trick is necessary to get the import working from a different folder
# that isn't a subfolder
sys.path.insert(1, str(MY_SCRIPT_DIR / '../../UBL'))
from MemoryMap import TM01

# Grab the address blocks we care about
MCC      = TM01.MCC
IOMATRIX = TM01.IOMATRIX
ADC      = TM01.ADC
CLKCTRL  = TM01.CLKCTRL
SYSCTRL  = TM01.SYSCTRL

class MotorConfigClass:
    
    def __init__(self, interface, TM01Setup, enable_abn_encoder, abn_encoder_resolution, abn_encoder_direction=0 ):
        self.my_interface = interface
        self.TM01Setup = TM01Setup
        self.mcchelp = MccHelpersClass(self.my_interface)

        # Helper - shared with other programs.
        # Setup Basic Clocks for PWM.
        self.PWM_FREQUENCY = 25000  # Decimal - 25kHz
        self.CORE_CLOCK = 120000000  # Decimal - 120MHz
        self.PWM_COUNT_DEC = int(self.CORE_CLOCK/self.PWM_FREQUENCY)-1
        self.enable_encoder = enable_abn_encoder
        self.abn_encoder_resolution = abn_encoder_resolution
        self.abn_encoder_direction = abn_encoder_direction
        self.motor_poles = self.TM01Setup.MotorPoles
        self.VoltageScaling = self.TM01Setup.get_voltage_scaling()
        self.FluxScaling = self.TM01Setup.get_flux_scaling()

        # # Check for FPGA DOCK5 or TMC9660 IC.
        # info_revision = self.mcchelp.mcc_read(MCC.INFO_REVISION.REVISION)
        # if info_revision == 1:
        #     self.board = "tmc_9660_p2"
        # elif info_revision >= 1000:
        #     self.board = "dock5"
        # else: raise NotImplementedError("Uncertain MCC.INFO_REVISION.REVISION value")
        # print("#")
        # print("# Hardware Detected = ", self.board)
        # print("#")

        self.DEFAULT_BBM = 50
        self.PWM_RESOLUTION = 2**16
        self.PWM_PERIOD = 1/self.PWM_FREQUENCY
        self.BBM_PERIOD = 1/self.CORE_CLOCK
        self.ENABLE_BBM_COMPENSATION = False
        self.VM = self.TM01Setup.VM
        self.uduq_v_error = 0.0

    #######################################################################
    ## Make Lots of helper functions - every place there is duplicate code!
    #######################################################################

    def check_hardware(self):
        # Check for FPGA DOCK5 or TMC9660 IC.
        info_revision = self.mcchelp.mcc_read(MCC.INFO_REVISION.REVISION)
        if info_revision == 1:
            self.board = "tmc_9660_p2"
        elif info_revision >= 1000:
            self.board = "dock5"
        else: raise NotImplementedError("Uncertain MCC.INFO_REVISION.REVISION value")
        print("#")
        print("# Hardware Detected = ", self.board)
        print("#")


    def config_motor(self,motor):
        assert motor in [ "Stepper", "PANDrive", "BLDC", "DC"]
        print(f'#### Configuring {motor} Motor. ####')

        if motor == "Stepper":
            self.check_hardware()
            self.config_stepper_motor()
        elif motor == "BLDC":
            self.check_hardware()
            self.config_bldc_motor()
        elif motor == "PANDrive":
            # self.check_hardware() - Not needed for PAN Drive.
            self.config_pandrive_motor()
        else: # motor == "DC"
            self.check_hardware()
            self.config_dc_motor()

    def config_stepper_motor(self):
        # ===== Stepper Motor setup =====
        # Turn off everything before configuration.
        self.motor_off()
        # Zero All Inputs and Outputs.
        self.zero_all_io()
        # Configure Motor Type and PWM for Stepper Motor.
        self.config_stepper_motor_type_pwm()
        # Configure and Calibrate the ADCs for a Stepper Motor.
        self.config_stepper_motor_adc()
        # Configure ABN encoder settings
        self.config_abn_encoder(self.abn_encoder_resolution, self.abn_encoder_direction)
        # Configure Default Limits and PI settings
        self.config_default_limits_pid()
        # ===== ABN encoder test drive =====
        # Init encoder (mode 0)
        self.init_ext_encoder() # Initialize Encoder.
        # Feedback selection
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000003)
        # Switch to voltage_ext mode.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000) # VOLTAGE_EXT Mode
        # Set all Target and Actual Registers to be Measured to 0
        self.zero_all_io()
        time.sleep(3)
    
    def config_bldc_motor(self):
        # ===== BLDC Motor setup =====
        # Turn off everything before configuration.
        self.motor_off()
        # Zero All Inputs and Outputs.
        self.zero_all_io()
        # Motor type & PWM configuration
        self.config_bldc_motor_type_pwm()
        # ADC configuration
        self.config_bldc_motor_adc()
        # Configure ABN encoder settings
        self.config_abn_encoder(self.abn_encoder_resolution, self.abn_encoder_direction)
        # Configure Default Limits and PI settings
        self.config_default_limits_pid()

        '''
        # ABN encoder settings
        self.config_abn_encoder(self.abn_encoder_resolution, self.abn_encoder_direction)

        # Limits
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_LIMITS, 0x03E803E8)
        self.mcchelp.mcc_write(MCC.PID_UQ_UD_LIMITS, 0x7FFF7FFF)

        # PI settings
        self.mcchelp.mcc_write(MCC.PID_TORQUE_COEFF, 0x01000100)
        self.mcchelp.mcc_write(MCC.PID_FLUX_COEFF, 0x01000100)
        '''
        self.init_ext_encoder() # Initialize Encoder.
        # Feedback selection
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000003)

        # Switch to voltage_ext mode.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000) # VOLTAGE_EXT Mode
        # Set all Target and Actual Registers to be Measured to 0
        self.zero_all_io()
        time.sleep(3)

    def config_dc_motor(self):
        print("DC Mode does nothing.")

    def config_pandrive_motor(self):
        print("PANDrive does not need configuration.")
    
########################## Configuration Methods. ##################################################

    def config_default_limits_pid(self):

        # Default Limits: As part of the tuning process, Limits will be derived from Motor Paramaters and Customer Input.
        #
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_LIMITS, 0x03E803E8)
        self.mcchelp.mcc_write(MCC.PID_VELOCITY_LIMIT, 0x03E80000)
        self.mcchelp.mcc_write(MCC.PID_UQ_UD_LIMITS, 0x7FFF7FFF)

        # Default Torque/Flux PI settings.
        self.mcchelp.mcc_write(MCC.PID_TORQUE_COEFF, 0x01000100)
        self.mcchelp.mcc_write(MCC.PID_FLUX_COEFF, 0x01000100)

    def config_bldc_motor_type_pwm(self):
        # Motor type & PWM configuration

        # DOCK5
        if self.board == "dock5":
            print("###############  USING DOCK5 Board ####################")
            gdrv_bbm_l_uvw = 3
            gdrv_bbm_h_uvw = 5
            gdrv_bbm_l_y2  = 3
            gdrv_bbm_h_y2  = 5
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_L_UVW, gdrv_bbm_l_uvw)
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_H_UVW, gdrv_bbm_h_uvw)
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_L_Y2,  gdrv_bbm_l_y2)
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_H_Y2,  gdrv_bbm_h_y2)

            # self.mcchelp.mcc_write(MCC.MOTOR_CONFIG, 0x00030004)
            self.mcchelp.mcc_write(MCC.MOTOR_CONFIG.N_POLE_PAIRS, self.motor_poles)
            self.mcchelp.mcc_write(MCC.MOTOR_CONFIG.TYPE,  3)
        
            # self.mcchelp.mcc_write(MCC.PWM_MAXCNT, 0x000012BF)
            self.mcchelp.mcc_write(MCC.PWM_MAXCNT, self.PWM_COUNT_DEC)          # 32-bits

            # self.mcchelp.mcc_write(MCC.GDRV_TIMING, 0xFFFFFFFF)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SINK_UVW, 0xFF)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SOURCE_UVW, 0xFF)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SINK_Y2, 0xFF)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SOURCE_Y2, 0xFF)

            # self.mcchelp.mcc_write(MCC.GDRV_CFG,   0x00030000)
            self.mcchelp.mcc_write(MCC.GDRV_CFG.ADAPTIVE_MODE_UVW, 1)
            self.mcchelp.mcc_write(MCC.GDRV_CFG.ADAPTIVE_MODE_Y2, 1)
        
            # self.mcchelp.mcc_write(MCC.PWM_CONFIG, 0x00000F07)              
            # wait for bst caps to charge
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.CHOP, 1)
            time.sleep(0.001)
            # done pwm can be used
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.CHOP, 7)                      # 3-bits
            # Bit 3 is not used.                                                # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.SV_MODE, 0x0)                 # 2-bits
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.Y2_HS_SRC, 0x0)               # 2-bits
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_UX1, 1)                # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_VX2, 1)                # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_WY1, 1)                # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_Y2, 1)                 # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.EXT_ENABLE_UX1, 0)            # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.EXT_ENABLE_VX2, 0)            # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.EXT_ENABLE_WY1, 0)            # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.EXT_ENABLE_Y2, 0)             # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.DUTY_CYCLE_OFFSET, 0x0000)    # 16-bits
            self.mcchelp.mcc_write(MCC.PWM_SWITCH_LIMIT, 0x0000CB20)
        # TMC9600
        elif self.board == "tmc_9660_p2":
            print("###############  USING TMC9660 Board ####################")
            self.mcchelp.mcc_write(CLKCTRL.OPT.PWM_CLK_ENA, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.BIAS_EN, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.CHARGEPUMP_EN, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.BST_SW_CP_EN, 1)
            self.mcchelp.mcc_write(SYSCTRL.PMU_CTRL.CP_ENABLE, 1)

            self.mcchelp.mcc_write(MCC.GDRV_CFG.IGATE_SINK_UVW, 15)
            self.mcchelp.mcc_write(MCC.GDRV_CFG.IGATE_SINK_Y2, 15)
            self.mcchelp.mcc_write(MCC.GDRV_CFG.IGATE_SOURCE_UVW, 10)
            self.mcchelp.mcc_write(MCC.GDRV_CFG.IGATE_SOURCE_Y2, 10)

            self.mcchelp.mcc_write(MCC.GDRV_CFG.ADAPTIVE_MODE_UVW, 1)
            self.mcchelp.mcc_write(MCC.GDRV_CFG.ADAPTIVE_MODE_Y2, 1)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SINK_UVW, 0)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SINK_Y2, 0)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SOURCE_UVW, 10)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SOURCE_Y2, 10)
            self.mcchelp.mcc_write(MCC.GDRV_BBM, 0)

            self.mcchelp.mcc_write(MCC.PWM_MAXCNT.PWM_MAXCNT, self.PWM_COUNT_DEC)

            self.mcchelp.mcc_write(MCC.PID_UQ_UD_LIMITS.PID_UQ_UD_LIMITS, 2000)

            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_UX1, 1)
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_VX2, 1)
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_WY1, 1)
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_Y2, 1)

            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_U, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_V, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_W, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_Y2, 1)

            self.mcchelp.mcc_write(MCC.PWM_CONFIG.CHOP, 1) # wait for bst caps to charge
            time.sleep(0.001)
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.CHOP, 7) # done pwm can be used

            print("Configuring Motor")
            self.mcchelp.mcc_write(MCC.MOTOR_CONFIG.TYPE, 3) #BLDC
            self.mcchelp.mcc_write(MCC.MOTOR_CONFIG.N_POLE_PAIRS, self.motor_poles)

        # Board type not found.
        else: raise NotImplementedError("Uncertain MCC.INFO_REVISION.REVISION value")


    def config_stepper_motor_type_pwm(self):
        # Motor type & PWM configuration
                
        # DOCK5
        if self.board == "dock5":
            print("###############  USING DOCK5 Board ####################")
            gdrv_bbm_l_uvw = 3
            gdrv_bbm_h_uvw = 5
            gdrv_bbm_l_y2  = 3
            gdrv_bbm_h_y2  = 5
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_L_UVW, gdrv_bbm_l_uvw)
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_H_UVW, gdrv_bbm_h_uvw)
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_L_Y2,  gdrv_bbm_l_y2)
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_H_Y2,  gdrv_bbm_h_y2)

            # self.mcchelp.mcc_write(MCC.GDRV_HW,   0x0000000F)
            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_U,   1)            # 1-bit
            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_V,   1)            # 1-bit
            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_W,   1)            # 1-bit
            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_Y2,  1)            # 1-bit
        
            # self.mcchelp.mcc_write(MCC.PWM_CONFIG, 0x00000F07)                  
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.SV_MODE, 0x0)                 # 2-bits
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.Y2_HS_SRC, 0x0)               # 2-bits
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_UX1, 1)                # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_VX2, 1)                # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_WY1, 1)                # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_Y2, 1)                 # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.EXT_ENABLE_UX1, 0)            # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.EXT_ENABLE_VX2, 0)            # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.EXT_ENABLE_WY1, 0)            # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.EXT_ENABLE_Y2, 0)             # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.DUTY_CYCLE_OFFSET, 0x0000)    # 16-bits
            # wait for bst caps to charge
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.CHOP, 1)
            time.sleep(0.001)
            # done pwm can be used
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.CHOP, 7)                      # 3-bits
            # Bit 3 is not used.                                                # 1-bit
            self.mcchelp.mcc_write(MCC.PWM_SWITCH_LIMIT, 0x0000CB20)

            self.mcchelp.mcc_write(MCC.PWM_MAXCNT, self.PWM_COUNT_DEC)          # 32-bits
            # self.mcchelp.mcc_write(MCC.MOTOR_CONFIG, 0x00020032)
            self.mcchelp.mcc_write(MCC.MOTOR_CONFIG.N_POLE_PAIRS, self.motor_poles)
            self.mcchelp.mcc_write(MCC.MOTOR_CONFIG.TYPE,  2)

        elif self.board == "tmc_9660_p2":
            # TMC9600
            print("###############  USING TMC9660 Board ####################")
            self.mcchelp.mcc_write(CLKCTRL.OPT.PWM_CLK_ENA, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.BIAS_EN, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.CHARGEPUMP_EN, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.BST_SW_CP_EN, 1)
            
            self.mcchelp.mcc_write(MCC.GDRV_CFG.IGATE_SINK_UVW, 15)
            self.mcchelp.mcc_write(MCC.GDRV_CFG.IGATE_SINK_Y2, 15)
            self.mcchelp.mcc_write(MCC.GDRV_CFG.IGATE_SOURCE_UVW, 10)
            self.mcchelp.mcc_write(MCC.GDRV_CFG.IGATE_SOURCE_Y2, 10)
            
            self.mcchelp.mcc_write(MCC.GDRV_CFG.ADAPTIVE_MODE_UVW, 1)
            self.mcchelp.mcc_write(MCC.GDRV_CFG.ADAPTIVE_MODE_Y2, 1)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SINK_UVW, 0)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SINK_Y2, 0)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SOURCE_UVW, 10)
            self.mcchelp.mcc_write(MCC.GDRV_TIMING.T_DRIVE_SOURCE_Y2, 10)
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_H_UVW, 0)
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_L_UVW, 0)
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_H_Y2,  0)
            self.mcchelp.mcc_write(MCC.GDRV_BBM.BBM_L_Y2,  0)
            
            self.mcchelp.mcc_write(MCC.PWM_MAXCNT, self.PWM_COUNT_DEC)
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_UX1, 1)
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_VX2, 1)
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_WY1, 1)
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.ENABLE_Y2, 1)

            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_U, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_V, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_W, 1)
            self.mcchelp.mcc_write(MCC.GDRV_HW.BRIDGE_ENABLE_Y2, 1)

            self.mcchelp.mcc_write(MCC.PWM_CONFIG.CHOP, 1)
            time.sleep(0.001)
            # done pwm can be used
            self.mcchelp.mcc_write(MCC.PWM_CONFIG.CHOP, 7)                      # 3-bits
            # Bit 3 is not used.                                                # 1-bit

            self.mcchelp.mcc_write(MCC.PWM_MAXCNT, self.PWM_COUNT_DEC)          # 32-bits
            # self.mcchelp.mcc_write(MCC.MOTOR_CONFIG, 0x00020032)
            self.mcchelp.mcc_write(MCC.MOTOR_CONFIG.N_POLE_PAIRS, self.motor_poles)
            self.mcchelp.mcc_write(MCC.MOTOR_CONFIG.TYPE,  2)
        # Board type not found.
        else: raise NotImplementedError("Uncertain MCC.INFO_REVISION.REVISION value")

    def motor_off(self):
        # Turn off everything.
        self.mcchelp.mcc_write(MCC.PWM_CONFIG, 0x0000_0000) # PWM fully off
        self.set_motion_mode(0) # STOP mode

    def config_adc_csa(self):
        print("Enabling ADCs and CSA")
        ### Configure and initialize the ADCs
        if self.board == "dock5":
            print("###############  CONFIGURING ADCs for DOCK5 Board ####################")
            # self.mcchelp.mcc_write(MCC.ADC_I_GEN_CONFIG, 0x000018E4)
            self.mcchelp.mcc_write(MCC.ADC_I_GEN_CONFIG.UX1_SELECT, 0x0)
            self.mcchelp.mcc_write(MCC.ADC_I_GEN_CONFIG.VX2_SELECT, 0x1)
            self.mcchelp.mcc_write(MCC.ADC_I_GEN_CONFIG.WY1_SELECT, 0x2)
            self.mcchelp.mcc_write(MCC.ADC_I_GEN_CONFIG.Y2_SELECT,  0x3)
            self.mcchelp.mcc_write(MCC.ADC_I_GEN_CONFIG.SOURCE_SELECT,  0)
            self.mcchelp.mcc_write(MCC.ADC_I_GEN_CONFIG.MEASUREMENT_MODE, 4)
            self.mcchelp.mcc_write(MCC.ADC_I_GEN_CONFIG.TRIGGER_SELECT,  1)
            self.mcchelp.mcc_write(MCC.ADC_I_GEN_CONFIG.TRIGGER_POS,  0)

        elif self.board == "tmc_9660_p2":
            print("###############  CONFIGURING ADCs for TMC9660 Board ####################")
            self.mcchelp.mcc_write(CLKCTRL.OPT.ADC_CLK_ENA, 1)
 
            self.mcchelp.mcc_write(IOMATRIX.PIN_7_0.ALTF3, 0)
            self.mcchelp.mcc_write(IOMATRIX.PIN_7_0.ALTF3, 0)
            self.mcchelp.mcc_write(IOMATRIX.PIN_7_0.ALTF4, 0)
            self.mcchelp.mcc_write(IOMATRIX.PIN_7_0.ALTF5, 0)
 
            self.mcchelp.mcc_write(ADC.SETUP.NRST_ADC_0, 0x00)
            self.mcchelp.mcc_write(ADC.SETUP.NRST_ADC_1, 0x00)
            self.mcchelp.mcc_write(ADC.SETUP.NRST_ADC_2, 0x00)   
            self.mcchelp.mcc_write(ADC.SETUP.NRST_ADC_3, 0x00)
 
            self.mcchelp.mcc_write(ADC.SETUP.SELECT_ADC, 0x0F)
            self.mcchelp.mcc_write(ADC.SETUP.NRST_ADC_0, 0x01)
            self.mcchelp.mcc_write(ADC.SETUP.NRST_ADC_1, 0x01)
            self.mcchelp.mcc_write(ADC.SETUP.NRST_ADC_2, 0x01)
            self.mcchelp.mcc_write(ADC.SETUP.NRST_ADC_3, 0x01)
 
            self.mcchelp.mcc_write(ADC.SETUP.MMU_ENABLE, 0x01)
            self.mcchelp.mcc_write(ADC.RW_ADDR_DATA, 0xAA02BF01)
            self.mcchelp.mcc_write(ADC.RW_ADDR_DATA, 0xAA021104)
            self.mcchelp.mcc_write(ADC.RW_ADDR_DATA, 0xAA025705)
            self.mcchelp.mcc_write(ADC.RW_ADDR_DATA, 0xAA020506)
            self.mcchelp.mcc_write(ADC.RW_ADDR_DATA, 0xAA02200B)
            self.mcchelp.mcc_write(ADC.RW_ADDR_DATA, 0x55000000)
            self.mcchelp.mcc_write(ADC.CSA_SETUP.CSA0_EN, 1)
            self.mcchelp.mcc_write(ADC.CSA_SETUP.CSA1_EN, 1)
            self.mcchelp.mcc_write(ADC.CSA_SETUP.CSA2_EN, 1)
            self.mcchelp.mcc_write(ADC.CSA_SETUP.CSA3_EN, 1)
            self.mcchelp.mcc_write(ADC.CSA_SETUP.CSA012_GAIN, 1) # Set ADC Gain to 10X
            self.mcchelp.mcc_write(ADC.CSA_SETUP.CSA3_GAIN, 1)   # Set ADC Gain to 10X

            self.mcchelp.mcc_write(MCC.ADC_I_GEN_CONFIG.MEASUREMENT_MODE, 4)
        else: raise NotImplementedError("Uncertain MCC.INFO_REVISION.REVISION value")

    def config_stepper_motor_adc(self):

        # ADC configuration    
        self.config_adc_csa()

        # Ensure Motor is Not Turning.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000028) # VOLTAGE_EXT Mode
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0x0000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0x0000)

        print("#### Checking ADC Scale Sign - Depends on Power Stage Configuration.")

        # Set the Default Scale Configuration.
        self.mcchelp.mcc_write(MCC.ADC_I0_CONFIG, 0xFC000000)
        self.mcchelp.mcc_write(MCC.ADC_I1_CONFIG, 0x04000000)
        self.mcchelp.mcc_write(MCC.ADC_I2_CONFIG, 0xFC000000)
        self.mcchelp.mcc_write(MCC.ADC_I3_CONFIG, 0x04000000)
        
        # Write a 1000 code level to UD.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000028) # VOLTAGE_EXT Mode
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0x0000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0x03E8)
        
        # Wait for UD input to settle.
        time.sleep(1)

        # Capture FLUX Samples
        id_avg = 0
        NumSamples = 1024
        for i in range(NumSamples):
            id_avg = id_avg + self.mcchelp.mcc_read(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL)
        id_avg = int(id_avg/NumSamples)
        
        # Write 0 to UD
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000028) # VOLTAGE_EXT Mode
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0x0000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0x0000)

        # Wait for UD input to settle.
        time.sleep(1)

        # Update ADC Scale Sign based on polarity of UD / FLUX level - UD and FLUX should have the same sign.
        if id_avg >= 0:
            gain_sign = +1
            # Fix sign before offset check.
            self.mcchelp.mcc_write(MCC.ADC_I0_CONFIG, 0xFC000000)
            self.mcchelp.mcc_write(MCC.ADC_I1_CONFIG, 0x04000000)
            self.mcchelp.mcc_write(MCC.ADC_I2_CONFIG, 0xFC000000)
            self.mcchelp.mcc_write(MCC.ADC_I3_CONFIG, 0x04000000)
        else:
            gain_sign = -1
            # Fix sign before offset check.
            self.mcchelp.mcc_write(MCC.ADC_I0_CONFIG, 0x04000000)
            self.mcchelp.mcc_write(MCC.ADC_I1_CONFIG, 0xFC000000)
            self.mcchelp.mcc_write(MCC.ADC_I2_CONFIG, 0x04000000)
            self.mcchelp.mcc_write(MCC.ADC_I3_CONFIG, 0xFC000000)

        print("#### Calibrating ADC Offsets.")
        NumSamples = 1024
        adc_i0_avg = 0
        adc_i1_avg = 0
        adc_i2_avg = 0
        adc_i3_avg = 0
        for i in range(NumSamples):
            adc_i0_avg = adc_i0_avg + self.mcchelp.mcc_read(MCC.ADC_I1_I0_RAW.I0)
            adc_i1_avg = adc_i1_avg + self.mcchelp.mcc_read(MCC.ADC_I1_I0_RAW.I1)
            adc_i2_avg = adc_i2_avg + self.mcchelp.mcc_read(MCC.ADC_I3_I2_RAW.I2)
            adc_i3_avg = adc_i3_avg + self.mcchelp.mcc_read(MCC.ADC_I3_I2_RAW.I3)
        adc_i0_avg_correct = int(-1*adc_i0_avg/NumSamples)
        adc_i1_avg_correct = int(-1*adc_i1_avg/NumSamples)
        adc_i2_avg_correct = int(-1*adc_i2_avg/NumSamples)
        adc_i3_avg_correct = int(-1*adc_i3_avg/NumSamples)

        # Assemble ADC Scale and Offset values for register writes.
        adc0_config = to_register32(-1024*gain_sign, adc_i0_avg_correct)
        adc1_config = to_register32( 1024*gain_sign, adc_i1_avg_correct)
        adc2_config = to_register32(-1024*gain_sign, adc_i2_avg_correct)
        adc3_config = to_register32( 1024*gain_sign, adc_i3_avg_correct)
        
        # Write ADC Config Register values.
        self.mcchelp.mcc_write(MCC.ADC_I0_CONFIG, adc0_config)
        self.mcchelp.mcc_write(MCC.ADC_I1_CONFIG, adc1_config)
        self.mcchelp.mcc_write(MCC.ADC_I2_CONFIG, adc2_config)
        self.mcchelp.mcc_write(MCC.ADC_I3_CONFIG, adc3_config)

        # Read and print ADC Config Register values.
        print("ADC0 Config = ", '0x{0:08X}'.format(self.mcchelp.mcc_read(MCC.ADC_I0_CONFIG)))
        print("ADC1 Config = ", '0x{0:08X}'.format(self.mcchelp.mcc_read(MCC.ADC_I1_CONFIG)))
        print("ADC2 Config = ", '0x{0:08X}'.format(self.mcchelp.mcc_read(MCC.ADC_I2_CONFIG)))
        print("ADC3 Config = ", '0x{0:08X}'.format(self.mcchelp.mcc_read(MCC.ADC_I3_CONFIG)))

        print("#### ADC Stepper Offset Calibration Complete.")

    def config_bldc_motor_adc(self):
        # ADC configuration
        self.config_adc_csa()
        
        print("#### Checking ADC Scale Sign - Depends on Power Stage Configuration.")

        # Set the Default Scale Configuration.
        self.mcchelp.mcc_write(MCC.ADC_I0_CONFIG, 0x04000000)
        self.mcchelp.mcc_write(MCC.ADC_I1_CONFIG, 0x04000000)
        self.mcchelp.mcc_write(MCC.ADC_I2_CONFIG, 0x04000000)
        self.mcchelp.mcc_write(MCC.ADC_I3_CONFIG, 0x04000000)
        
        # Write a 1000 code level to UD.
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000028) # VOLTAGE_EXT Mode
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0x0000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0x03E8)
        
        # Wait for UD input to settle.
        time.sleep(1)

        # Capture FLUX Samples
        id_avg = 0
        NumSamples = 1024
        for i in range(NumSamples):
            id_avg = id_avg + self.mcchelp.mcc_read(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL)
        id_avg = int(id_avg/NumSamples)
        
        # Write 0 to UD
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000028) # VOLTAGE_EXT Mode
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0x0000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0x0000)

        # Wait for UD input to settle.
        time.sleep(1)

        # Update ADC Scale Sign based on polarity of UD / FLUX level - UD and FLUX should have the same sign.
        if id_avg >= 0:
            gain_sign = +1
            # Fix sign before offset check.
            self.mcchelp.mcc_write(MCC.ADC_I0_CONFIG, 0x04000000)
            self.mcchelp.mcc_write(MCC.ADC_I1_CONFIG, 0x04000000)
            self.mcchelp.mcc_write(MCC.ADC_I2_CONFIG, 0x04000000)
            self.mcchelp.mcc_write(MCC.ADC_I3_CONFIG, 0x04000000)
        else:
            gain_sign = -1
            # Fix sign before offset check.
            self.mcchelp.mcc_write(MCC.ADC_I0_CONFIG, 0xFC000000)
            self.mcchelp.mcc_write(MCC.ADC_I1_CONFIG, 0xFC000000)
            self.mcchelp.mcc_write(MCC.ADC_I2_CONFIG, 0xFC000000)
            self.mcchelp.mcc_write(MCC.ADC_I3_CONFIG, 0xFC000000)

        print("#### Calibrating ADC Offsets.")
        NumSamples = 1024
        adc_i0_avg = 0
        adc_i1_avg = 0
        adc_i2_avg = 0
        adc_i3_avg = 0
        for i in range(NumSamples):
            adc_i0_avg = adc_i0_avg + self.mcchelp.mcc_read(MCC.ADC_I1_I0_RAW.I0)
            adc_i1_avg = adc_i1_avg + self.mcchelp.mcc_read(MCC.ADC_I1_I0_RAW.I1)
            adc_i2_avg = adc_i2_avg + self.mcchelp.mcc_read(MCC.ADC_I3_I2_RAW.I2)
            adc_i3_avg = adc_i3_avg + self.mcchelp.mcc_read(MCC.ADC_I3_I2_RAW.I3)
        adc_i0_avg_correct = int(-1*adc_i0_avg/NumSamples)
        adc_i1_avg_correct = int(-1*adc_i1_avg/NumSamples)
        adc_i2_avg_correct = int(-1*adc_i2_avg/NumSamples)
        adc_i3_avg_correct = int(-1*adc_i3_avg/NumSamples)

        # Assemble ADC Scale and Offset values for register writes.
        adc0_config = to_register32( 1024*gain_sign, adc_i0_avg_correct)
        adc1_config = to_register32( 1024*gain_sign, adc_i1_avg_correct)
        adc2_config = to_register32( 1024*gain_sign, adc_i2_avg_correct)
        adc3_config = to_register32( 1024*gain_sign, adc_i3_avg_correct)
        
        # Write ADC Config Register values.
        self.mcchelp.mcc_write(MCC.ADC_I0_CONFIG, adc0_config)
        self.mcchelp.mcc_write(MCC.ADC_I1_CONFIG, adc1_config)
        self.mcchelp.mcc_write(MCC.ADC_I2_CONFIG, adc2_config)
        self.mcchelp.mcc_write(MCC.ADC_I3_CONFIG, adc3_config)

        # Read and print ADC Config Register values.
        print("ADC0 Config = ", '0x{0:08X}'.format(self.mcchelp.mcc_read(MCC.ADC_I0_CONFIG)))
        print("ADC1 Config = ", '0x{0:08X}'.format(self.mcchelp.mcc_read(MCC.ADC_I1_CONFIG)))
        print("ADC2 Config = ", '0x{0:08X}'.format(self.mcchelp.mcc_read(MCC.ADC_I2_CONFIG)))
        print("ADC3 Config = ", '0x{0:08X}'.format(self.mcchelp.mcc_read(MCC.ADC_I3_CONFIG)))

        print("#### ADC BLDC Offset Calibration Complete.")

    def config_abn_encoder(self, resolution, direction):
        print("### ABN Encoder Configuration - NOTE that Encoder Resolution, not Step Count should be provided. Generally, Step Count is 4 times the Resolution value.")
        # ABN encoder settings
        self.mcchelp.mcc_write(MCC.ABN_MODE, 0x00000000)
        # Calculate the inverse of the steps value within the register range.
        steps = int(4*resolution)
        register_scale = 2**32
        steps_inv = int(register_scale/steps)
        self.mcchelp.mcc_write(MCC.ABN_CPR, steps)
        self.mcchelp.mcc_write(MCC.ABN_CPR_INV, steps_inv)
        self.mcchelp.mcc_write(MCC.ABN_MODE.DIRECTION, direction)
        print("#### ABN Encoder CPR = ",self.mcchelp.mcc_read(MCC.ABN_CPR))
        print("#### ABN Encoder Inverse CPR = ",self.mcchelp.mcc_read(MCC.ABN_CPR_INV))
        self.mcchelp.mcc_write(MCC.VELOCITY_CONFIG.METER_TYPE, 0)
        #### End of config_abn_encoder

    def init_abn_encoder(self, voltage_codes=1000): # Add Optional Parameters for Voltage Magnitude and Delay
        self.init_ext_encoder(voltage_codes)
        print("### Switch to ABN Encoder.")
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000003)
        print("### ABN Encoder selected for PHI_E.")

    def init_ext_encoder(self,voltage_codes=1000): # Add Optional Parameters for Voltage Magnitude and Delay
        print("### Initialize ABN Encoder.")
        # ABN encoder configuration (Init encoder (mode 0))
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000038)
        self.mcchelp.mcc_write(MCC.ABN_PHI_E_OFFSET, 0x00000000)
        self.mcchelp.mcc_write(MCC.PHI_E_SELECTION, 0x00000001)
        self.mcchelp.mcc_write(MCC.PHI_EXT, 0x00000000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT, to_register32(0,voltage_codes)) # NOTE: potentially can use a higher voltoage and monitor current.
        time.sleep(2)
        self.mcchelp.mcc_write(MCC.ABN_COUNT, 0x00000000)
        time.sleep(1)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT, 0x00000000)
        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000)
        abn_count = self.mcchelp.mcc_read(MCC.ABN_COUNT)
        print("### ABN Count Initialized - ABN_COUNT = ", abn_count)

    def set_allowed_voltageext_level(self, max_current, target_current_percentage, ud_inc_volts=0.1, max_ud_codes=1000):
        print("### Detect UD Level required for a given Motor Target Flux Current.")

        if target_current_percentage > 100:
            target_current = max_current*0.75
            print("# Target Current Percentage is too large, setting Target Current to 75% of Max Current = ",target_current,"A.")
        else:
            target_current = max_current*target_current_percentage/100
            print("# Setting Target Current = ",target_current,"A.")

        target_flux_actual_codes = target_current/self.FluxScaling
        max_current_codes = int(max_current/self.FluxScaling)

        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000028) # VOLTAGE_EXT Mode
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0x0000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0x0000)
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0x0000)
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL, 0x0000)

        ud_level_codes = 0
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, ud_level_codes)
        measured_current_codes =  0
        target_voltageext_codes = 0
        ud_inc_codes = int(ud_inc_volts/self.VoltageScaling)

        print("# Starting UD Ramp, Increment = ", ud_inc_volts, "V or ",ud_inc_codes, "codes.")

        while measured_current_codes <= target_flux_actual_codes:
            if ud_level_codes > max_ud_codes:
                print("# WARNING: Max UD Codes Exceeded. Stopping Test and setting UD to Max Level. UD = ", ud_level_codes,"Codes.")
                target_voltageext_codes = max_ud_codes
                break
            else:
                self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, ud_level_codes)
                measured_current_codes = self.measure_flux_current_codes(samples=10)
                if measured_current_codes >= target_flux_actual_codes:
                    target_voltageext_codes = ud_level_codes
                    print("# UD Level Identified. UD = ", target_voltageext_codes,"Codes.")
                    break
                else:
                    ud_level_codes = ud_level_codes + ud_inc_codes

        self.mcchelp.mcc_write(MCC.MOTION_CONFIG, 0x00000000) # Stopped Mode.
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0x0000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0x0000)
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0x0000)
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL, 0x0000)

        target_voltageext = self.TM01Setup.get_voltage_value(target_voltageext_codes)
        target_measured_current = measured_current_codes*self.FluxScaling
        estimated_resistance = target_voltageext / target_measured_current

        print("###### Estimated Target Voltage, Current and Resistance ######")
        print("# Target UD   = ", target_voltageext,"V.")
        print("# Target Flux = ", target_measured_current,"A.")
        print("# Estimated Resistance = ",estimated_resistance,"Ohms.")
        print("######")

        return target_voltageext_codes, max_current_codes

    def measure_flux_current_codes(self, samples):
        id_avg = 0
        for i in range(samples):
            id_avg = id_avg + self.mcchelp.mcc_read(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL)
        id_avg = int(id_avg/samples)
        return id_avg

    def zero_voltageext(self):
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UQ, 0x0000)
        self.mcchelp.mcc_write(MCC.VOLTAGE_EXT.UD, 0x0000)
        #### End of zero_voltageext

    def zero_torquefluxtarget(self):
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET.PID_TORQUE_TARGET, 0x0000)
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_TARGET.PID_FLUX_TARGET, 0x0000)

    def zero_velocitytarget(self):
        self.mcchelp.mcc_write(MCC.PID_VELOCITY_TARGET, 0x0000_0000)

    def zero_positiontarget(self):
        self.mcchelp.mcc_write(MCC.PID_POSITION_TARGET, 0x0000_0000)

    def zero_torquefluxactual(self):
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL, 0x0000)
        self.mcchelp.mcc_write(MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL, 0x0000)

    def zero_velocityactual(self):
        self.mcchelp.mcc_write(MCC.PID_VELOCITY_ACTUAL, 0x0000_0000)

    def zero_positionactual(self):
        self.mcchelp.mcc_write(MCC.PID_POSITION_ACTUAL, 0x0000_0000)

    def zero_all_io(self):
        self.zero_voltageext()
        self.zero_torquefluxtarget
        self.zero_velocitytarget
        self.zero_positiontarget
        self.zero_torquefluxactual
        self.zero_velocityactual
        self.zero_positionactual

    def set_motion_mode(self,mode):
        if mode in range(0, 10):
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG.MOTION_MODE, mode) # Set Motion Mode
            print("#### Motion Mode set to: ", self.check_motion_mode(mode))
        else:
            print("#### Motion Mode Not Valid!")
            print("#### Allowed Values are: 0:OFF, 1:TORQUE, 2:VELOCITY, 3:POSITION, 4:PRBS_FLUX, 5:PRBS_TORQUE, 6:PRBS_VELOCITY, 7:PRBS_POSITION, 8:VOLTAGE_EXT, 9:PRBS_UD, 10:PWM_EXT")
            exit(1)

    def set_ramp_mode(self,mode):
        if mode in range(0, 1):
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG.RAMP_MODE, mode) # Set Ramp Mode
            print("#### Ramp Mode set to: ", self.check_ramp_mode(mode))
        else:
            print("#### Ramp Mode Not Valid!")
            print("#### Allowed Values are: 0:POSITION, 1:VELOCITY")
            exit(1)

    def set_ramp_enable(self,enable):
        if enable:
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG.RAMP_ENABLE, 1) # Enable Ramp Mode
            print("#### Ramp Mode Enabled.")
        else:
            self.mcchelp.mcc_write(MCC.MOTION_CONFIG.RAMP_ENABLE, 0) # Disable Ramp Mode
            print("#### Ramp Mode Disabled.")
 
    def check_ramp_mode(self, mode):
        # Check to see if there is an object oriented way to do the mode lookups!
        if mode == 0:
            ramp_mode = "POSITION"
        elif mode == 1:
            ramp_mode = "VELOCITY"
        else:
            ramp_mode = "Not Found!"
        return ramp_mode

    def check_motion_mode(self, mode):
        # Check to see if there is an object oriented way to do the mode lookups!
        if mode == 0:
            motion_mode = "OFF"
        elif mode == 1:
            motion_mode = "TORQUE"
        elif mode == 2:
            motion_mode = "VELOCITY"
        elif mode == 3:
            motion_mode = "POSITION"
        elif mode == 4:
            motion_mode = "PRBS_FLUX"
        elif mode == 5:
            motion_mode = "PRBS_TORQUE"
        elif mode == 6:
            motion_mode = "PRBS_VELOCITY"
        elif mode == 7:
            motion_mode = "PRBS_POSITION"
        elif mode == 8:
            motion_mode = "VOLTAGE_EXT"
        elif mode == 9:
            motion_mode = "PRBS_UD"
        elif mode == 10:
            motion_mode = "PWM_EXT"
        else:
            motion_mode = "Not Found!"
        return motion_mode




