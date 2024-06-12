################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import time
import math

class TM01SystemUnitsSetup:
    
    def __init__(self, # *, Forces all arguements to be named - check code and update.
                SystemPWMFrequency = 25000,     # System PWM Frequency.
                UserVM=24,                      # Max System Power Supply Voltage.
                UserRshunt=0.005,               # H-Bridge Shunt Resistor Value - Ohms
                UserShuntOpAmpGain = 20,        # V/V ( V1.2 Boards = 20, V1.1 Board = 10)
                UserADCVMax = 5.0,              # Volts
                UserMotorType = "Stepper",      # 
                UserMotorR=5.0,                 # Nominal Motor Winding Resistance.
                UserMotorL=0.001,               # Nominal Motor Winding Inductance.
                UserMotorPoles=50,              # Number of Poses in Motor.
                UserMotorVmax=5,                # Maximum Allowed Motor Voltage.
                UserMotorImax=1.0,              # Maximum Allowed Motor Current.
                ):

        self.MotorType = UserMotorType              # Motor Type - "Stepper", "BLDC", "DC".
        self.PWM_FREQUENCY = SystemPWMFrequency     # System PWM Frequency.
        self.PWM_PERIOD = 1/self.PWM_FREQUENCY      # PWM Period.
        self.PWM_HPERIOD = self.PWM_PERIOD/2        # PWM Half Period.
        self.BBM_TIME = 0                           # Total BBM time.
        self.VM = UserVM                            # Max System Power Supply Voltage.
        self.Rshunt = UserRshunt                    # H-Bridge Shunt Resistor Value - Ohms
        self.MotorR = UserMotorR                    # Nominal Motor Winding Resistance.
        self.MotorL = UserMotorL                    # Nominal Motor Winding Inductance.
        self.MotorPoles = UserMotorPoles            # Number of Poses in Motor.
        self.MotorVMax = UserMotorVmax              # Maximum Allowed Motor Voltage.
        self.MotorIMax = UserMotorImax              # Maximum Allowed Motor Current.
        self.ShuntOpAmpGain = UserShuntOpAmpGain    # V/V ( V1.2 Boards = 20, V1.1 Board = 10)
        self.ADCVMax = UserADCVMax                  # Volts

        # TM01 Datapath Bit Widths - MCC Datapth scaling.
        self.CurrentADCBits = 16    # bits
        self.CalculationWdith = 32  # bits
        self.PWMDutyCycleBits = 16  # bits
        self.VoltageBits = 15 # bits
        self.CurrentScalingFactorShift = 10 # Shift value of the current scaling calculation

        # Calculated Voltage Scaling Values.
        self.VoltageDutyCycleSteps = self.VM/(2**self.PWMDutyCycleBits) # Voltage step per PWM duty cycle increment. Note that since the motor sees a voltage delta across the bridges, we can have a voltage range of -VM;+VM represented by a PWM duty cycle delta of -65535;+65535
        self.VoltageScalingFactor_BLDC    = self.VoltageDutyCycleSteps*(2**(self.PWMDutyCycleBits-(self.VoltageBits - 0)) ) # Uncompensated Voltage step per internal voltage unit - BLDC
        self.VoltageScalingFactor_Stepper = self.VoltageDutyCycleSteps*(2**(self.PWMDutyCycleBits-(self.VoltageBits - 1)) ) # Uncompensated Voltage step per internal voltage unit - Stepper

        if self.MotorType == "Stepper":
            self.VoltageScalingFactor = self.VoltageScalingFactor_Stepper
        elif self.MotorType == "BLDC":
            self.VoltageScalingFactor = self.VoltageScalingFactor_BLDC
        elif self.MotorType == "PANDrive":
            print("Warning: No PANDrive Settings yet.")
            self.VoltageScalingFactor = self.VoltageScalingFactor_Stepper
        elif self.MotorType == "DC":
            print("ERROR: DC Motor Type not supported.")
            exit(1)
        else:
            print("ERROR: Incorrect Motor Type Specified.")
            exit(1)

        # Calculated Current Scaling Values.
        # NOTE: In the TM01 Units spreadsheet (RL Measurement tab), Current scaling factor (mA) is additionally multiplied by 2**self.CurrentScalingFactorShift, which is removed here to give a true Amps/code scaling factor.
        self.CurrentSenseVoltagePerAmp = (self.Rshunt)*self.ShuntOpAmpGain # Voltage at the current sense ADC (after applying gain) representing the motor current
        self.CurrentScalingFactor = self.ADCVMax/(self.CurrentSenseVoltagePerAmp*2**self.CurrentADCBits) # Factor value of the current scaling calculation (for Amp amplitudes) 
        # NOTE: In the TM01 Units spreadsheet (RL Measurement tab), Current scaling factor (mA) is additionally multiplied by 2**self.CurrentScalingFactorShift, which is removed here to give a true Amps/code scaling factor.
       
        # Values used in Other Functions.
        self.VoltageScaling = self.VoltageScalingFactor # 0.000732422
        self.FluxScaling = self.CurrentScalingFactor   #*1024/1000 # 0.00078125
        self.FluxScalingRMS = self.CurrentScalingFactor/math.sqrt(2) # 0.0005524271728
        self.FluxScalingShift = self.CurrentScalingFactorShift # 10

        # Calculated Limit Values.
        self.MaxUDUQ = int(self.MotorVMax/self.VoltageScaling)
        self.MaxTorqueFluxPeak = int(self.MotorIMax/self.FluxScaling)
        self.MaxTorqueFluxRMS = int(self.MotorIMax/self.FluxScalingRMS)
        self.MaxPWMDutyCycleSteps = 13653 # int(self.MotorVMax/self.VoltageDutyCycleSteps)
        
        self.kp_bits_q8p8 = 8 # Assumes that TM01 is using the Q8.8 scaling.
        self.ki_bits_q8p8 = 8
        self.kp_bits_q0p16 = 16 # Assumes that TM01 is using the Q0.16 scaling.
        self.ki_bits_q0p16 = 16

        self.kp_torqueflux_scaling_q8p8 = (self.FluxScaling) * ( 2**self.VoltageBits / self.VM ) * 2**self.kp_bits_q8p8 # Should be 133.33 according to the spreadsheet for a 10mOhm shunt Resistor.
        self.ki_torqueflux_scaling_q8p8 = (self.FluxScaling) * ( 2**self.VoltageBits / self.VM ) * 2**self.ki_bits_q8p8 # Should be 133.33 according to the spreadsheet for a 10mOhm shunt Resistor.
        self.kp_torqueflux_scaling_q0p16 = (self.FluxScaling) * ( 2**self.VoltageBits / self.VM ) * 2**self.kp_bits_q0p16 
        self.ki_torqueflux_scaling_q0p16 = (self.FluxScaling) * ( 2**self.VoltageBits / self.VM ) * 2**self.ki_bits_q0p16

        self.kp_torqueflux_scaling_q8p8_Stepper  = self.kp_torqueflux_scaling_q8p8/2.0
        self.ki_torqueflux_scaling_q8p8_Stepper  = self.ki_torqueflux_scaling_q8p8/2.0
        self.kp_torqueflux_scaling_q0p16_Stepper = self.kp_torqueflux_scaling_q0p16/2.0
        self.ki_torqueflux_scaling_q0p16_Stepper = self.ki_torqueflux_scaling_q0p16/2.0

        self.kp_torqueflux_scaling_q8p8_BLDC  = self.kp_torqueflux_scaling_q8p8/1.0
        self.ki_torqueflux_scaling_q8p8_BLDC  = self.ki_torqueflux_scaling_q8p8/1.0
        self.kp_torqueflux_scaling_q0p16_BLDC = self.kp_torqueflux_scaling_q0p16/1.0
        self.ki_torqueflux_scaling_q0p16_BLDC = self.ki_torqueflux_scaling_q0p16/1.0
        
        # NOTE: In the TM01 Units spreadsheet (RL Measurement tab), the Kp_int, Ki_int scaling factors are divided by 2**self.FluxScalingShift which cancels out the 2**self.CurrentScalingFactorShift multiplier in Current scaling factor (mA).

        print("############### Estimated PWM / Voltage / Current Scaling and Limits. ################")
        print("#")
        print(f"# Voltage Scaling = {self.VoltageScaling} Volts/code.")
        print(f"# Flux Scaling = {self.FluxScaling} Amps/code.")
        print(f"# Flux Scaling(RMS) = {self.FluxScalingRMS} Amps/code.")
        print(f"# Max UD/UQ values = {self.MaxUDUQ} codes.")
        print(f"# Max Torque/Flux Peak values = {self.MaxTorqueFluxPeak} codes.")
        print(f"# Max Torque/Flux RMS values = {self.MaxTorqueFluxRMS} codes.")
        print(f"# Max PWM Duty Cycel Step values = {self.MaxPWMDutyCycleSteps} codes.")
        print("#")
        print("########### End of Estimated PWM / Voltage / Current Scaling and Limits. #############")

        # Place Holder for Velocity / Position loop PI scaling.

        self.print_coeffs = False
        self.CLK_FREQUENCY = 40000000
        self.PWM_CLK_FREQUENCY = 120000000
        self.POLES = self.MotorPoles
        self.ABN_ENC_PPR = 4096
        self.VELOCITY_SCALING = 10486
        self.VEL_SCALE = 8
        self.PWM_MAXCNT = 4799
        self.POS_SAMPL = 0
        self.VEL_SAMPL = 0
        self.VELOCITY_NORM_P_SHFT = 0 # Allowed options are: 0, 8, 16, 24
        self.VELOCITY_NORM_I_SHFT = 8 # Allowed options are: 8, 16, 24, 32
        self.POSITION_NORM_P_SHFT = 0 # Allowed options are: 0, 8, 16, 24
        self.POSITION_NORM_I_SHFT = 8 # Allowed options are: 8, 16, 24, 32

        self.vel_norm_p_shft0  = 0
        self.vel_norm_p_shft8  = 8
        self.vel_norm_p_shft16 = 16
        self.vel_norm_p_shft24 = 24

        self.vel_norm_i_shft8  = 8
        self.vel_norm_i_shft16 = 16
        self.vel_norm_i_shft24 = 24
        self.vel_norm_i_shft32 = 32

        self.pos_norm_p_shft0  = 0
        self.pos_norm_p_shft8  = 8
        self.pos_norm_p_shft16 = 16
        self.pos_norm_p_shft24 = 24

        self.pos_norm_i_shft8  = 8
        self.pos_norm_i_shft16 = 16
        self.pos_norm_i_shft24 = 24
        self.pos_norm_i_shft32 = 32

        self.encoder = "phi_e_abn" # Allowed options are: phi_e_abn, phi_m_abn, abn_count, hall_count
        self.vel_meas = "velocity_period" # Allowed options are: velocity_period, velocity_frequency.

    def SetupVelocityPIParams(self):

        self.PWM_FREQUENCY = self.PWM_CLK_FREQUENCY / ( self.PWM_MAXCNT + 1)
        self.POS_UPDATE_RATE =  self.PWM_FREQUENCY / ( self.POS_SAMPL + 1)
        self.VEL_UPDATE_RATE =  self.PWM_FREQUENCY / ( self.VEL_SAMPL + 1)

        self.VEL_PER_RADperSecToMcc_PHI_E  = (2**16 * self.POLES * 2**24 ) / (self.CLK_FREQUENCY * 2 * math.pi )
        self.VEL_PER_RADperSecToMcc_PHI_M  = (2**16              * 2**24 ) / (self.CLK_FREQUENCY * 2 * math.pi )
        self.VEL_PER_RADperSecToMcc_ABN_C  = (self.ABN_ENC_PPR   * 2**24 ) / (self.CLK_FREQUENCY * 2 * math.pi )
        self.VEL_PER_RADperSecToMcc_HALL_C = (6     * self.POLES * 2**24 ) / (self.CLK_FREQUENCY * 2 * math.pi )
        if self.print_coeffs:
            print("#")
            print("# VEL_PER_RADperSecToMcc_PHI_E = ", self.VEL_PER_RADperSecToMcc_PHI_E )
            print("# VEL_PER_RADperSecToMcc_PHI_M = ", self.VEL_PER_RADperSecToMcc_PHI_M )
            print("# VEL_PER_RADperSecToMcc_ABN_C = ", self.VEL_PER_RADperSecToMcc_ABN_C )
            print("# VEL_PER_RADperSecToMcc_HALL_C = ",  self.VEL_PER_RADperSecToMcc_HALL_C)
            print("#")
        else:
            pass
        
        self.VEL_FREQ_RADperSecToMcc_PHI_E  = (2**16 * self.POLES * self.VELOCITY_SCALING ) / (self.VEL_UPDATE_RATE * 2 * math.pi )
        self.VEL_FREQ_RADperSecToMcc_PHI_M  = (2**16              * self.VELOCITY_SCALING ) / (self.VEL_UPDATE_RATE * 2 * math.pi )
        self.VEL_FREQ_RADperSecToMcc_ABN_C  = (self.ABN_ENC_PPR   * self.VELOCITY_SCALING ) / (self.VEL_UPDATE_RATE * 2 * math.pi )
        self.VEL_FREQ_RADperSecToMcc_HALL_C = (6     * self.POLES * self.VELOCITY_SCALING ) / (self.VEL_UPDATE_RATE * 2 * math.pi )
        if self.print_coeffs:
            print("#")
            print("# VEL_FREQ_RADperSecToMcc_PHI_E = ", self.VEL_FREQ_RADperSecToMcc_PHI_E )
            print("# VEL_FREQ_RADperSecToMcc_PHI_M = ", self.VEL_FREQ_RADperSecToMcc_PHI_M )
            print("# VEL_FREQ_RADperSecToMcc_ABN_C = ", self.VEL_FREQ_RADperSecToMcc_ABN_C )
            print("# VEL_FREQ_RADperSecToMcc_HALL_C = ",  self.VEL_FREQ_RADperSecToMcc_HALL_C)
            print("#")
        else:
            pass

        if self.vel_meas == "velocity_period":
            if self.encoder == "phi_e_abn":
                self.VELOCITY_SCALING_FACTOR = self.VEL_PER_RADperSecToMcc_PHI_E
            elif self.encoder == "phi_m_abn":
                self.VELOCITY_SCALING_FACTOR = self.VEL_PER_RADperSecToMcc_PHI_M
            elif self.encoder == "abn_count":
                self.VELOCITY_SCALING_FACTOR = self.VEL_PER_RADperSecToMcc_ABN_C
            elif self.encoder == "hall_count":
                self.VELOCITY_SCALING_FACTOR = self.VEL_PER_RADperSecToMcc_HALL_C
            else:
                print("Encoder Type not defined properly. Valid options are: phi_e_abn, phi_m_abn, abn_count, hall_count")
                exit(1)
        elif self.vel_meas == "velocity_frequency":
            if self.encoder == "phi_e_abn":
                self.VELOCITY_SCALING_FACTOR = self.VEL_FREQ_RADperSecToMcc_PHI_E
            elif self.encoder == "phi_m_abn":
                self.VELOCITY_SCALING_FACTOR = self.VEL_FREQ_RADperSecToMcc_PHI_M
            elif self.encoder == "abn_count":
                self.VELOCITY_SCALING_FACTOR = self.VEL_FREQ_RADperSecToMcc_ABN_C
            elif self.encoder == "hall_count":
                self.VELOCITY_SCALING_FACTOR = self.VEL_FREQ_RADperSecToMcc_HALL_C
            else:
                print("Encoder Type not defined properly. Valid options are: phi_e_abn, phi_m_abn, abn_count, hall_count")
                exit(1)
        else:
            print("Velocity measurement type not defined properly. Valid options are: velocity_period or velocity_frequency")
            exit(1)

        self.VelocityScalingFactor = 1/self.VELOCITY_SCALING_FACTOR # Used in other functions.
        self.VelocityScaling = self.VelocityScalingFactor

        print("#")
        print("# Selected Velocity Scaling Factor (TM01 Chip Units per rads/sec )= ", self.VELOCITY_SCALING_FACTOR)
        print("# Selected Velocity Scaling Factor (rads/ser per TM01 Chip Unit) = ", self.VelocityScaling)
        print("#")
            
        self.VEL_KP_SCALING = (2**self.VEL_SCALE) * (1 / self.FluxScaling) * (1/self.VELOCITY_SCALING_FACTOR) * (2**self.VELOCITY_NORM_P_SHFT)
        self.VEL_KI_SCALING = (2**self.VEL_SCALE) * (1 / self.FluxScaling) * (1/self.VELOCITY_SCALING_FACTOR) * (2**self.VELOCITY_NORM_I_SHFT)

        self.kp_vel_scaling_shft0  = (2**self.VEL_SCALE) * (1 / self.FluxScaling) * (1/self.VELOCITY_SCALING_FACTOR) * (2**self.vel_norm_p_shft0)
        self.kp_vel_scaling_shft8  = (2**self.VEL_SCALE) * (1 / self.FluxScaling) * (1/self.VELOCITY_SCALING_FACTOR) * (2**self.vel_norm_p_shft8)
        self.kp_vel_scaling_shft16 = (2**self.VEL_SCALE) * (1 / self.FluxScaling) * (1/self.VELOCITY_SCALING_FACTOR) * (2**self.vel_norm_p_shft16)
        self.kp_vel_scaling_shft24 = (2**self.VEL_SCALE) * (1 / self.FluxScaling) * (1/self.VELOCITY_SCALING_FACTOR) * (2**self.vel_norm_p_shft24)

        if self.print_coeffs:
            print("#")
            print("# Velocity - Kp Scaling - Shift 0  = ", self.kp_vel_scaling_shft0 )
            print("# Velocity - Kp Scaling - Shift 8  = ", self.kp_vel_scaling_shft8 )
            print("# Velocity - Kp Scaling - Shift 16 = ", self.kp_vel_scaling_shft16)
            print("# Velocity - Kp Scaling - Shift 24 = ", self.kp_vel_scaling_shft24)
            print("#")
        else:
            pass

        self.ki_vel_scaling_shft8  = (2**self.VEL_SCALE) * (1 / self.FluxScaling) * (1/self.VELOCITY_SCALING_FACTOR) * (2**self.vel_norm_i_shft8)
        self.ki_vel_scaling_shft16 = (2**self.VEL_SCALE) * (1 / self.FluxScaling) * (1/self.VELOCITY_SCALING_FACTOR) * (2**self.vel_norm_i_shft16)
        self.ki_vel_scaling_shft24 = (2**self.VEL_SCALE) * (1 / self.FluxScaling) * (1/self.VELOCITY_SCALING_FACTOR) * (2**self.vel_norm_i_shft24)
        self.ki_vel_scaling_shft32  = (2**self.VEL_SCALE) * (1 / self.FluxScaling) * (1/self.VELOCITY_SCALING_FACTOR) * (2**self.vel_norm_i_shft32)
        if self.print_coeffs:
            print("#")
            print("# Velocity - Ki Scaling - Shift 8  = ", self.ki_vel_scaling_shft8 )
            print("# Velocity - Ki Scaling - Shift 16 = ", self.ki_vel_scaling_shft16)
            print("# Velocity - Ki Scaling - Shift 24 = ", self.ki_vel_scaling_shft24)
            print("# Velocity - Ki Scaling - Shift 32 = ", self.ki_vel_scaling_shft32 )
            print("#")
        else:
            pass


    def SetupPositionPIParams(self):
        self.POS_RADToMcc_PHI_E  = (2**16 * self.POLES ) / ( 2 * math.pi )
        self.POS_RADToMcc_PHI_M  = (2**16              ) / ( 2 * math.pi )
        self.POS_RADToMcc_ABN_C  = (self.ABN_ENC_PPR   ) / ( 2 * math.pi )
        self.POS_RADToMcc_HALL_C = (6     * self.POLES ) / ( 2 * math.pi )

        if self.encoder == "phi_e_abn":
            self.POSITION_SCALING_FACTOR = self.POS_RADToMcc_PHI_E
        elif self.encoder == "phi_m_abn":
            self.POSITION_SCALING_FACTOR = self.POS_RADToMcc_PHI_M
        elif self.encoder == "abn_count":
            self.POSITION_SCALING_FACTOR = self.POS_RADToMcc_ABN_C
        elif self.encoder == "hall_count":
            self.POSITION_SCALING_FACTOR = self.POS_RADToMcc_HALL_C
        else:
            print("Encoder Type not defined properly. Valid options are: phi_e_abn, phi_m_abn, abn_count, hall_count")
            exit(1)

        self.POS_KP_SCALING = ( self.VELOCITY_SCALING_FACTOR) * (1/self.POSITION_SCALING_FACTOR) * (2**self.POSITION_NORM_P_SHFT)
        self.POS_KI_SCALING = ( self.VELOCITY_SCALING_FACTOR) * (1/self.POSITION_SCALING_FACTOR) * (2**self.POSITION_NORM_I_SHFT)

        self.kp_pos_scaling_shft0  = ( self.VELOCITY_SCALING_FACTOR) * (1/self.POSITION_SCALING_FACTOR) * (2**self.pos_norm_p_shft0)
        self.kp_pos_scaling_shft8  = ( self.VELOCITY_SCALING_FACTOR) * (1/self.POSITION_SCALING_FACTOR) * (2**self.pos_norm_p_shft8)
        self.kp_pos_scaling_shft16 = ( self.VELOCITY_SCALING_FACTOR) * (1/self.POSITION_SCALING_FACTOR) * (2**self.pos_norm_p_shft16)
        self.kp_pos_scaling_shft24 = ( self.VELOCITY_SCALING_FACTOR) * (1/self.POSITION_SCALING_FACTOR) * (2**self.pos_norm_p_shft24)

        self.ki_pos_scaling_shft8  = ( self.VELOCITY_SCALING_FACTOR) * (1/self.POSITION_SCALING_FACTOR) * (2**self.pos_norm_i_shft8)
        self.ki_pos_scaling_shft16 = ( self.VELOCITY_SCALING_FACTOR) * (1/self.POSITION_SCALING_FACTOR) * (2**self.pos_norm_i_shft16)
        self.ki_pos_scaling_shft24 = ( self.VELOCITY_SCALING_FACTOR) * (1/self.POSITION_SCALING_FACTOR) * (2**self.pos_norm_i_shft24)
        self.ki_pos_scaling_shft32 = ( self.VELOCITY_SCALING_FACTOR) * (1/self.POSITION_SCALING_FACTOR) * (2**self.pos_norm_i_shft32)

        self.PositionScalingFactor = 1/self.POSITION_SCALING_FACTOR # Placeholder to over-ride "magic number" set in class constructor.
        self.PositionScaling = self.PositionScalingFactor

        print("#")
        print("# Selected Position Scaling Factor (TM01 Chip Units per rads )= ", self.POSITION_SCALING_FACTOR)
        print("# Selected Position Scaling Factor (rads per TM01 Chip Unit) = ", self.PositionScaling)
        print("#")

    def get_motor_type(self): # Returns the defined motor type.
        return self.MotorType

    def get_pwm_frequency(self): # Returns the PWM Frequency.
        return self.PWM_FREQUENCY

    def get_voltage_scaling(self): # Returns the Internal Voltage Scaling for UQ, UQ and other voltage registers.
        return self.VoltageScaling
    
    def get_velocity_scaling(self): # Returns the Internal Velocity Scaling for velocity registers ( units are in rads/sec per TM01 Chip Unit).
        return self.VelocityScaling

    def get_position_scaling(self): # Returns the Internal Position Scaling for position registers ( units are in rads per TM01 Chip Unit).
        return self.PositionScaling

    def get_flux_scaling(self): # Returns the Internal Current Scaling for TORQUE, FLUX and other current registers.
        return self.FluxScaling

    def get_torque_flux_pidcoeffs_scaling(self): # Returns Scaled Factor for PI Coefficients for TORQUE/FLUX PI Controller.

        if self.MotorType == "Stepper":
            return self.kp_torqueflux_scaling_q8p8_Stepper, self.ki_torqueflux_scaling_q8p8_Stepper, self.kp_torqueflux_scaling_q0p16_Stepper, self.ki_torqueflux_scaling_q0p16_Stepper
        elif self.MotorType == "BLDC":
            return self.kp_torqueflux_scaling_q8p8_BLDC, self.ki_torqueflux_scaling_q8p8_BLDC, self.kp_torqueflux_scaling_q0p16_BLDC, self.ki_torqueflux_scaling_q0p16_BLDC
        else:
            print("ERROR: Motor Type (", self.MotorType, ") is not supported.")
            exit(1)

    def get_velocity_pidcoeffs_scaling(self): # Returns Scaled Factor for PI Coefficients for Velocity Controller.
        return self.kp_vel_scaling_shft0, self.kp_vel_scaling_shft8, self.kp_vel_scaling_shft16, self.kp_vel_scaling_shft24, self.ki_vel_scaling_shft8, self.ki_vel_scaling_shft16, self.ki_vel_scaling_shft24, self.ki_vel_scaling_shft32
    
    def get_position_pidcoeffs_scaling(self): # Returns Scaled Factor for PI Coefficients for Position Controller.
        return self.kp_pos_scaling_shft0, self.kp_pos_scaling_shft8, self.kp_pos_scaling_shft16, self.kp_pos_scaling_shft24, self.ki_pos_scaling_shft8, self.ki_pos_scaling_shft16, self.ki_pos_scaling_shft24, self.ki_pos_scaling_shft32

    def get_velocity_pidcoeffs_scaling_norm0(self): # Returns Scaled Factor for PI Coefficients for Velocity Controller.
        return self.kp_vel_scaling_shft0, self.ki_vel_scaling_shft8

    def get_velocity_pidcoeffs_scaling_max(self): # Returns Scaled Factor for PI Coefficients for Velocity Controller.
        return self.kp_vel_scaling_shft24, self.ki_vel_scaling_shft32
    
    def get_position_pidcoeffs_scaling_norm0(self): # Returns Scaled Factor for PI Coefficients for Position Controller.
        return self.kp_pos_scaling_shft0, self.ki_pos_scaling_shft8

    def get_position_pidcoeffs_scaling_max(self): # Returns Scaled Factor for PI Coefficients for Position Controller.
        return self.kp_pos_scaling_shft24, self.ki_pos_scaling_shft32
        
    def get_max_uduq(self): # Returns Max Motor Voltage limit (in codes).
        return self.MaxUDUQ
        
    def get_max_torqueflux(self): # Returns Max Peak Motor Current limit (in codes).
        return self.MaxTorqueFluxPeak

    def get_max_torqueflux_rms(self): # Returns Max RMS Motor Current limit (in codes).
        return self.MaxTorqueFluxRMS

    def get_max_pwm_dutycycle(self): # Returns Max PWMDuty Cycle Limit based on Motor Vmax (in codes).
        return self.MaxPWMDutyCycleSteps

    def get_SI_value(self, key, value): # Returns corresponding scaling according to register name. 
        if key == 'MCC.PIDIN_TORQUE_FLUX_TARGET.PIDIN_FLUX_TARGET' or key == 'MCC.PIDIN_TORQUE_FLUX_TARGET.PIDIN_TORQUE_TARGET' or key == 'MCC.PID_TORQUE_FLUX_TARGET.PID_FLUX_TARGET' or key == 'MCC.PID_TORQUE_FLUX_TARGET.PID_TORQUE_TARGET' or key == 'MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL' or key == 'MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL': 
            # print("Using Flux Scaling Value = ",self.FluxScaling, "A for Key = ", key)
            return self.FluxScaling * value
        elif key == 'MCC.VOLTAGE_EXT.UD' or key == 'MCC.VOLTAGE_EXT.UQ' or key == 'MCC.FOC_UQ_UD_LIMITED.UD' or key == 'MCC.FOC_UQ_UD_LIMITED.UQ' or key == 'MCC.FOC_UQ_UD.UD' or key == 'MCC.FOC_UQ_UD.UQ': 
            # print("Using Voltage Scaling Value = ",self.VoltageScalingFactor, "V for Key = ", key)
            return self.VoltageScalingFactor * value
        elif key == 'MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL' or key == 'MCC.PID_VELOCITY_TARGET.PID_VELOCITY_TARGET' or key == 'MCC.PIDIN_VELOCITY_TARGET.PIDIN_VELOCITY_TARGET': 
            # print("Using Velocity Scaling Value = ",self.VelocityScaling, "rads/sec for Key = ", key)
            return self.VelocityScaling * value
        else: 
            raise SystemExit('Wrong register name: ' + key)

    def get_offset(self, key): # Returns corresponding error offset according to register name. 
        if key == 'MCC.PID_TORQUE_FLUX_TARGET.PID_FLUX_TARGET' or key == 'MCC.PID_TORQUE_FLUX_TARGET.PID_TORQUE_TARGET' or key == 'MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL' or key == 'MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL': 
            return 0.0
        elif key == 'MCC.VOLTAGE_EXT.UD' or key == 'MCC.VOLTAGE_EXT.UQ': 
            return 0.0
        elif key == 'MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL': 
            return 0.0
        else: 
            raise SystemExit('Wrong register name: ' + key)

    def get_voltage_value(self, code): # Returns the voltage value for a given UD/UQ code.
        vout = 0.0
        vout = code * self.VoltageScalingFactor
        return vout