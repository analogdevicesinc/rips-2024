################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import logging
import sys

import pytrinamic
from ubltools.memory.tm01map._tm01_map_latest import MCC  # ToDo: Clean this up
from ubltools.memory.tm01map import _tm01_map_latest as MAP
from ubltools.helpers.field import Field
from ubltools.helpers.register import Register
import ubltools

class MccHelpersClass:
    def __init__(self, interface):
        self.my_interface = interface
        # logging.basicConfig(stream=sys.stdout, level=logging.CRITICAL)
        logging.basicConfig(stream=sys.stdout, level=logging.ERROR)
        # logging.basicConfig(stream=sys.stdout, level=logging.WARNING)
        # logging.basicConfig(stream=sys.stdout, level=logging.INFO)
        # logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
        # logging.basicConfig(stream=sys.stdout, level=logging.NOTSET)

        logging.info("Checking whether to use legacy or current register accessing")
        try:
            # TM01 had 12 bit wide register addresses at first, now 11 bit
            bit_width = self.my_interface.send(157, 21, 0, 0).value
            if bit_width == 11:
                self.use_legacy_mapping = False
            elif bit_width == 12:
                self.use_legacy_mapping = True
            else:
                # This case is due to a bug in the register app - where the GET_INFO
                # command used here erroneously just says REPLY_STATUS_OK with the
                # value sent to it in the reply.
                raise RuntimeError("Error: RegApp in P3 has broken GET_INFO")
        except pytrinamic.tmcl.TMCLReplyStatusError:
            # If the GetInfo entry does not exist, use legacy mapping
            self.use_legacy_mapping = True
        logging.info(f"Using {'legacy' if self.use_legacy_mapping else 'current'} register mapping")

    # Helper function - Use the UblTools memory description to access the registers
    # via TMCL. To map the absolute addresses of UblTools to TMCL addresses, we
    # get the register offset divided by 4.
    # E.g. the first register is register number 0, the second register is register 1.
    def get_reg_addr(self, target):
        # Map an address block to it's TMCL number, base address and address width
        block_mapping_legacy = {
                "MCC":       (0,  0x4800_A000, 4),
                "ADC":       (1,  0x4801_C000, 4),
                "SYSCTRL":   (2,  0x4801_B000, 4),
                "CLKCTRL":   (3,  0x4801_A000, 4),
                "GPIO":      (4,  0x4800_2000, 4),
                "IOMATRIX":  (5,  0x4801_3000, 4),
                "AON":       (6,  0x4801_9000, 4),
                "CORDIC":    (7,  0x4800_F000, 4),
                "OTPCTRL":   (8,  0x4802_0000, 4),
                "PIC":       (9,  0x4800_5000, 4),
                "TIMADV":    (10, 0x4801_2000, 4),
                "TIMBASIC0": (11, 0x4801_1000, 4),
                "TIMBASIC1": (12, 0x4801_5000, 4),
                "TIMBASIC2": (13, 0x4801_6000, 4),
                "TIMSYS":    (14, 0x4801_8000, 4),
                "WDT":       (15, 0x4801_7000, 4),
                "I2C":       (16, 0x4800_B000, 4),
                "SPI0":      (17, 0x4800_9000, 4),
                "SPI1":      (18, 0x4800_D000, 4),
                "SPISLAVE":  (19, 0x4800_E000, 4),
                "UART":      (20, 0x4801_0000, 4),
            }
        block_mapping_latest = {
                "MCC":       ( 0, 0x4800_A000, 4),
                "ADC":       ( 1, 0x4801_C000, 4),
                "SYSCTRL":   ( 2, 0x4801_B000, 4),
                "CLKCTRL":   ( 3, 0x4801_A000, 4),
                "GPIO":      ( 4, 0x4800_2000, 4),
                "IOMATRIX":  ( 5, 0x4801_3000, 4),
                "UART":      ( 6, 0x4801_0000, 4),
                "SPI0":      ( 7, 0x4800_9000, 4),
                "SPI1":      ( 8, 0x4800_D000, 4),
                "SPISLAVE":  ( 9, 0x4800_E000, 4),
                "I2C":       (10, 0x4800_B000, 4),
                "TIMBASIC0": (11, 0x4801_1000, 4),
                "TIMBASIC1": (12, 0x4801_5000, 4),
                "TIMBASIC2": (13, 0x4801_6000, 4),
                "TIMADV":    (14, 0x4801_2000, 4),
                "AON":       (15, 0x4801_9000, 4),
                "OTPCTRL":   (16, 0x4802_0000, 4),
                "CORDIC":    (17, 0x4800_F000, 4),
                "PIC":       (18, 0x4800_5000, 4),
                "TIMSYS":    (19, 0x4801_8000, 4),
                "WDT":       (20, 0x4801_7000, 4),
            }

        address_block = target.get_address_block()

        # Note: This is technically a bit unclean since it relies on is_legacy_mapping()
        # being called at least once first. But _tm01_read and _tm01_write both do that.
        block_mapping = block_mapping_legacy if self.use_legacy_mapping else block_mapping_latest

        if address_block._NAME not in block_mapping:
            raise ValueError(f"Invalid register/field: {target}")

        tmcl_block_nr, base_address, register_width = block_mapping[address_block._NAME]

        register_offset = ( target.ADDRESS - base_address ) // register_width

        return (tmcl_block_nr, register_offset)

    # These allow using ubltools register & field definitions conveniently
    def tm01_read(self, read_target):
        if self.use_legacy_mapping:
            # Legacy access - use the pytrinamic functions that are hardcoded to 12 bit register addresses
            if isinstance(read_target, ubltools.helpers.register.Register):
                logging.info(f"Reading register {read_target.get_name()}")
                return self.my_interface.read_mc_by_id(*self.get_reg_addr(read_target))
            elif isinstance(read_target, ubltools.helpers.field.Field):
                logging.info(f"Reading field {read_target.get_name()}")
                reg_val = self.my_interface.read_mc_by_id(*self.get_reg_addr(read_target))
                return read_target.read(reg_val)
            else:
                raise ValueError("Invalid argument to _tm01_read")
        else:
            # Current access - 11 bit mapping, not yet supported directly in pytrinamic
            # We construct the raw TMCL command here instead
            if isinstance(read_target, ubltools.helpers.register.Register):
                logging.info(f"Reading register {read_target.get_name()}")
                reg_block, reg_addr = self.get_reg_addr(read_target)
                tmcl_motor = reg_block | ((reg_addr & 0x0700) >> 3)
                tmcl_type = reg_addr & 0xFF
                return self.my_interface.send(pytrinamic.tmcl.TMCLCommand.READ_MC, tmcl_type, tmcl_motor, 0).value
            elif isinstance(read_target, ubltools.helpers.field.Field):
                logging.info(f"Reading field {read_target.get_name()}")
                reg_block, reg_addr = self.get_reg_addr(read_target)
                tmcl_motor = reg_block | ((reg_addr & 0x0700) >> 3)
                tmcl_type = reg_addr & 0xFF
                reg_val = self.my_interface.send(pytrinamic.tmcl.TMCLCommand.READ_MC, tmcl_type, tmcl_motor, 0).value
                return read_target.read(reg_val)
            else:
                raise ValueError("Invalid argument to tm01_read")

    def tm01_write(self, write_target, value):
        if self.use_legacy_mapping:
            # Legacy access - use the pytrinamic functions that are hardcoded to 12 bit register addresses
            if isinstance(write_target, ubltools.helpers.register.Register):
                logging.info(f"Writing register {write_target.get_name()}, Value {value}")
                self.my_interface.write_mc_by_id(*self.get_reg_addr(write_target), value)
            elif isinstance(write_target, ubltools.helpers.field.Field):
                logging.info(f"Writing field {write_target.get_name()}, value {value}")
                reg_val = self.my_interface.read_mc_by_id(*self.get_reg_addr(write_target))
                reg_val = write_target.update(reg_val, value)
                self.my_interface.write_mc_by_id(*self.get_reg_addr(write_target), reg_val)
            else:
                raise ValueError("Invalid argument to tm01_write")
        else:
            # Current access - 11 bit mapping
            if isinstance(write_target, ubltools.helpers.register.Register):
                logging.info(f"Writing register {write_target.get_name()}, Value {value}")
                reg_block, reg_addr = self.get_reg_addr(write_target)
                tmcl_motor = reg_block | ((reg_addr & 0x0700) >> 3)
                tmcl_type = reg_addr & 0xFF
                self.my_interface.send(pytrinamic.tmcl.TMCLCommand.WRITE_MC, tmcl_type, tmcl_motor, value)
            elif isinstance(write_target, ubltools.helpers.field.Field):
                logging.info(f"Writing field {write_target.get_name()}, value {value}")
                reg_block, reg_addr = self.get_reg_addr(write_target)
                tmcl_motor = reg_block | ((reg_addr & 0x0700) >> 3)
                tmcl_type = reg_addr & 0xFF
                reg_val = self.my_interface.send(pytrinamic.tmcl.TMCLCommand.READ_MC, tmcl_type, tmcl_motor, 0).value
                reg_val = write_target.update(reg_val, value)
                self.my_interface.send(pytrinamic.tmcl.TMCLCommand.WRITE_MC, tmcl_type, tmcl_motor, reg_val)
            else:
                raise ValueError("Invalid argument to tm01_write")

    def mcc_read(self, read_target):
        # print("WARNING: mcc_read function will be deprecated, use tm01_read instead.")
        return self.tm01_read(read_target)

    def mcc_write(self, write_target, value):
        # print("WARNING: mcc_write function will be deprecated, use tm01_write instead.")
        return self.tm01_write(write_target, value)


    def read_all_Registers(self, file_name):
        """
        This function runs the actual test sequence after the TM01 has been
        configured and started.
        """

        print(f"Starting to read out all registers.")
        cmd_write_txt_file = open(file_name, "w")
        cmd_write_txt_file.write(f"Group,Register Name,Register Value,Field Name,Field Value \n")

        groups = [MAP.MCC, MAP.ADC, MAP.SYSCTRL, MAP.CLKCTRL, MAP.GPIO, MAP.IOMATRIX]
   
        for group in groups:
            for reg_name, reg in group.__dict__.items():
                try:
                    value_reg = self.mcc_read(reg)
                    for field_name, field in reg.__dict__.items():
                        if field_name != "_PARENT":
                            print(f"Debug:{field_name}")
                            value_field = self.mcc_read(field)
                            cmd_write_txt_file.write(
                                f"{group._NAME},{reg_name},0x{value_reg:08x},{field_name},{value_field} \n")
                except pytrinamic.tmcl.TMCLReplyStatusError:
                    cmd_write_txt_file.write(f"{group._NAME},{reg_name},error,error,error \n")

        cmd_write_txt_file.close()



    """
    ### MCC register & field helpers ###############################################
    # Helper function - Use the UblTools memory description to access the registers
    # via TMCL. To map the absolute addresses of UblTools to TMCL addresses, we
    # get the register offset divided by 4.
    # E.g. the first register is register number 0, the second register is register 1.


    def get_reg_addr(self,target):
        _BASE_ADDRESS   = 0x4800_A000
        _REGISTER_WIDTH = 4

        return (target.ADDRESS - _BASE_ADDRESS) // _REGISTER_WIDTH

    # These allow using ubltools register & field definitions conveniently
    def mcc_read(self, read_target):

        if isinstance(read_target, Register):
            logging.info(f"Reading register {read_target.get_name()}")
            return self.my_interface.read_mc(self.get_reg_addr(read_target))
        elif isinstance(read_target, Field):
            logging.info(f"Reading field {read_target.get_name()}")
            reg_val = self.my_interface.read_mc(self.get_reg_addr(read_target))
            return read_target.read(reg_val)
        else:
            raise ValueError("Invalid argument to mcc_read")

    def mcc_write(self, write_target, value):

        if isinstance(write_target, Register):
            logging.info(f"Writing register {write_target.get_name()}, Value {value}")
            self.my_interface.write_mc(self.get_reg_addr(write_target), value)
        elif isinstance(write_target, Field):
            logging.info(f"Writing field {write_target.get_name()}, value {value}")
            reg_val = self.my_interface.read_mc(self.get_reg_addr(write_target))
            reg_val = write_target.update(reg_val, value)
            self.my_interface.write_mc(self.get_reg_addr(write_target), reg_val)
        else:
            raise ValueError("Invalid argument to mcc_write")
    """
