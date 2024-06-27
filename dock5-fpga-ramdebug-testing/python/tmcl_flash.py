################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
from pytrinamic.tmcl import TMCLCommand

TMCL_FLASH_CMD = TMCLCommand.TMCL_UF1

SUBCMD_GET_INFO      = 0
SUBCMD_GET_PARTITION = 1
SUBCMD_ERASE_SECTOR  = 2
SUBCMD_WRITE_BYTE    = 3
SUBCMD_READ_BYTE     = 4

MAX_FLASH_SIZE = 1 << 24

import pathlib
import sys
MY_SCRIPT_DIR = pathlib.Path(__file__).parent
# This trick is necessary to get the import working from a different folder
# that isn't a subfolder
sys.path.insert(1, str(MY_SCRIPT_DIR / '../../UBL'))
from PartitionTypes import PartitionType

class TMCLFlash():
    def __init__(self, connection):
        self._connection = connection

    def get_size(self):
        size_idx = self._cmd(SUBCMD_GET_INFO, 0, 0).value
        return 1 << size_idx if size_idx else MAX_FLASH_SIZE

    def get_sector_size(self):
        sector_idx = self._cmd(SUBCMD_GET_INFO, 0, 1).value
        return 1 << sector_idx

    def get_partition_count(self):
        return self._cmd(SUBCMD_GET_INFO, 0, 2).value

    def is_busy(self):
        return self._cmd(SUBCMD_GET_INFO, 0, 3).value

    def get_partition(self, index):
        partition_type  = self._cmd(SUBCMD_GET_PARTITION, index, 3).value
        partition_start = self._cmd(SUBCMD_GET_PARTITION, index, 4).value
        partition_size  = self._cmd(SUBCMD_GET_PARTITION, index, 5).value

        partition = {
            "start": partition_start,
            "size": partition_size,
            "type": PartitionType(partition_type & 0x7F),
            "writable": (partition_type & 0x80) != 0,
            }

        return partition

    def erase_sector(self, sector_addr, wait_for_completion=True):
        self._cmd(SUBCMD_ERASE_SECTOR, 0, sector_addr)

        if wait_for_completion:
            while self.is_busy():
                pass

    def write_byte(self, addr, value):
        # print(f"flash[{addr:06X}] = {value}")
        self._cmd(SUBCMD_WRITE_BYTE, value, addr)

    def read(self, addr, byte_count=4):
        if not (0 < byte_count <= 4):
            raise ValueError("Invalid read size")

        return self._cmd(SUBCMD_READ_BYTE, byte_count, addr).value

    def _cmd(self, op_type, motor, value, module_id=None):
        return self._connection.send(TMCL_FLASH_CMD, op_type, motor, value, module_id=module_id)