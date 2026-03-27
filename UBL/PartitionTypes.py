################################################################################
# Copyright © 2024 Analog Devices, Inc.
################################################################################
from enum import IntEnum, unique

@unique
class PartitionType(IntEnum):
    """
    Partition types for external memory
    """

    APP           = 0  # Application
    # Deprecated:   1
    TMCL_CONFIG   = 2  # TMCL configuration storage
    STIMULUS_DATA = 3  # Stimulus data for system ID/tuning
    TMCL_SCRIPT   = 4  # TMCL script storage
    # Reserved:     5-63 (for Trinamic)
    # Reserved:     64-127 (for Users)
