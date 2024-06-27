################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import argparse
import logging
import struct
import sys

import matplotlib.pyplot as plot
import serial
from pytrinamic.connections.serial_tmcl_interface import SerialTmclInterface
from tmcl_flash import TMCLFlash

from motorDataFileFunctionsClass import motorDataFileFunctionsClass

import pathlib
MY_SCRIPT_DIR = pathlib.Path(__file__).parent
# This trick is necessary to get the import working from a different folder
# that isn't a subfolder
sys.path.insert(1, str(MY_SCRIPT_DIR / '../../UBL'))
from PartitionTypes import PartitionType

class StimulusReaderClass:

    def __init__(self, tcml_interface=None, tmcl_port=None, plot_stimulus=False, csv_file = "", target_register="MCC.PID_VELOCITY_TARGET", no_timeout=True, verbose=0):
        self.tcml_interface = tcml_interface
        self.tmcl_port = tmcl_port
        self.plot_stimulus = plot_stimulus
        self.csv_file_name = csv_file
        self.stimulus_register_name = target_register
        self.no_timeout = no_timeout
        self.verbose = verbose
        self.myFileIO = motorDataFileFunctionsClass(self.csv_file_name)

        self.STIMULUS_VERSION_MAJOR = 0
        self.STIMULUS_VERSION_MINOR = 1
        
        if self.verbose == 0:
            log_level = logging.ERROR
        elif self.verbose == 1:
            log_level = logging.WARNING
        elif self.verbose == 2:
            log_level = logging.INFO
        elif self.verbose >= 3:
            log_level = logging.DEBUG
        # Create local logger
        logging.basicConfig(stream=sys.stdout, level=log_level)
        
    def GenerateStimulusData(self):

        ### Generate stimulus data #####################################################
        # Single Channel Only - Backwards Compatible function.
        stimulus_channels = 1
        stimulus = []
        self.myFileIO.read()

        # Assign Stimulus Channel 0 from file.
        stimulus.append([])
        stimulus[0] = self.myFileIO.read_data_list
        TARGET_REGISTER = self.myFileIO.register

        # Assign Stimulus Channel 1 to all 0s.
        # stimulus.append(list(range(len(stimulus[0]))))

        for channel in range(len(stimulus)):
            for i, val in enumerate(stimulus[channel]):
                print(f"Channel {channel} Sample {i:3d}: 0x{val:08X}")

        ### Prepare stimulus data for flash storage ####################################
        # Pack the stimulus data
        stimulus_bytes = struct.pack("<4sBBxxBxH", "stim".encode("utf-8"), self.STIMULUS_VERSION_MAJOR, self.STIMULUS_VERSION_MINOR, len(stimulus), len(stimulus[0]))
        for idx in range(len(stimulus[0])):
            for channel in range(len(stimulus)):
                stimulus_bytes += struct.pack("<I", stimulus[channel][idx])

        logging.debug(f"Raw stimulus data ({len(stimulus_bytes)} bytes): {stimulus_bytes}")

        ### Plot the stimulus data if requested ########################################
        if self.plot_stimulus:
            plot.figure()
            for channel in range(len(stimulus)):
                plot.title(f"Stimulus")
                plot.plot(stimulus[channel], label=f"Channel {channel}")
            plot.legend()
            plot.show()

        ### Upload the stimulus if requested ###########################################
        if self.tmcl_port:
            # Disable timeout if flag --no-timeout is used
            tmcl_timeout = 5
            if args.no_timeout:
                tmcl_timeout = 0

            my_interface = SerialTmclInterface(self.tmcl_port, timeout_s=tmcl_timeout)

            # Check if there is an available partition table
            partition_count = my_interface.send(65, 0, 0, 2).value
            if partition_count == 0:
                print("No partition found. Check that partition table is flashed correctly")
                exit(1)

            # Partition table load succeeded -> Search for viable partitions
            viable_partitions = []
            for i in range(partition_count):
                logging.debug(f"Checking partition {i}")

                tmp = my_interface.send(65, 1, i, 3).value
                partition_type = PartitionType(tmp & 0x7F)
                partition_writable = (tmp & 0x80) != 0
                partition_start = my_interface.send(65, 1, i, 4).value
                partition_size  = my_interface.send(65, 1, i, 5).value


                # Check if partition is writable
                if not partition_writable:
                    logging.debug(f"Skipping partition {i} - Read-only")
                    continue


                if partition_type != PartitionType.STIMULUS_DATA:
                    logging.debug(f"Skipping partition {i} - Type {partition_type._name_ } not viable for upload")
                    continue

                if partition_size < len(stimulus_bytes):
                    logging.debug(f"Skipping partition {i} - Too small for stimulus data")
                    continue

                logging.debug(f"Partition {i} viable for upload")
                viable_partitions.append((i, partition_start, partition_size))

            logging.info(f"Viable partitions: {viable_partitions}")
            if len(viable_partitions) == 0:
                print("Error: No fitting partition found.")
                exit(1)


            # ToDo: Have user choice here via CLI
            upload_to = viable_partitions[0]

            print(f"Uploading stimulus data to partiton {upload_to[0]} at address 0x{upload_to[1]:08X}")
            sector_size = 1 << my_interface.send(65, 0, 0, 1).value

            # Wait for isBusy == 0
            while my_interface.send(65, 0, 0, 3).value != 0:
                pass

            for i in range(upload_to[1], upload_to[1]+upload_to[2], sector_size):
                print("Erasing", hex(i))
                my_interface.send(65, 2, 0, i) # Erase Sector
                # Wait for isBusy == 0
                while my_interface.send(65, 0, 0, 3).value != 0:
                    pass

            for i in range(len(stimulus_bytes)):
                my_interface.send(65, 3, stimulus_bytes[i], upload_to[1]+i)

            print(f"Write complete")

            # Close the connection.
            my_interface.close()

        elif self.tcml_interface:

            my_interface = self.tcml_interface

            # Check if there is an available partition table
            partition_count = my_interface.send(65, 0, 0, 2).value
            if partition_count == 0:
                print("No partition found. Check that partition table is flashed correctly")
                exit(1)

            # Partition table load succeeded -> Search for viable partitions
            viable_partitions = []
            for i in range(partition_count):
                logging.debug(f"Checking partition {i}")

                tmp = my_interface.send(65, 1, i, 3).value
                partition_type = PartitionType(tmp & 0x7F)
                partition_writable = (tmp & 0x80) != 0
                partition_start = my_interface.send(65, 1, i, 4).value
                partition_size  = my_interface.send(65, 1, i, 5).value


                # Check if partition is writable
                if not partition_writable:
                    logging.debug(f"Skipping partition {i} - Read-only")
                    continue


                if partition_type != PartitionType.STIMULUS_DATA:
                    logging.debug(f"Skipping partition {i} - Type {partition_type._name_ } not viable for upload")
                    continue

                if partition_size < len(stimulus_bytes):
                    logging.debug(f"Skipping partition {i} - Too small for stimulus data")
                    continue

                logging.debug(f"Partition {i} viable for upload")
                viable_partitions.append((i, partition_start, partition_size))

            logging.info(f"Viable partitions: {viable_partitions}")
            if len(viable_partitions) == 0:
                print("Error: No fitting partition found.")
                exit(1)


            # ToDo: Have user choice here via CLI
            upload_to = viable_partitions[0]

            print(f"Uploading stimulus data to partiton {upload_to[0]} at address 0x{upload_to[1]:08X}")
            sector_size = 1 << my_interface.send(65, 0, 0, 1).value

            # Wait for isBusy == 0
            while my_interface.send(65, 0, 0, 3).value != 0:
                pass

            for i in range(upload_to[1], upload_to[1]+upload_to[2], sector_size):
                print("Erasing", hex(i))
                my_interface.send(65, 2, 0, i) # Erase Sector
                # Wait for isBusy == 0
                while my_interface.send(65, 0, 0, 3).value != 0:
                    pass

            for i in range(len(stimulus_bytes)):
                my_interface.send(65, 3, stimulus_bytes[i], upload_to[1]+i)

            # Close the connection.
            # my_interface.close()

        else:
            print("No Port specified or Found!")
            exit(1)
