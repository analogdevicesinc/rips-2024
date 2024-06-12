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
import ubltools
from ubltools.upload.exceptions import ApplicationException
from ubltools.memory.tm01map._tm01_map_latest import MCC # ToDo: Clean this up
from pytrinamic.connections.serial_tmcl_interface import SerialTmclInterface
from tmcl_flash import TMCLFlash

from motorDataFileFunctionsClass import motorDataFileFunctionsClass


class StimulusReaderClass:

    def __init__(self, tcml_interface=None, tmcl_port=None, ubl_port=None, plot_stimulus=False, csv_file = "", target_register="MCC.PID_VELOCITY_TARGET", no_timeout=True, verbose=0):
        self.tcml_interface = tcml_interface
        self.tmcl_port = tmcl_port
        self.ubl_port = ubl_port
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
        if self.verbose == 3:
            # Keep the ubldevice logger at INFO level (disabling the packet dump)
            bl_logger = logging.getLogger("ubltools.protocol.ubldevice")
            bl_logger.setLevel(logging.INFO)
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
        if self.ubl_port:
            # Disable timeout if flag --no-timeout is used, useful for debugging the bootloader.
            bootloader_timeout = 5
            if self.no_timeout:
                bootloader_timeout = 0

            serial_port = serial.Serial(self.ubl_port, 115200)
            tmc9660 = ubltools.UblDevice(connection=serial_port, chip_address=0x01, timeout=bootloader_timeout)

            # Create SPI memory object
            spi = ubltools.ExternalFlash(tmc9660)

            # Check if there is an available partition table
            try:
                partition_table = ubltools.PartTable.read_from_memory(spi)
            except ApplicationException:
                print("Failed to load table")
                exit(1)

            # Partition table load succeeded -> Search for viable partitions
            viable_partitions = []
            for i, partition in enumerate(partition_table.part_entries):
                logging.debug(f"Checking {partition}")

                if not partition.writable:
                    logging.debug(f"Skipping partition {partition.name} - Read-only")
                    continue

                if partition.type != ubltools.constants.PartitionType.STIMULUS_DATA:
                    logging.debug(f"Skipping partition {partition.name} - Type {partition.type._name_ } not viable for upload")
                    continue

                if partition.size < len(stimulus_bytes):
                    logging.debug(f"Skipping partition {partition.name} - Too small for stimulus data")
                    continue

                logging.debug(f"Partition {partition.name} viable for upload")
                viable_partitions.append((i, partition))

            logging.info(f"Viable partitions: {viable_partitions}")
            if len(viable_partitions) == 0:
                print("Error: No fitting partition found.")
                exit(1)

            # ToDo: Have user choice here via CLI
            partition_nr, upload_to = viable_partitions[0]

            print(f"Uploading stimulus data to partiton {partition_nr} ({upload_to.name}) at address 0x{upload_to.offset:08X}")
            spi.erase(upload_to.offset, upload_to.offset+upload_to.size)
            spi.write(upload_to.offset, stimulus_bytes)

        elif self.tmcl_port:
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
                partition_type = ubltools.constants.PartitionType(tmp & 0x7F)
                partition_writable = (tmp & 0x80) != 0
                partition_start = my_interface.send(65, 1, i, 4).value
                partition_size  = my_interface.send(65, 1, i, 5).value


                # Check if partition is writable
                if not partition_writable:
                    logging.debug(f"Skipping partition {i} - Read-only")
                    continue


                if partition_type != ubltools.constants.PartitionType.STIMULUS_DATA:
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
                partition_type = ubltools.constants.PartitionType(tmp & 0x7F)
                partition_writable = (tmp & 0x80) != 0
                partition_start = my_interface.send(65, 1, i, 4).value
                partition_size  = my_interface.send(65, 1, i, 5).value


                # Check if partition is writable
                if not partition_writable:
                    logging.debug(f"Skipping partition {i} - Read-only")
                    continue


                if partition_type != ubltools.constants.PartitionType.STIMULUS_DATA:
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
