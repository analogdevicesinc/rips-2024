################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import csv
import numpy as np
import os
from datetime import datetime
from collections import OrderedDict
from ubltools.memory.tm01map._tm01_map_latest import MCC # ToDo: Clean this up

from register_helpers import to_unsigned_32
from register_helpers import to_unsigned_16

class motorDataFileFunctionsClass:
    
    def __init__(self, file_path):
        self.csvfilename = file_path # CSV File Name, used for read and write file names.
        self.headers = [ "timestamp", "DataA", "DataB"] # .csv file headers,  
        self.read_data_list = []       # Data to be returned from a .csv file that is being read.
        self.write_data_list = []      # Data to be written to a .csv file
        self.sample_period = 0.1        # Data Sample Period, used to create timestamp data for .csv fiel write.
        self.register = None
        self.read_data_table = OrderedDict()

    def write(self, data, sample_period):
        """
        data: Dictionary of measured data lists. Keys are register/field names
        sample_period: Sampling period of the measurement (to construct timestamp)
        """
        # Create a copy of the dict
        data = OrderedDict(data)

        # We store the header first to ensure timestamp is the first column
        headers = ["timestamp"] + list(data)

        data_len = len(data[list(data)[0]])

        data["timestamp"] = list([x*sample_period for x in range(data_len) ])

        with open(self.csvfilename, 'w', newline='') as f:
            writer = csv.writer(f, delimiter=";")
            writer.writerow(headers)
            for i in range(data_len):
                writer.writerow([data[x][i] for x in headers])
        print("Written file", self.csvfilename)

    def write_summary(self, header, data): 

        now = datetime.now() # current date and time
        date_time = now.strftime("%Y%m%d_%H%M%S")
        output_fname = os.path.splitext(self.csvfilename)[0]+'_'+date_time+'.csv'

        with open(output_fname, 'w', newline='') as f: 
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(data)
        print("Written data summary", output_fname)

    def read(self):
        header_info = []
        read_data = []
        with open(self.csvfilename, mode ='r') as f:
            reader = csv.reader(f,delimiter=";")
            for row_number, row in enumerate(reader): 
                if row_number == 0: # Treat 1st line in the .csv file as a header line.
                    header_info = row
                    assert row[0] == "timestamp"
                    assert row[1].startswith("MCC.")
                    register_name = row[1]
                    parts = register_name.split(".")
                    assert parts[0] == "MCC"

                    self.register = getattr(MCC, parts[1])
                    assert self.register
                    field=None
                    if len(parts) == 3:
                        # we have a field name
                        field = getattr(self.register, parts[2])
                        assert field

                    print("CSV header result:", self.register, field)

                    for header in header_info: 
                        self.read_data_table[header] = []

                else:
                    raw_value = int(row[1])
                    if field:
                        # Convert the raw value to the right field value
                        value = field.write(raw_value)
                    else:
                        value = raw_value & ((1 << (self.register.WIDTH*8))-1)

                    # print("value", row_number, ":", raw_value, "->", value)
                    read_data.append(value)

                    # Read the whole data file to an ordereddict. 
                    for i, header in enumerate(header_info): 
                        self.read_data_table[header].append(float(row[i]))

        read_data_array = np.array(read_data) # Transpose the array before returning it.
        print("# Array Shape of Data to be passed to stimulus flash: ", read_data_array.shape)
        self.headers = header_info
        self.read_data_list = read_data_array.tolist() # Convert to List.
