################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import sys

import matplotlib.pyplot as plot
from collections import OrderedDict

import pathlib
MY_SCRIPT_DIR = pathlib.Path(__file__).parent
# This trick is necessary to get the import working from a different folder
# that isn't a subfolder
sys.path.insert(1, str(MY_SCRIPT_DIR / '../../UBL'))
from MemoryMap import TM01

# Grab the address blocks we care about
MCC      = TM01.MCC


class PlotHelperClass:
    def __init__(self):
        print("##### Starting Plot. #####")
        self.data = OrderedDict()

    def PlotData(self, data_input, plot_enable=True, plot_disappear=True):
        self.data = data_input
        if plot_enable:
            plot.figure()
            for key in self.data:
                # Plot the data
                # plot.title(f"target and actual stimulus")
                # plot.plot(data[key], label=key)
                # plot.legend()
                key_reg = MCC.find(key)
                plot_mangitude = key_reg.MASK >> key_reg.SHIFT
                if plot_mangitude == 65535:
                    plot.subplot(211)
                    plot.title("16-bit data")
                    plot.plot(self.data[key], label=key)
                    plot.legend(fontsize=6)
                else:
                    # second line
                    plot.subplot(212)
                    plot.title("32-bit data")
                    plot.plot(self.data[key], label=key)
                    plot.legend(fontsize=6)
            if plot_disappear:
                plot.show(block=False)
            else:
                plot.show()
            print("##### DONE WITH PLOT ########")

            if plot_disappear:
                plot.pause(5)
                plot.close()
