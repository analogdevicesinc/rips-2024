################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
from tokenize import Double
from scipy.signal import chirp, spectrogram
from scipy.fft import fft, ifft, fftfreq
import numpy as np
import matplotlib.pyplot as plot
from collections import OrderedDict

from register_helpers import to_register32

class TorqueFluxSystemIDTuningClass:
    
    def __init__(self, TM01Setup):

        self.TM01Setup = TM01Setup
        
        self.PWM_FREQUENCY = self.TM01Setup.get_pwm_frequency()
        self.VoltageScaling = self.TM01Setup.get_voltage_scaling()
        self.FluxScaling = self.TM01Setup.get_flux_scaling()



class MotorDataFFTClass:
    
    def __init__(self):
        self.headers = [ "timestamp", "DataA", "DataB"] # .csv file headers,  
        self.read_data_list = []       # Data to be returned from a .csv file that is being read.
        self.write_data_list = []      # Data to be written to a .csv file
        self.sample_period = 0.1        # Data Sample Period, used to create timestamp data for .csv fiel write.
        self.register = None
        self.read_data_table = OrderedDict()


    def fft(self, data, sample_period):
        """
        data: Dictionary of measured data lists. Keys are register/field names
        sample_period: Sampling period of the measurement (to construct timestamp)
        """
        # Create a copy of the dict
        data = OrderedDict(data)

        Npts = len(data[list(data)[0]])
        SamplePeriod = sample_period
        SampleRate = int(1/SamplePeriod)

        StartT = 0
        EndT = Npts/SampleRate
        SampleTime = (EndT-StartT)/Npts

        w = np.array(data[list(data)[2]])

        wf = fft(w)
        wf_mag = np.abs(wf)
        wf_phase = np.angle(wf, deg=True)

        wf_max = np.max(wf_mag)
        wf_norm = np.array(wf_mag)/wf_max
        wf_db = 20*np.log10(wf_norm)

        xf = fftfreq(Npts, SampleTime)[:Npts//2]

        plot.figure()
        plot.subplot(211)
        plot.title("FFT Magnitude")
        plot.plot(xf, wf_db[0:Npts//2])
        
        plot.subplot(212)
        plot.title("FFT Phase")
        plot.plot(xf, wf_phase[0:Npts//2])
     
        plot.grid()
        plot.show()
        print("##### DONE WITH FFT PLOT ########")


