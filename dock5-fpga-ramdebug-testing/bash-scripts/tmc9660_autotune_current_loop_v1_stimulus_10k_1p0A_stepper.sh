################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
#!/bin/bash
set -e

echo "#########################################################################################################"
echo "# Check Environment Variables.                                                                          #"
echo "#########################################################################################################"
if [[ -z "${COM_TMC_DATA}" ]]; then
	echo "COM_TMC_DATA not defined"
	exit 1
fi

if [[ -z "${Stimulus_Data}" ]]; then
	echo "Stimulus_Data not defined"
	exit 1
fi

if [[ -z "${Capture_Data}" ]]; then
	echo "Capture_Data not defined"
	exit 1
fi

echo "#########################################################################################################"
echo "# Basic bash script to run the Torque/Flux Auto-Tuning script.                                          #"
echo "#########################################################################################################"
echo "# Note: 1. The Dock5 board should be power-cycled or reset before running this script.                  #"
echo "#       2. This script should be run from the dock5-fpga-ramdebug-testing/run directory.                #"
echo "#       3. The init_dock5.sh script should be run to properly initialise the Landungsbruke board        #"
echo "#          and upload Firmware.                                                                        #"
echo ""
echo "Step #1. Run the Torque/Flux Loop Auto-Tuning script."
echo ""
python -u ../python/tmc9660_autotune_current_loop_flash_stimulus_v1.py $COM_TMC_DATA \
    --sine-csv-file $Stimulus_Data/matlab_synthetic_flux_data_additive_sine_sweep_110823_combined_norm.csv \
    --ud-output $Capture_Data/motor_ud_step_1000.csv \
    --torque-output $Capture_Data/motor_flux_step_1000.csv \
    --prbs-ud-output $Capture_Data/prbs_ud_output.csv \
    --prbs-flux-output $Capture_Data/prbs_flux_output.csv \
    --sine-flux-output $Capture_Data/sine_flux_multitone_output.csv \
    --damping-factor=1.0 \
    --tuning-method=5 \
    --shunt-resistance=0.005 \
    --shunt-op-amp-gain=20.0 \
    --poles=50 \
    --maximum-current=1.0 \
    --abn-encoder-resolution=10000 \
    --enable-sine-test \
    --current-loop-test-channel="Flux" 
sleep 1.0
echo ""
echo "End of Script."
echo ""
    # --enable-sine-test \
    # --enable-prbs-test \

