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

if [[ -z "${CAPTURE_DATA}" ]]; then
	echo "CAPTURE_DATA not defined"
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
python -u ../python/tmc9660_autotune_v1.py $COM_TMC_DATA \
    --ud-output $CAPTURE_DATA/motor_ud_step_1000.csv \
    --torque-output $CAPTURE_DATA/motor_flux_step_1000.csv \
    --velocity-output $CAPTURE_DATA/motor_velocity_sawtooth.csv \
    --offset-output $CAPTURE_DATA/motor_velocity_compensated.csv \
    --systemID-output $CAPTURE_DATA/systemID.csv \
    --damping-factor=1.0 \
    --tuning-method=5 \
    --shunt-resistance=0.005 \
    --shunt-op-amp-gain=20.0 \
    --poles=50 \
    --maximum-current=1.0 \
    --abn-encoder-resolution=10000 \
    --current-loop-test-channel="Torque"
sleep 1.0
echo ""
echo "End of Script."
echo ""
#    --current-loop-bw=2000.0 \
#    --current-loop-bw=3296.2 \
