#!/bin/bash
set -e

echo "#########################################################################################################"
echo "# Setup Default Environment Variables.                                                                          #"
echo "#########################################################################################################"
: ${LandungsbrueckeController_Tools:=../..//LandungsbrueckeController/tools}
: ${Ubltools_Scripts:=../../Ubltools/scripts}
: ${TM01_Workspace:=../../Dock5_Package_20220728_v0.9.3/TM01_workspace}
: ${Stimulus_Data:=/c/users/cprende/Software/MotionControlAI/dock5-fpga-ramdebug-testing/stimulus_data{}
: ${Capture_Data:=/c/users/cprende/Software/MotionControlAI/dock5-fpga-ramdebug-testing/capture_data{}
: ${JSON_Config:=/c/users/cprende/Software/MotionControlAI/dock5-fpga-ramdebug-testing/json{}
: ${COM_TMC_CONTROL:=COM17}
: ${COM_TMC_DATA:=COM16}

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
    --ud-output $Capture_Data/motor_ud_step_1000.csv \
    --torque-output $Capture_Data/motor_flux_step_1000.csv \
    --velocity-output $Capture_Data/motor_velocity_sawtooth.csv \
    --offset-output $Capture_Data/motor_velocity_compensated.csv \
    --systemID-output $Capture_Data/systemID.csv \
    --damping-factor=1.0 \
    --tuning-method=5 \
    --shunt-resistance=0.005 \
    --shunt-op-amp-gain=20.0 \
    --target-motor="BLDC" \
    --poles=4 \
    --maximum-current=3.47 \
    --abn-encoder-resolution=1024 \
    --abn-encoder-direction=1
sleep 1.0
echo ""
echo "End of Script."
echo ""
