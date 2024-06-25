#!/bin/bash
################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################

set -e

echo "#########################################################################################################"
echo "# Check Environment Variables.                                                                          #"
echo "#########################################################################################################"
if [[ -z "${COM_TMC_DATA}" ]]; then
	echo "COM_TMC_DATA not defined"
	exit 1
fi

if [[ -z "${UBLCLI_TOOL}" ]]; then
	echo "UBLCLI_TOOL not defined"
	exit 1
fi

if [[ -z "${TMC9660_BOOT_CONFIG_PARAM}" ]]; then
	echo "TMC9660_BOOT_CONFIG_PARAM not defined"
	exit 1
fi

echo "#########################################################################################################"
echo "# Basic bash script to read the flash partition table from the Dock5 Flash.                             #"
echo "#########################################################################################################"
echo "# Note: 1.    The Dock5 board should be power-cycled or reset before running this script.               #"
echo "#       2.    This script should be run from the dock5-fpga-ramdebug-testing/run directory.             #"
echo "#       3.    This requires that the Landungsbruecke Controller be updated with either:                 #"
echo "#             Landungsbruecke_Controller_v1.04_DOCK5_DEV.hex or                                         #"
echo "#             LandungsbrueckeSmall_Controller_v1.04_DOCK5_DEV.hex firmware depending on the HW Version. #"
echo ""
echo "# Please press the Dock5 HW Reset Button Now.                                                           #"
read -p "Press enter to continue"

# Update the application configuration for the TMC9660-EVAL
# This step applies the OTP config at runtime. It must be done after every powerup or physical reset.
# If the OTP is programmed, this step can be skipped.
echo "### Configuring TM01"
export UBL_PORT=$COM_TMC_DATA
$UBLCLI_TOOL -vvv write config $TMC9660_BOOT_CONFIG_PARAM
echo "### Finished configuring TM01"

echo ""
echo "Step #1. Read the Analog Garage Partition Table from the TMC9660 Flash."
echo ""
$UBLCLI_TOOL -vv inspect SPI header
sleep 1.0
echo ""
echo "End of Script."
echo ""
