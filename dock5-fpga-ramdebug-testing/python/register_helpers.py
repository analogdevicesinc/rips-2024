################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
def to_signed_16(x, mask, shift):
    m = (x >> shift) & mask
    return (m ^ 0x00008000) - 0x00008000

def to_signed_32(x, mask, shift):
    m = (x >> shift) & mask
    return (m ^ 0x80000000) - 0x80000000

def to_unsigned_32(x):
   return x & 0xffff_ffff

def to_unsigned_16(x, mask, shift):
    return (x << shift) & mask
    
def to_register32(msb, lsb):
    m = ( (msb << 16) & 0xffff0000 ) | ( (lsb >> 0) & 0x0000ffff )
    return m