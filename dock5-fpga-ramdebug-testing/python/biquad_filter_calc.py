################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
"""This is a Python module to calculate biquad coefficients for TMC4671 and TM01.

In TMC4671 the Biquad is implemented with 32bit coefficients, in TM01 with 24bit coefficients.
The normalization factors are 2^29/2^20.
The calculation of the biquad output inside TM01 is basically this:
    Sum1 = b_0 * X_n ;
    Sum2 = b_1 * X_n_1;
    Sum3 = b_2 * X_n_2;
    Sum4 = a_1 * Y_n_1;
    Sum5 = a_2 * Y_n_2;
    Y_fine = (Sum1 + Sum2 + Sum3 + Sum4 + Sum5)/norm_fac

    X is input value
    Y is filter output value
    index _n indicates this cycle, while _n_1 indicates the previous cycle etc.

The calculation of the biquad filter coefficients is based on a continuous lowpass filter in a structure like this:

Y(s)                         1
---  = G(s) = ------------------------------------------
U(s)          (1/omega_c**2)*s**2 + 2*D/omega_c * s + 1

(Nominal form of a 2nd order lowpass filter with cutoff frequency and damping)
"""

import dataclasses
import math
import cmath


@dataclasses.dataclass
class RealBiquadFilterCoefficients:
    a_1: float = 0.0
    a_2: float = 0.0
    b_0: float = 0.0
    b_1: float = 0.0
    b_2: float = 0.0


@dataclasses.dataclass
class NormalizedBiquadFilterCoefficients:
    """The coefficients to be written into the chips register."""
    a_1: int = 0
    a_2: int = 0
    b_0: int = 0
    b_1: int = 0
    b_2: int = 0


@dataclasses.dataclass
class TildeBiquadFilterCoefficients:
    a_0: float = 0.0
    a_1: float = 0.0
    a_2: float = 0.0
    b_0: float = 0.0
    b_1: float = 0.0
    b_2: float = 0.0


@dataclasses.dataclass
class ContinuousBiquadFilterCoefficients:
    """Time continuous filter coefficients."""
    a_0: float = 0.0
    a_1: float = 0.0
    a_2: float = 0.0
    b_0: float = 0.0
    b_1: float = 0.0
    b_2: float = 0.0


@dataclasses.dataclass
class LowPassFilter:
    f_p: float
    d_p: float = None


@dataclasses.dataclass
class AntiResonanceFilter:
    f_p: float
    f_z: float
    d_p: float
    d_z: float


class Chip:
    pass


class Tm01(Chip):
    normalization_factor_short = 2 ** 20
    max_value = 2 ** 23 - 1
    min_value = -(2 ** 23)


class Tmc4671(Chip):
    normalization_factor_short = 2 ** 29
    max_value = 2 ** 31 - 1
    min_value = -(2 ** 31)


def calculate_biquad_filter_coefficients(chip_type, f_s, down_sampling_factor, filter_spec):
    @dataclasses.dataclass
    class Result:
        real_coefficients: RealBiquadFilterCoefficients
        normalized_coefficients: NormalizedBiquadFilterCoefficients
        continuous_coefficients: ContinuousBiquadFilterCoefficients
        tilde_coefficients: TildeBiquadFilterCoefficients

    f_s = f_s / (down_sampling_factor + 1)
    T = 1 / f_s

    if isinstance(filter_spec, LowPassFilter):
        # Coefficients for continuous filter
        f_p = filter_spec.f_p
        d_p = filter_spec.d_p

        omega_c = 2.0 * math.pi * f_p
        b_0_cont = 1.0
        b_1_cont = 0.0
        b_2_cont = 0.0
        if d_p is None:
            a_0_cont = 1.0
            a_1_cont = 1.0 / omega_c
            a_2_cont = 0.0
        else:
            a_0_cont = 1.0
            a_1_cont = 2.0 * d_p / omega_c
            a_2_cont = 1.0 / omega_c ** 2.0

    elif isinstance(filter_spec, AntiResonanceFilter):
        # Coefficients for continuous filter
        f_p = filter_spec.f_p
        f_z = filter_spec.f_z
        d_p = filter_spec.d_p
        d_z = filter_spec.d_z

        b_2_cont = 1.0 / ((2.0 * math.pi * f_z) ** 2.0)
        b_1_cont = 2.0 * d_z / (2.0 * math.pi * f_z)
        b_0_cont = 1.0

        a_2_cont = 1.0 / ((2.0 * math.pi * f_p) * (2.0 * math.pi * f_p))
        a_1_cont = 2.0 * d_p / (2.0 * math.pi * f_p)
        a_0_cont = 1.0
    else:
        raise TypeError('Invalid Filter Type!')

    cont_coeffs = ContinuousBiquadFilterCoefficients()
    cont_coeffs.b_0 = b_0_cont
    cont_coeffs.b_1 = b_1_cont
    cont_coeffs.b_2 = b_2_cont
    cont_coeffs.a_0 = a_0_cont
    cont_coeffs.a_1 = a_1_cont
    cont_coeffs.a_2 = a_2_cont

    b_2_z = b_0_cont * T ** 2 + 2 * b_1_cont * T + 4 * b_2_cont
    b_1_z = 2 * b_0_cont * T ** 2 - 8 * b_2_cont
    b_0_z = b_0_cont * T ** 2 - 2 * b_1_cont * T + 4 * b_2_cont

    a_2_z = T ** 2 + 2 * a_1_cont * T + 4 * a_2_cont
    a_1_z = 2 * T ** 2 - 8 * a_2_cont
    a_0_z = T ** 2 - 2 * a_1_cont * T + 4 * a_2_cont

    b_2_tilde = b_2_z / a_0_z
    b_1_tilde = b_1_z / a_0_z
    b_0_tilde = b_0_z / a_0_z

    a_2_tilde = a_2_z / a_0_z
    a_1_tilde = a_1_z / a_0_z
    a_0_tilde = a_0_z / a_0_z

    time_disc_coeffs = TildeBiquadFilterCoefficients()
    time_disc_coeffs.b_0 = b_0_tilde
    time_disc_coeffs.b_1 = b_1_tilde
    time_disc_coeffs.b_2 = b_2_tilde
    time_disc_coeffs.a_0 = a_0_tilde
    time_disc_coeffs.a_1 = a_1_tilde
    time_disc_coeffs.a_2 = a_2_tilde

    b_0 = b_2_tilde / a_2_tilde
    b_1 = b_1_tilde / a_2_tilde
    b_2 = b_0_tilde / a_2_tilde

    a_0 = a_2_tilde / a_2_tilde
    a_1 = a_1_tilde / a_2_tilde
    a_2 = a_0_tilde / a_2_tilde

    real_coeffs = RealBiquadFilterCoefficients()
    real_coeffs.b_0 = b_0
    real_coeffs.b_1 = b_1
    real_coeffs.b_2 = b_2
    real_coeffs.a_1 = a_1
    real_coeffs.a_2 = a_2

    norm_coeffs = NormalizedBiquadFilterCoefficients()
    norm_coeffs.b_0 = round(b_0 * chip_type.normalization_factor_short)
    norm_coeffs.b_1 = round(b_1 * chip_type.normalization_factor_short)
    norm_coeffs.b_2 = round(b_2 * chip_type.normalization_factor_short)
    norm_coeffs.a_1 = round(-a_1 * chip_type.normalization_factor_short)
    norm_coeffs.a_2 = round(-a_2 * chip_type.normalization_factor_short)

    # Coefficient balancing is needed.
    # The sum of all coefficients needs to be equal to normalization_factor_short. Why is this so?
    # Let's say the filter input X is steady then we want the filter output Y to be steady as well after a certain time.
    # X shall be Y.
    # Y = (b_0_S24 + b_1_S24 + b_2_S24 + a_1_S24 + a_2_S24) * X/normalization_factor_short
    # as the rounding might take away some portion of the coefficient this needs to be compensated.
    norm_coeffs.a_1 += chip_type.normalization_factor_short - (
                norm_coeffs.b_0 + norm_coeffs.b_1 + norm_coeffs.b_2 + norm_coeffs.a_1 + norm_coeffs.a_2)

    # Range check for the normalized coefficients, to make sure they fit in the chips register.
    for coeff_name, coeff_value in vars(norm_coeffs).items():
        if not chip_type.min_value < coeff_value < chip_type.max_value:
            raise AttributeError(f'Error: {coeff_name} is too small!')

    root = abs(cmath.sqrt(-a_2 / a_0 + a_1 ** 2 / (4 * a_0 ** 2)) - a_1 / (2 * a_0))

    print("Root = ", root)
    if root > 0.98:
        raise AttributeError('The given filter configuration would result in an unstable filter!')

    return Result(real_coeffs, norm_coeffs, cont_coeffs, time_disc_coeffs)
