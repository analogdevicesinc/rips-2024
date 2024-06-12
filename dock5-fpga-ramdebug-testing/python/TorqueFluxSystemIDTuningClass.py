################################################################################
# Copyright © 2024 Analog Devices Inc. All Rights Reserved.
# This software is proprietary to Analog Devices, Inc. and its licensors.
################################################################################
import time
import math
from tkinter import E
import numpy as np
import matplotlib.pyplot as plt

from TM01SystemUnitsSetup import TM01SystemUnitsSetup
from register_helpers import to_register32

class TorqueFluxSystemIDTuningClass:
    
    def __init__(self, TM01Setup):

        self.TM01Setup = TM01Setup
        
        self.PWM_FREQUENCY = self.TM01Setup.get_pwm_frequency()
        self.Tsample = 1/self.PWM_FREQUENCY
        self.VoltageScaling = self.TM01Setup.get_voltage_scaling()
        self.FluxScaling = self.TM01Setup.get_flux_scaling()
        self.VelocityScaling =  self.TM01Setup.get_velocity_scaling()

        self.ud_nonzero_idx = 0
        self.id_nonzero_idx = 0

        self.freq3dB = 0
        self.w_natural = 0
        self.freq_natural = 0
        self.wn_mech = 0.
        self.po = 0
        self.epsilon = 1e-6

    def gaussiankernel(self, length=3, sigma=2):
        # Create a Gaussian Kernel, given lenght and sigma.
        x = np.linspace(-(length - 1) / 2., (length - 1) / 2., length)
        gaussian = np.exp(-0.5 * (np.square(x) ) / np.square(sigma))
        gaussian = gaussian / np.sum(gaussian)
        return gaussian

    def gaussianconvolve(self, input, kernel, mode='valid'):
        output_full = np.convolve(input, kernel, mode=mode)
        input_len  = len(input)
        # output_len = len(output_full)
        # start = int((output_len-input_len)/2)
        # end = start+input_len
        # output = output_full[start:end]
        output = output_full
        return output

    def moving_average_filter(self, time_series, signal, window_size=2): 
        """ This function applies a moving average filter to the signal. """
        signal_averaged = [0 for _ in range(window_size-1)]
        for i in range(len(signal) - int(window_size) + 1): 
            window_signal = signal[i : i + window_size]
            window_time = time_series[i : i + window_size]
            signal_averaged.append(np.average(window_signal))
        signal_average = np.array(signal_averaged)

        return signal_average

    def apply_filter(self, signal, num_d, den_d): 
        """ This function applies a filter using its discrete transfer function. """
        n = len(signal)
        in_order = len(num_d)
        out_order = len(den_d)

        signal_filtered = [signal[0]]
        for i in range(1, n): 
            item = 0
            for j in range(in_order): 
                if i - j >= 0: 
                    item += num_d[j] * signal[i-j]
                else: 
                    item += num_d[j] * signal[0]

            for j in range(1, out_order): 
                if i - j >= 0: 
                    item -= den_d[j] * signal_filtered[i-j]
                else: 
                    item -= den_d[j] * signal_filtered[0]
            item /= den_d[0]
            signal_filtered.append(item)

        return signal_filtered

    def butterworth_filter(self, signal, ts, wc_ratio, zeta=np.sqrt(2)/2): 
        """ Apply butterworth filter forward and backward on the given signal. """
        
        wc = wc_ratio * 2 * np.pi / ts

        """ Continuous transfer function of Butterworth filter. """
        num_c = [1]
        den_c = [1/wc**2, 2*zeta/wc, 1]

        """ Discrete transfer function of Butterworth filter (converted from continuous using Tusting discretization). """
        num_d = [1, 2, 1]
        den_d = [4/ts**2/wc**2 + 4*zeta/wc/ts + 1, 2 - 8/ts**2/wc**2, 4/ts**2/wc**2 - 4*zeta/wc/ts + 1]

        # print("standard coeffs", num_d/den_d[0], den_d/den_d[0])
        signal_filtered_forward = self.apply_filter(signal, num_d, den_d)
        signal_filtered_forward_flip = np.flip(signal_filtered_forward)
        signal_filtered_backward = self.apply_filter(signal_filtered_forward_flip, num_d, den_d)
        signal_filtered = np.flip(signal_filtered_backward)

        return signal_filtered

    def verify_torque_Kp_Ki_taud(self, data, input_name, output_name, R_est, L_est, period=None): 
        """ This function estimate the Kp Ki values for torque PI controller using RLS as a verification. """
        in_mat = np.identity(1)
        out_mat = np.identity(2)
        order = int(np.sum(in_mat) + np.sum(out_mat))

        constraint = np.zeros(order)
        constraint_val = np.zeros(order)

        # input_name = "MCC.PID_TORQUE_FLUX_TARGET.PID_FLUX_TARGET"
        # output_name = "MCC.PID_TORQUE_FLUX_ACTUAL.PID_FLUX_ACTUAL"
        input_filter_wn = 0.1
        output_filter_wn = 0.1
        coeffs = self.SystemID_Sdomain_RLS(data, input_name, output_name, in_mat, out_mat, constraint, constraint_val, input_filter_wn, output_filter_wn, period)

        if (period):
            delta_t = period
        else:
            if 'timestamp' in data: 
                t = np.array([value for value in data['timestamp']])
            else: 
                print("System ID Motor signal timestamp not found!")
                exit(1)
            delta_t = t[1] - t[0]

        a, b, c = coeffs
        x = (1 - a / (1 - b)) * 2 / delta_t**2
        y = (1 + b) / (1 - b) / delta_t
        z = c / (1 - a - b)

        Ki = x / 2 / y * R_est
        tau_d = 1 / y / 2
        Kp = L_est / R_est * Ki

        return Kp, Ki * delta_t, tau_d / delta_t

    def calculate_gradient(self, func, x): 
        n_dim = len(x)
        gradient = []

        for i in range(n_dim): 
            x_upper = [x[j] if not j == i else x[j] + self.epsilon for j in range(n_dim)]
            x_lower = [x[j] if not j == i else x[j] - self.epsilon for j in range(n_dim)]
            gradient.append((func(x_upper) - func(x_lower)) / (2 * self.epsilon))

        return np.array(gradient)

    def find_stepsize(self, func, x, p, gradient, alpha=1): 

        
        fx = func(x)
        grad_project = gradient.T@p

        n_steps = 1000
        while n_steps > 0: 
            n_steps -= 1 
            x_new = x + alpha * p
            gradient_new = self.calculate_gradient(func, x_new)
            grad_proj_new = gradient_new.T@p

            if func(x_new) < fx + (self.epsilon * alpha * grad_project) or grad_proj_new > grad_project: 
                break
            alpha /= 2

        return alpha

    def minimize_BFGS(self, func, ini): 
        """ BFGS algorithm to find the parameters where the target function is minimum. """

        n_dim = len(ini)
        n_steps = 10000

        # initialization
        Hessian = np.identity(n_dim)
        x = ini
        gradient = self.calculate_gradient(func, ini)

        while np.linalg.norm(gradient) > self.epsilon and n_steps > 0: 
            n_steps -= 1

            p = -np.matmul(Hessian, gradient)   # search direction
            alpha = self.find_stepsize(func, x, p, gradient)
            s = np.array([alpha * p]).T
            x += s.T.squeeze()
            y = np.array([self.calculate_gradient(func, x) - gradient]).T
            
            Hessian += (s.T@y + y.T@Hessian@y) * (s@s.T) / (s.T@y)**2 - (Hessian@y@s.T + s@y.T@Hessian) / (s.T@y)
            gradient += y.T.squeeze()

        return x

    def find_divisor_mod_factor(self, omega, omega_target, delta_t, PWM_FREQUENCY, omega_eval_ratio=0.1, omega_eval_window_size=10, plot_enable=True, plot_disappear=False): 

        N = len(omega)

        # Filter the velocity before calculating acceleration. 
        omega_filtered = self.butterworth_filter(omega, delta_t, 0.05)
        omega_target_filtered = self.butterworth_filter(omega_target, delta_t, 0.05)

        if np.argmax(omega_filtered) < np.argmax(omega_target_filtered) * 1.05: 
            return 1

        omega_target_max = max(omega_target)

        omega_actual_nonzero_idx = 0
        for i in range(len(omega)):
            if omega_filtered[i] > 0:
                print("actual omega nonzero idx is", i, "values is", omega_filtered[i])
                omega_actual_nonzero_idx = i
                break

        peak_samples = np.argmax(omega_target)
        peak_time = delta_t * peak_samples
        # print(f"delta t is {delta_t}, peak time is {peak_time}. ")

        t = [i * delta_t for i in range(peak_samples)]
        acc_actual_list = []
        acc_target_list = []
        acc_ratio_list = []
        for i in range(omega_actual_nonzero_idx, peak_samples): 
            start_pt = max(0, i - omega_eval_window_size)
            end_pt = min(i + omega_eval_window_size, N-1)
            acc_actual_eval = np.mean([self.get_acceleration_at_idx(delta_t, omega_filtered, j) for j in range(start_pt, end_pt)])
            acc_target_eval = np.mean([self.get_acceleration_at_idx(delta_t, omega_target_filtered, j) for j in range(start_pt, end_pt)])
            acc_actual_list.append(acc_actual_eval)
            acc_target_list.append(acc_target_eval)
            acc_ratio_list.append(acc_target_eval / acc_actual_eval)

        acc_eval = np.mean([self.get_acceleration_at_idx(delta_t, omega_filtered, i) for i in range(start_pt, end_pt)])
        acc_target_eval = np.mean([self.get_acceleration_at_idx(delta_t, omega_target_filtered, i) for i in range(start_pt, end_pt)])
        acc_actual_target_ratio = acc_target_eval / acc_eval

        velocity_ratio_list = [omega_target_filtered[i] / omega_filtered[i] for i in range(omega_actual_nonzero_idx, peak_samples)]
        velocity_acc_ratio = [velocity_ratio_list[i] / acc_ratio_list[i] for i in range(peak_samples - omega_actual_nonzero_idx)]

        theoretical_acc = omega_target_max / peak_time
        acc_actual_max = max(acc_actual_list)
        acc_conv_idx = 0

        for i in range(peak_samples - omega_actual_nonzero_idx): 
            acc_conv_idx = i
            if acc_actual_list[i] > 0.5 * acc_actual_max: 
                break
        acc_slope = (acc_actual_list[acc_conv_idx] - acc_actual_list[0]) / (acc_conv_idx * delta_t)
        acc_slope_new, acc_intercept = self.linear_fit_BFGS(np.array(t[omega_actual_nonzero_idx:omega_actual_nonzero_idx+acc_conv_idx]), np.array(acc_actual_list[:acc_conv_idx]))
        start_time = -acc_intercept / acc_slope_new
        divisor_updated_factor_new = omega_target_max / acc_slope_new / omega_eval_ratio / (peak_time - start_time)**2
        divisor_updated_factor = omega_target_max / acc_slope / omega_eval_ratio / (peak_time)**2

        if plot_enable: 

            fig, (a0, a1, a2) = plt.subplots(3, 1, gridspec_kw={'height_ratios': [1, 1, 1]}, figsize=(16, 16))
            plt.subplots_adjust(hspace=0)
            a0.plot(omega_filtered[:peak_samples], label='actual', color='tomato')
            a0.plot(omega_target_filtered[:peak_samples], label='target', color='cornflowerblue')
            a1.plot(t[omega_actual_nonzero_idx:peak_samples], acc_actual_list[:peak_samples], label='actual', color='tomato')
            a1.plot(t[omega_actual_nonzero_idx:peak_samples], acc_target_list[:peak_samples], label='target', color='cornflowerblue')
            a2.plot(velocity_ratio_list, label='velocity', color='cornflowerblue')
            a2.plot(acc_ratio_list, label='acceleration', color='tomato')
            a1.axhline(y = theoretical_acc, color='black')
            a2.axhline(y = 1, color='lightgray')
            a0.legend()
            a1.legend()
            a2.legend()
            a0.set_ylabel("$\omega (rad / s)$")
            a1.set_ylabel("acceleration $(rad / s^2)$")
            a2.set_ylabel("target / actual ratio")
            a2.set_xlabel("sample #")

            if not plot_disappear: 
                plt.show()
            else: 
                plt.pause(5)
                plt.close()

        return divisor_updated_factor

    def extract_data_for_sawtooth(self, data, omega_register_name, iq_register_name, period=None): 

        if omega_register_name in data:
            key = omega_register_name
            omega = np.array([self.TM01Setup.get_SI_value(key, value) for value in data[key]])
        else:
            print("System ID Motor velocity register name not found!")
            exit(1)

        if iq_register_name in data:
            key = iq_register_name
            iq = np.array([self.TM01Setup.get_SI_value(key, value) for value in data[key]])
        else:
            print("System ID Motor torque register name not found!")
            exit(1)

        if (period):
            delta_t = period
            t = np.array([delta_t * i for i in range(len(omega))])
        else:
            if 'timestamp' in data: 
                t = np.array([value for value in data['timestamp']])
            else: 
                print("System ID Motor signal timestamp not found!")
                exit(1)
            delta_t = t[1] - t[0]

        N = len(omega)
        
        zero_pt = N // 2
        for i in range(N//4, N): 
            if omega[i] <= 0: 
                zero_pt = i
                break

        return np.array(omega[:zero_pt]), np.array(iq[:zero_pt])
        
    def get_acceleration_at_idx(self, delta_t, omega, idx, window_size=10): 
        start_pt = max(0, idx - window_size)
        end_pt = min(idx + window_size, len(omega) - 1)

        acceleration = (omega[end_pt] - omega[start_pt]) / (delta_t * (end_pt - start_pt))
        # print(f"acceleration at idx {idx}, denominator is {omega[end_pt] - omega[start_pt]}, numerator is {delta_t * (end_pt - start_pt)}, delta_t is {delta_t}")

        return acceleration

    def get_acceleration(self, delta_t, omega, window_size=10, average_window_size = 10): 
        
        acc = []
        for i in range(len(omega)): 
            acc.append(self.get_acceleration_at_idx(delta_t, omega, i, window_size))

        acc_average_window_list = acc[:int(average_window_size / 2)]
        acc_sum = np.sum(acc_average_window_list)
        acc_average_list = [acc_sum / len(acc_average_window_list)]
        for i in range(1, len(acc)): 
            if len(acc_average_window_list) < average_window_size: 
                acc_average_window_list.append(acc[i])
                acc_sum += acc[i]
                acc_average_list.append(acc_sum / len(acc_average_window_list))
            else: 
                acc_sum -= acc_average_window_list.pop(0)
                acc_average_window_list.append(acc[i])
                acc_sum += acc[i]
                acc_average_list.append(acc_sum / len(acc_average_window_list))

        return np.array(acc_average_list)

    def linear_interpolator(self, x, y, x0): 
        """ Given two array x and y, find the corresponding y0 for arbitrary x0. 
            x is an increasing array. 
        """

        if x0 < min(x): 
            y0 = y[0] - (y[1] - y[0]) / (x[1] - x[0]) * (x[0] - x0)
            return y0

        if x0 > max(x): 
            y0 = y[-1] + (y[-1] - y[-2]) / (x[-1] - x[-2]) * (x0 - x[-1])
            return y0

        N = len(x)
        i_x0 = min(int((x0 - x[0]) / (x[-1] - x[0]) * N), N-2)
        while i_x0 > 0 and x[i_x0] > x0: 
            i_x0 -= 1
        while i_x0 + 1 < N - 1 and x[i_x0 + 1] < x0: 
            i_x0 += 1

        y0 = y[i_x0] + (y[i_x0+1] - y[i_x0]) / (x[i_x0+1] - x[i_x0]) * (x0 - x[i_x0])
        # print(f"interpolator x0 is {x0}, x1 is {x[i_x0]}, x1+1 is {x[i_x0+1]}, y1 is {y[i_x0]}, x2 is {x[i_x0+1]}, y2 is {y[i_x0+1]}. ")
        
        return y0

    def find_positive_inertia_region(self, iq_inc, iq_dec): 
        """ Find the longest region that could generate positive inertia estimation. """
        start_pt_list = []
        end_pt_list = []

        N = len(iq_inc)
        valid_region = False
        for i in range(N): 
            if valid_region and iq_inc[i] < iq_dec[i]: 
                end_pt_list.append(i)
                valid_region = False
            elif (not valid_region) and iq_inc[i] > iq_dec[i]: 
                start_pt_list.append(i)
                valid_region = True
        
        if len(end_pt_list) < len(start_pt_list): 
            end_pt_list.append(N)

        longest_region = 0
        for i in range(len(start_pt_list)): 
            if end_pt_list[i] - start_pt_list[i] > longest_region: 
                longest_region = end_pt_list[i] - start_pt_list[i]
                start_pt = start_pt_list[i]
                end_pt = end_pt_list[i]

        return start_pt, end_pt

    def estimate_J_b_fs_sawtooth(self, omega, iq, K_est, delta_t, bin_length = 32, bin_moving_length = 8, butterworth_corner_freq_ratio=0.05, plot_enable=False, plot_disappear=True): 
        
        omega_max_idx = np.argmax(omega)

        omega_inc = self.butterworth_filter(omega[:omega_max_idx], delta_t, butterworth_corner_freq_ratio)
        omega_dec = self.butterworth_filter(omega[omega_max_idx:][::-1], delta_t, butterworth_corner_freq_ratio)
        
        iq_inc = self.butterworth_filter(iq[:omega_max_idx], delta_t, butterworth_corner_freq_ratio)
        iq_dec = self.butterworth_filter(iq[omega_max_idx:][::-1], delta_t, butterworth_corner_freq_ratio)

        acc_inc = self.get_acceleration(delta_t, omega_inc)
        acc_dec = self.get_acceleration(delta_t, omega_dec)
        
        N = len(omega)
        N_inc = len(acc_inc)
        N_dec = len(acc_dec)

        omega_max = max(omega)
        omega_min = min(omega)
        omega_interpolated_full = omega_inc
        n_samples = N_inc

        iq_inc_interpolated_full = iq_inc
        iq_dec_interpolated_full = []
        iq_averaged_full = []
        for i in range(n_samples): 
            iq_dec_interpolated_full.append(self.linear_interpolator(omega_dec, iq_dec, omega_interpolated_full[i]))
            iq_averaged_full.append((iq_inc_interpolated_full[i] + iq_dec_interpolated_full[i]) / 2)
        iq_inc_interpolated_full = np.array(iq_inc_interpolated_full)
        iq_dec_interpolated_full = np.array(iq_dec_interpolated_full)
        iq_averaged_full = np.array(iq_averaged_full)

        valid_start_pt, valid_end_pt = self.find_positive_inertia_region(iq_inc_interpolated_full, iq_dec_interpolated_full)
        iq_inc_interpolated = iq_inc_interpolated_full[valid_start_pt:valid_end_pt]
        iq_dec_interpolated = iq_dec_interpolated_full[valid_start_pt:valid_end_pt]
        omega_interpolated = omega_interpolated_full[valid_start_pt:valid_end_pt]
        iq_averaged = iq_averaged_full[valid_start_pt:valid_end_pt]
        n_samples = valid_end_pt - valid_start_pt
        print(f"valid positive points for velocity loop systemID are from {valid_start_pt} to {valid_end_pt}. ")

        acc_interpolated = self.get_acceleration(delta_t, omega_interpolated)
        acc_dec_interpolated = []
        for i in range(n_samples): 
            acc_dec_interpolated.append(self.linear_interpolator(omega_dec, acc_dec, omega_interpolated[i]))


        b_est, fs_est = None, None
        bin_length = min(bin_length, n_samples)
        n_bins = int(n_samples / bin_length)
        bin_moving_length = min(bin_moving_length, bin_length)

        acc_interval_std = [np.std(acc_interpolated[i*bin_length : (i+1)*bin_length]) for i in range(n_bins)] + [np.std(acc_dec_interpolated[i*bin_length : (i+1)*bin_length]) for i in range(n_bins)]
        acc_interval_diff = [abs(np.mean(acc_interpolated[i*bin_length:(i+1)*bin_length]) - np.mean(acc_dec_interpolated[i*bin_length:(i+1)*bin_length])) for i in range(n_bins)]

        min_acc_std = min(acc_interval_std)
        max_acc_std = max(acc_interval_std)
        range_acc_std = max_acc_std - min_acc_std
        acc_std_ref = min(np.mean(acc_interval_std), np.median(acc_interval_std), min_acc_std*10)
        min_acc_diff = min(acc_interval_diff)
        acc_diff_ref = min(np.mean(acc_interval_diff), np.median(acc_interval_diff), min_acc_diff*10)
        
        # print(f"acc interval std is {acc_interval_std}")
        # print(f"acc interval diff is {acc_interval_diff}")
        # print(f"min std is {min_acc_std}, max std is {max_acc_std}, mean std is {np.mean(acc_interval_std)}, median is {np.median(acc_interval_std)}")
        # print(f"reference std is {acc_std_ref}, diff is {acc_diff_ref}. ")
        
        # print("start initial search")
        start_pt, end_pt = -1, -1
        bin_length = int(bin_length / 2) 
        while start_pt < 0 and bin_length < n_samples / 2: 
            bin_length = int(bin_length * 2)
            # print(f"length of bin is {bin_length}")
            n_bins = int(n_samples / bin_length)
            
            acc_interval_std = [np.std(acc_interpolated[i*bin_length : (i+1)*bin_length]) for i in range(n_bins)] + [np.std(acc_dec_interpolated[i*bin_length : (i+1)*bin_length]) for i in range(n_bins)]
            acc_interval_diff = [abs(np.mean(acc_interpolated[i*bin_length:(i+1)*bin_length]) - np.mean(acc_dec_interpolated[i*bin_length:(i+1)*bin_length])) for i in range(n_bins)]

            acc_std_ref = min(np.mean(acc_interval_std), np.median(acc_interval_std), min_acc_std*10)
            acc_diff_ref = min(np.mean(acc_interval_diff), np.median(acc_interval_diff), min_acc_diff*10)
            
            min_acc_mean = np.mean(acc_interpolated[:bin_length])
            for i in range(0, n_samples - bin_length, bin_moving_length): 
                i_std = (np.std(acc_interpolated[i:i+bin_length]) + np.std(acc_dec_interpolated[i:i+bin_length])) / 2
                i_mean_diff = abs(np.mean(acc_interpolated[i:i+bin_length]) - np.mean(acc_dec_interpolated[i:i+bin_length]))
                # print(f"tmp start pt {i}, tmp end pt {i+bin_length}")
                # print(f"curr std is {i_std}, curr mean diff is {i_mean_diff}")
                if i_std <= acc_std_ref and i_mean_diff < min_acc_mean: 
                    start_pt_tmp = i
                    end_pt_tmp = i + bin_length
                    slope, intercept = self.linear_fit_BFGS(omega_interpolated[start_pt_tmp:end_pt_tmp], iq_averaged[start_pt_tmp:end_pt_tmp])
                    if slope > 0 and intercept > 0: 
                        min_acc_mean = i_mean_diff
                        b_est, fs_est = slope * (3/2*K_est), intercept * (3/2*K_est)
                        start_pt, end_pt = start_pt_tmp, end_pt_tmp
                        acc_eval = (np.mean(acc_interpolated[start_pt:end_pt]) + np.mean(acc_dec_interpolated[start_pt:end_pt])) / 2
                        # print(f"initial search, b = {b_est}, fs = {fs_est}, start pt is {start_pt}, end pt is {end_pt}. ")

        if start_pt == -1: 
            start_pt, end_pt = int(n_samples / 2 - bin_length / 2), int(n_samples / 2 + bin_length / 2) + 1

        while start_pt > bin_moving_length: 
            start_pt_tmp = start_pt - bin_moving_length
            end_pt_tmp = end_pt
            curr_std = (np.std(acc_interpolated[start_pt_tmp:start_pt_tmp+bin_length]) + np.std(acc_dec_interpolated[start_pt_tmp:start_pt_tmp+bin_length])) / 2
            curr_diff = abs(np.mean(acc_interpolated[start_pt_tmp:start_pt_tmp+bin_length]) - np.mean(acc_dec_interpolated[start_pt_tmp:start_pt_tmp+bin_length]))
            # print(f"start pt is {start_pt_tmp}, end pt is {end_pt_tmp}, std is {curr_std}")
            # print(f"curr std is {curr_std}, curr diff is {curr_diff}")
            if  curr_std <= acc_std_ref and curr_diff <= acc_diff_ref: 
                slope, intercept = self.linear_fit_BFGS(omega_interpolated[start_pt_tmp:end_pt_tmp], iq_averaged[start_pt_tmp:end_pt_tmp])
                if slope > 0 and intercept > 0: 
                    b_est, fs_est = slope * (3/2*K_est), intercept * (3/2*K_est)
                    start_pt, end_pt = start_pt_tmp, end_pt_tmp
                    acc_eval = (np.mean(acc_interpolated[start_pt:end_pt]) + np.mean(acc_dec_interpolated[start_pt:end_pt])) / 2
                    # print(f"start bin search, b = {b_est}, fs = {fs_est}, start pt is {start_pt}, end pt is {end_pt}. ")
                else: 
                    break
            else: 
                break
        # print("end pt search")
        while end_pt < n_samples - bin_moving_length: 
            start_pt_tmp = start_pt
            end_pt_tmp = end_pt + bin_moving_length
            curr_std = (np.std(acc_interpolated[end_pt_tmp-bin_length:end_pt_tmp]) + np.std(acc_dec_interpolated[end_pt_tmp-bin_length:end_pt_tmp])) / 2
            curr_diff = abs(np.mean(acc_interpolated[end_pt_tmp-bin_length:end_pt_tmp]) - np.mean(acc_dec_interpolated[end_pt_tmp-bin_length:end_pt_tmp]))
            # print(f"curr std is {curr_std}, curr diff is {curr_diff}")
            # print(f"start pt is {start_pt_tmp}, end pt is {end_pt_tmp}, std is {curr_std}")
            if curr_std <= acc_std_ref and curr_diff < acc_diff_ref: 
                slope, intercept = self.linear_fit_BFGS(omega_interpolated[start_pt_tmp:end_pt_tmp], iq_averaged[start_pt_tmp:end_pt_tmp])
                if slope > 0 and intercept > 0: 
                    b_est, fs_est = slope * (3/2*K_est), intercept * (3/2*K_est)
                    start_pt, end_pt = start_pt_tmp, end_pt_tmp
                    acc_eval = (np.mean(acc_interpolated[start_pt:end_pt]) + np.mean(acc_dec_interpolated[start_pt:end_pt])) / 2
                    # print(f"end bin search, b = {b_est}, fs = {fs_est}, start pt is {start_pt}, end pt is {end_pt}. ")
                else: 
                    break
            else: 
                break
        if b_est is None or fs_est is None: 
            # raise Exception('#### WARN: cannot generate proper estimation of friction, the data might be too noisy. ')
            print('#### WARN: cannot generate proper estimation of friction, the data might be too noisy. ')
            b_est = -1.e-5
            fs_est = -1.e-5
            start_pt, end_pt = int(n_samples/2-bin_length/2), int(n_samples/2+bin_length/2)
            acc_eval = (np.mean(acc_interpolated[start_pt:end_pt]) + np.mean(acc_dec_interpolated[start_pt:end_pt])) / 2

        if plot_enable: 
            fig, (a0, a1, a2) = plt.subplots(3, 1, gridspec_kw={'height_ratios': [1, 1, 1]}, figsize=(16, 16))
            a0.plot(omega_inc, acc_inc, label='inc', color='cornflowerblue')
            a0.plot(omega_dec, acc_dec, label='dec', color='lightgray')
            a0.plot(omega_interpolated, acc_dec_interpolated, '--', label='dec interpolated', color='tomato')
            a0.axvline(x = omega_interpolated[start_pt], color='black')
            a0.axvline(x = omega_interpolated[end_pt], color='black')
            a0.set_xlabel("$\omega (rad/s)$")
            a0.set_ylabel("$acceleration (rad / s^2)$")
            a0.legend()

            a1.plot([i for i in range(len(omega_inc))], omega_inc, color='cornflowerblue', label='inc')
            a1.plot([i + (len(omega_inc) - len(omega_dec)) for i in range(len(omega_dec))], omega_dec, color='tomato', label='dec')
            a1.axvline(x = valid_start_pt + start_pt, color='black')
            a1.axvline(x = valid_start_pt + end_pt, color='black')
            a1.legend()
            a1.set_ylabel('$\omega (rad / s)$')
            a1.set_xlabel("sample #")

            a2.plot(omega_inc, iq_inc, '.', label='inc', color='cornflowerblue')
            a2.plot(omega_dec, iq_dec, '.', label='dec', color='lightgray')
            a2.plot(omega_interpolated_full, iq_dec_interpolated_full, '.', label='dec interpolated', color='tomato')
            a2.plot(omega_interpolated_full, iq_averaged_full, '.', color='black', label='averaged')
            a2.axvline(x = omega_interpolated[start_pt], color='black')
            a2.axvline(x = omega_interpolated[end_pt], color='black')
            a2.axvline(x = omega[valid_start_pt], color='lightgray')
            a2.axvline(x = omega[valid_end_pt], color='lightgray')
            a2.legend()
            a2.set_xlabel('$\omega (rad / s)$')
            a2.set_ylabel('$i_q (A)$')

            if plot_disappear: 
                plt.show(block=False)
                plt.pause(5)
                plt.close()
            else: 
                plt.show()
        
        # Estimate J
        slope, intercept = b_est / (3/2*K_est), fs_est / (3/2*K_est)

        omega_1, omega_2 = omega_interpolated[start_pt:end_pt], omega_interpolated[start_pt:end_pt]
        iq_1, iq_2 = iq_inc_interpolated[start_pt:end_pt], iq_dec_interpolated[start_pt:end_pt]

        def func(params): 
            intercept_diff = params[0]
            predicted_y1 = slope * omega_1 + (intercept + intercept_diff / 2)
            predicted_y2 = slope * omega_2 + (intercept - intercept_diff / 2)
            errors = np.concatenate((iq_1 - predicted_y1, iq_2 - predicted_y2))

            return np.sum(errors ** 2)

        intercept_diff_est = self.minimize_BFGS(func, [0])[0]

        J_est = (3/2) * K_est * intercept_diff_est / (2 * acc_eval)

        def linear_fit(input_data, coeffs): 
            output = input_data * coeffs[0] + coeffs[1]
            return output

        fitted_1 = linear_fit(omega_1, [slope, intercept + intercept_diff_est / 2])
        fitted_2 = linear_fit(omega_2, [slope, intercept - intercept_diff_est / 2])

        if plot_enable:
            plt.figure()
            plt.plot(omega_interpolated_full, iq_inc_interpolated_full, color='cornflowerblue')
            plt.plot(omega_interpolated_full, iq_dec_interpolated_full, color='mediumseagreen')
            plt.plot(omega_1, fitted_1, color='red')
            plt.plot(omega_2, fitted_2, color='orange')
            plt.xlabel('$\omega$ (rad/s)')
            plt.ylabel('$i_q$ (A)')
            if plot_disappear:
                plt.show(block=False)
            else:
                plt.show()
            print("##### DONE WITH PLOT ########")

            if plot_disappear:
                plt.pause(5)
                plt.close()

        return J_est, b_est, fs_est


    def linear_fit_BFGS(self, x, y): 

        def func_linear(params): 
            slope, intercept = params
            predicted_y = slope * x + intercept
            errors = y - predicted_y

            return np.sum(errors ** 2)

        slope, intercept = self.minimize_BFGS(func_linear, [0, 0])

        return slope, intercept

##################################################### Signal Correlation Delay Measurement Test Code ##################

    def estimate_R_L_delay_corr(self, data, input_name, output_name, period=None, max_delay=4): 
        """ This function estimates motors's resistance, inductance and signal delay time at the same time using RLS. Only works for small sampling rate. """
        in_mat = np.identity(int(max_delay)+2)
        # in_mat = np.identity(int(max_delay)+1)
        out_mat = np.identity(1)
        order = int(np.sum(in_mat) + np.sum(out_mat))

        constraint = np.zeros(order)
        constraint_val = np.zeros(order)
        for i in range(1, len(constraint)): 
            constraint[i] = 1
            constraint_val[i] = 0

        input_filter_wn = 0
        output_filter_wn = 0.2
        coeffs, corr_delay_ts = self.SystemID_Sdomain_RLS_Corr(data, input_name, output_name, in_mat, out_mat, constraint, constraint_val, input_filter_wn, output_filter_wn, period)

        if (period):
            delta_t = period
        else:
            if 'timestamp' in data: 
                t = np.array([value for value in data['timestamp']])
            else: 
                print("System ID Motor signal timestamp not found!")
                exit(1)
            delta_t = t[1] - t[0]

        coeffs_sum = sum(coeffs[1:])
        delay_ts = 0  # Defined as delay time / sampling time
        for j in range(1, len(coeffs)): 
            delay_ts += (j-1)*coeffs[j]
        delay_ts /= coeffs_sum

        R_est = (1 - coeffs[0]) / coeffs_sum
        L_est = delta_t / coeffs_sum * (coeffs[0] + 1) / 2

        delay = delay_ts * delta_t

        # Estimate 3dB Corner Frequency.
        Tau = L_est/R_est
        self.freq3dB = 1/(2*math.pi*Tau)

        #  delay_ts = delay_ts - 1.0 # NOTE: This correction of 1 cycle is added because comparisons versus the simulation model suggests that there is one less cycle in the on-chip closed loop response than the RAMDebug data suggests.
        delay_ts = 1.0 # Based on the Nov 30th conversation with Onno, this is the expected closed loop delay for the TM01 ( tweaked from 1.0 to 1.25 cycles ).
        return R_est, L_est, delay_ts, corr_delay_ts
                  
    def corr_lag_finder(self,y1, y2, period):
        sr = 1/period
        print("#")
        print("# - Cross Correlation Test.")
        print("#")
        print("Cross-Correlation Sample Rate = ", sr, " Hz.")
        n = len(y1)

        corr = signal.correlate(y2, y1, mode='same') / np.sqrt(signal.correlate(y1, y1, mode='same')[int(n/2)] * signal.correlate(y2, y2, mode='same')[int(n/2)])

        delay_arr = np.linspace(-0.5*n/sr, 0.5*n/sr, n)
        delay = delay_arr[np.argmax(corr)]
        print('# y2 is ' + str(delay) + ' secs behind y1')
        delay_ts = delay/period
        print('# y2 is ' + str(delay_ts) + ' cycles behind y1')
        print("#")
        print("#")

        # plt.figure()
        # plt.plot(delay_arr, corr)
        # plt.title('Lag: ' + str(np.round(delay, 3)) + ' s')
        # plt.xlabel('Lag')
        # plt.ylabel('Correlation coeff')
        # plt.show()

        return delay_ts


    def SystemID_Sdomain_RLS_Corr(self, data, input_name, output_name, in_mat, out_mat, constraint, constraint_val, input_filter_wn, output_filter_wn, period=None): 
        """ This systemID function works for first or second order transfer function. """

        # print("# UD/UQ Voltage Error = ",self.TM01Setup.get_offset(input_name),"V.")
        if input_name in data:
            key = input_name
            tfin = np.array([self.TM01Setup.get_SI_value(key,value) for value in data[key]])
            # tfin = np.array([value*self.TM01Setup.get_scaling(key) for value in data[key]])
        else:
            print("System ID Motor input name not found!")
            exit(1)

        if output_name in data:
            key = output_name
            tfout = np.array([self.TM01Setup.get_SI_value(key, value) for value in data[key]])
        else:
            print("System ID Motor output name not found!")
            exit(1)

        if (period):
            delta_t = period
            t = np.linspace(0, delta_t*len(tfin), len(tfin))
        else:
            if 'timestamp' in data: 
                t = np.array([value for value in data['timestamp']])
            else: 
                print("System ID Motor signal timestamp not found!")
                exit(1)
            delta_t = t[1] - t[0]

        # Filter both the input and the output signal.
        tfin_raw = tfin
        tfout_raw = tfout

        if input_filter_wn:
            tfin = self.butterworth_filter(tfin_raw, delta_t, input_filter_wn)
 
        if output_filter_wn: 
            tfout = self.butterworth_filter(tfout_raw, delta_t, output_filter_wn)

        corr_input_filter_wn = 0.25
        tfin_corr = tfin_raw # self.butterworth_filter(tfin_raw, delta_t, corr_input_filter_wn)
        corr_output_filter_wn = 0.25
        tfout_corr = self.butterworth_filter(tfout_raw, delta_t, corr_output_filter_wn)
        corr_delay_ts = self.corr_lag_finder(tfin_corr, tfout_corr, period)

        # gaussian = self.gaussiankernel(5,2)
        # tfin  = self.gaussianconvolve(tfin_raw, gaussian)
        # tfout = self.gaussianconvolve(tfout_raw, gaussian)

        n_sample = len(tfin)
        sigma_to_noise = 1. / 0.03 # The previous noise estimation only works for step response. Use the noise estimated from step signal here. 
        epsilon = 0

        # initialize recursive vector
        in_order = int(np.sum(in_mat))  # Sum up all elements in the 2d array. 
        out_order = int(np.sum(out_mat))  # Sum up all elements in the 2d array. 
        order = in_order + out_order  # Sum up all elements in the 2d array as the order of the RLS loop. 

        coeffs = np.zeros(order)
        c_lambda = 1
        delta = 1 / delta_t / sigma_to_noise
        Q = np.identity(order)*delta

        # recursively update
        # for i in range(max(in_order-2, out_order-1, 0), n_sample-1): 
        for i in range(n_sample-1): 

            Re = []
            for i_out_order in range(out_order):
                out_vec = np.array([tfout[i-j] if i >= j else 0 for j in range(len(out_mat[0]))])
                Re.append(np.dot(out_mat[i_out_order].T, out_vec))
            for i_in_order in range(in_order):
                in_vec = np.array([tfin[i-j] if i >= j else 0 for j in range(-1, len(in_mat[0])-1)])
                Re.append(np.dot(in_mat[i_in_order].T, in_vec))
            Re = np.array(Re)

            phi = np.dot(Q.T, Re)

            phiTimesR = np.dot(phi, Re)
            gamma = phi / (phiTimesR + c_lambda)

            id_est = np.dot(coeffs, Re)
            err = tfout[i+1] - id_est

            coeffs += err * gamma

            # Apply constraints
            for j in range(order): 
                if not constraint[j] == 0:
                    if constraint[j] == 1: 
                        if coeffs[j] < constraint_val[j]: 
                            e = np.zeros(order).T
                            e[j] = 1
                            coeffs_update = np.matmul(Q, e) / (np.matmul(np.matmul(e.T, Q), e)) * (0 - np.matmul(e.T, np.array([coeffs]).T) + epsilon)
                            coeffs += coeffs_update.T.squeeze()
                    elif constraint[j] == -1: 
                        if coeffs[j] > constraint_val[j]: 
                            e = np.zeros(order).T
                            e[j] = 1
                            coeffs_update = np.matmul(Q, e) / (np.matmul(np.matmul(e.T, Q), e)) * (0 - np.matmul(e.T, np.array([coeffs]).T) - epsilon)
                            coeffs += coeffs_update.T.squeeze()

            Q = (Q - np.outer(gamma, phi)) / c_lambda

        return coeffs, corr_delay_ts

##################################################### End of Signal Correlation Delay Measurement Test Code ##################

    def estimate_R_L_delay(self, data, input_name, output_name, period=None, max_delay=5): 
        """ This function estimates motors's resistance, inductance and signal delay time at the same time using RLS. Only works for small sampling rate. """
        # in_mat = np.identity(int(max_delay)+2)
        in_mat = np.identity(int(max_delay)+1)
        out_mat = np.identity(1)
        order = int(np.sum(in_mat) + np.sum(out_mat))

        constraint = np.zeros(order)
        constraint_val = np.zeros(order)
        for i in range(1, len(constraint)): 
            constraint[i] = 1
            constraint_val[i] = 0

        input_filter_wn = 0
        output_filter_wn = 0.2
        coeffs = self.SystemID_Sdomain_RLS(data, input_name, output_name, in_mat, out_mat, constraint, constraint_val, input_filter_wn, output_filter_wn, period)

        if (period):
            delta_t = period
        else:
            if 'timestamp' in data: 
                t = np.array([value for value in data['timestamp']])
            else: 
                print("System ID Motor signal timestamp not found!")
                exit(1)
            delta_t = t[1] - t[0]

        coeffs_sum = sum(coeffs[1:])
        delay_ts = 0  # Defined as delay time / sampling time
        for j in range(1, len(coeffs)): 
            delay_ts += (j-1)*coeffs[j]
        delay_ts /= coeffs_sum

        R_est = (1 - coeffs[0]) / coeffs_sum
        L_est = delta_t / coeffs_sum * (coeffs[0] + 1) / 2

        delay = delay_ts * delta_t

        # Estimate 3dB Corner Frequency.
        Tau = L_est/R_est
        self.freq3dB = 1/(2*math.pi*Tau)

        # delay_ts = delay_ts - 1.0 # NOTE: This correction of 1 cycle is added because comparisons versus the simulation model suggests that there is one less cycle in the on-chip closed loop response than the RAMDebug data suggests.
        delay_ts = 1.00 # Based on the Nov 30th conversation with Onno, this is the expected closed loop delay for the TM01 ( tweaked from 1.0 to 1.25 cycles ).
        return R_est, L_est, delay_ts
                
    def estimate_R_L(self, data, input_name, output_name, period=None): 
        """ This function estimates motors's resistance and inductance using RLS. """
        in_mat = np.array([[0, 1]])    # Fix the delay as 1 cycle
        out_mat = np.identity(1)
        order = int(np.sum(in_mat) + np.sum(out_mat))

        constraint = np.zeros(order)
        constraint_val = np.zeros(order)
        for i in range(1, len(constraint)): 
            constraint[i] = 1
            constraint_val[i] = 0

        input_filter_wn = 0
        output_filter_wn = 0.2
        coeffs = self.SystemID_Sdomain_RLS(data, input_name, output_name, in_mat, out_mat, constraint, constraint_val, input_filter_wn, output_filter_wn, period)

        if (period):
            delta_t = period
        else:
            if 'timestamp' in data: 
                t = np.array([value for value in data['timestamp']])
            else: 
                print("System ID Motor signal timestamp not found!")
                exit(1)
            delta_t = t[1] - t[0]

        R_est = (1 - coeffs[0]) / coeffs[1]
        L_est = delta_t / coeffs[1] * (coeffs[0] + 1) / 2

        """
        if self.TM01Setup.MotorType == "Stepper":
            R_est = R_est
            L_est = L_est
        elif self.TM01Setup.MotorType == "BLDC":
            bldc_phase_deg = 30
            bldc_phase_rad = (bldc_phase_deg/180)*math.pi
            R_est = R_est / math.tan(bldc_phase_rad) # Correction for 3-phase motor config. NOTE: Delta Vs Star configurations may be different.
            L_est = L_est / math.tan(bldc_phase_rad)
        elif self.TM01Setup.MotorType == "PANDrive":
            R_est = R_est
            L_est = L_est
        else:
            R_est = R_est
            L_est = L_est
        """
        
        # Estimate 3dB Corner Frequency.
        Tau = L_est/R_est
        self.freq3dB = 1/(2*math.pi*Tau)

        Tdelay = 1.0 # Based on the Nov 30th conversation with Onno, this is the expected closed loop delay for the TM01 ( tweaked from 1.0 to 1.25 cycles ).
        return R_est, L_est, Tdelay
                
    def extract_1st_order_tf_from_RLS(self, data, input_name, output_name, period=None): 
        # in_mat = np.array([[1/2, 1/2]])
        in_mat = np.identity(1)
        out_mat = np.identity(1)
        order = int(np.sum(in_mat) + np.sum(out_mat))
        constraint = np.zeros(order)
        constraint_val = np.zeros(order)

        # input_name = "MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL"
        # output_name = "MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL"
        input_filter_wn = 0.05
        output_filter_wn = 0.1
        coeffs = self.SystemID_Sdomain_RLS(data, input_name, output_name, in_mat, out_mat, constraint, constraint_val, input_filter_wn, output_filter_wn, period)

        if (period):
            delta_t = period
        else:
            if 'timestamp' in data: 
                t = np.array([value for value in data['timestamp']])
            else: 
                print("System ID Motor signal timestamp not found!")
                exit(1)
            delta_t = t[1] - t[0]

        gain = coeffs[1] / (1 - coeffs[0])
        Tau = delta_t / coeffs[1] * (coeffs[0] + 1) / 2 * coeffs[1] / (1 - coeffs[0])

        return gain, Tau


    def estimate_VelRMSRatio(self, data, period=None): 
        """ This systemID function works for motor constant using step response data. """

        if (period):
            delta_t = period
        else:
            if 'timestamp' in data: 
                t = np.array([value for value in data['timestamp']])
            else: 
                print("System ID Motor signal timestamp not found!")
                exit(1)
            delta_t = t[1] - t[0]

        if 'MCC.PIDIN_VELOCITY_TARGET.PIDIN_VELOCITY_TARGET' in data:
            key = 'MCC.PIDIN_VELOCITY_TARGET.PIDIN_VELOCITY_TARGET'
            velocity_target = np.array([self.TM01Setup.get_SI_value(key,value) for value in data[key]])
        else:
            print("System ID Motor velocity name not found!")
            exit(1)

        if 'MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL' in data:
            key = 'MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL'
            velocity_actual = np.array([self.TM01Setup.get_SI_value(key,value) for value in data[key]])
        else:
            print("System ID Motor velocity name not found!")
            exit(1)

        vel_array_len = len(velocity_target)
        vel_target_rms = np.sum(velocity_target ** 2)/vel_array_len
        vel_actual_rms = np.sum(velocity_actual ** 2)/vel_array_len

        vel_rms_error = (vel_target_rms-vel_actual_rms)/vel_target_rms
        return vel_rms_error


    def estimate_VelPeakRatio(self, data, period=None): 
        """ This systemID function works for motor constant using step response data. """

        if (period):
            delta_t = period
        else:
            if 'timestamp' in data: 
                t = np.array([value for value in data['timestamp']])
            else: 
                print("System ID Motor signal timestamp not found!")
                exit(1)
            delta_t = t[1] - t[0]

        if 'MCC.PIDIN_VELOCITY_TARGET.PIDIN_VELOCITY_TARGET' in data:
            key = 'MCC.PIDIN_VELOCITY_TARGET.PIDIN_VELOCITY_TARGET'
            velocity_target = np.array([self.TM01Setup.get_SI_value(key,value) for value in data[key]])
        else:
            print("System ID Motor velocity name not found!")
            exit(1)

        if 'MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL' in data:
            key = 'MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL'
            velocity_actual = np.array([self.TM01Setup.get_SI_value(key,value) for value in data[key]])
        else:
            print("System ID Motor velocity name not found!")
            exit(1)

        vel_target_peak = max(velocity_target)
        vel_actual_peak = max(velocity_actual)
        
        vel_peak_ratio = vel_actual_peak/vel_target_peak
        return vel_peak_ratio




    def estimate_K(self, data, R_est, period=None): 
        """ This systemID function works for motor constant using step response data. """

        if (period):
            delta_t = period
        else:
            if 'timestamp' in data: 
                t = np.array([value for value in data['timestamp']])
            else: 
                print("System ID Motor signal timestamp not found!")
                exit(1)
            delta_t = t[1] - t[0]

        if 'MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL' in data:
            key = 'MCC.PID_VELOCITY_ACTUAL.PID_VELOCITY_ACTUAL'
            velocity = np.array([self.TM01Setup.get_SI_value(key,value) for value in data[key]])
        else:
            print("System ID Motor velocity name not found!")
            exit(1)

        if 'MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL' in data:
            key = 'MCC.PID_TORQUE_FLUX_ACTUAL.PID_TORQUE_ACTUAL'
            iq_actual = np.array([self.TM01Setup.get_SI_value(key, value) for value in data[key]])
        elif 'MCC.PIDIN_TORQUE_FLUX_TARGET.PIDIN_TORQUE_TARGET' in data:
            key = 'MCC.PIDIN_TORQUE_FLUX_TARGET.PIDIN_TORQUE_TARGET'
            iq_actual = np.array([self.TM01Setup.get_SI_value(key, value) for value in data[key]])
        else:
            print("System ID Motor torque name not found!")
            exit(1)

        if 'MCC.FOC_UQ_UD.UQ' in data:
            key = 'MCC.FOC_UQ_UD.UQ'
            uq = np.array([self.TM01Setup.get_SI_value(key, value) for value in data[key]])
        else:
            print("System ID Motor uq name not found!")
            exit(1)


        starting_pt = 100   # The starting point supposed to be after iq saturated. 

        def func_linear(params): 

            slope = params[0]
            predict = slope * velocity[starting_pt:]
            errors = (uq[starting_pt:] - R_est * iq_actual[starting_pt:]) - predict

            return np.sum(errors ** 2)

        slope = self.minimize_BFGS(func_linear, [0])

        K_est = slope[0]

        return K_est
    
    def SystemID_Sdomain_RLS(self, data, input_name, output_name, in_mat, out_mat, constraint, constraint_val, input_filter_wn, output_filter_wn, period=None): 
        """ This systemID function works for first or second order transfer function. """

        # print("# UD/UQ Voltage Error = ",self.TM01Setup.get_offset(input_name),"V.")
        if input_name in data:
            key = input_name
            tfin = np.array([self.TM01Setup.get_SI_value(key,value) for value in data[key]])
            # tfin = np.array([value*self.TM01Setup.get_scaling(key) for value in data[key]])
        else:
            print("System ID Motor input name not found!")
            exit(1)

        if output_name in data:
            key = output_name
            tfout = np.array([self.TM01Setup.get_SI_value(key, value) for value in data[key]])
        else:
            print("System ID Motor output name not found!")
            exit(1)

        if (period):
            delta_t = period
            t = np.linspace(0, delta_t*len(tfin), len(tfin))
        else:
            if 'timestamp' in data: 
                t = np.array([value for value in data['timestamp']])
            else: 
                print("System ID Motor signal timestamp not found!")
                exit(1)
            delta_t = t[1] - t[0]

        # Filter both the input and the output signal.
        tfin_raw = tfin
        tfout_raw = tfout

        if input_filter_wn:
            tfin = self.butterworth_filter(tfin_raw, delta_t, input_filter_wn)
        if output_filter_wn: 
            tfout = self.butterworth_filter(tfout_raw, delta_t, output_filter_wn)

        # gaussian = self.gaussiankernel(5,2)
        # tfin  = self.gaussianconvolve(tfin_raw, gaussian)
        # tfout = self.gaussianconvolve(tfout_raw, gaussian)

        n_sample = len(tfin)
        sigma_to_noise = 1. / 0.03 # The previous noise estimation only works for step response. Use the noise estimated from step signal here. 
        epsilon = 0

        # initialize recursive vector
        in_order = int(np.sum(in_mat))  # Sum up all elements in the 2d array. 
        out_order = int(np.sum(out_mat))  # Sum up all elements in the 2d array. 
        order = in_order + out_order  # Sum up all elements in the 2d array as the order of the RLS loop. 

        coeffs = np.zeros(order)
        c_lambda = 1
        delta = 1 / delta_t / sigma_to_noise

        # recursively update
        n_iters = 10000
        # sum_coeffs = 0
        coeffs_tmp = np.zeros(order)
        for _ in range(n_iters): 
            Q = np.identity(order)*delta

            for i in range(n_sample-1): 

                Re = []
                for i_out_order in range(out_order):
                    out_vec = np.array([tfout[i-j] if i >= j else 0 for j in range(len(out_mat[0]))])
                    Re.append(np.dot(out_mat[i_out_order].T, out_vec))
                for i_in_order in range(in_order):
                    in_vec = np.array([tfin[i-j] if i >= j else 0 for j in range(-1, len(in_mat[0])-1)])
                    Re.append(np.dot(in_mat[i_in_order].T, in_vec))
                Re = np.array(Re)

                phi = np.dot(Q.T, Re)

                phiTimesR = np.dot(phi, Re)
                gamma = phi / (phiTimesR + c_lambda)

                id_est = np.dot(coeffs, Re)
                err = tfout[i+1] - id_est

                coeffs += err * gamma

                # Apply constraints
                for j in range(order): 
                    if not constraint[j] == 0:
                        if constraint[j] == 1: 
                            if coeffs[j] < constraint_val[j]: 
                                e = np.zeros(order).T
                                e[j] = 1
                                coeffs_update = np.matmul(Q, e) / (np.matmul(np.matmul(e.T, Q), e)) * (0 - np.matmul(e.T, np.array([coeffs]).T) + epsilon)
                                coeffs += coeffs_update.T.squeeze()
                        elif constraint[j] == -1: 
                            if coeffs[j] > constraint_val[j]: 
                                e = np.zeros(order).T
                                e[j] = 1
                                coeffs_update = np.matmul(Q, e) / (np.matmul(np.matmul(e.T, Q), e)) * (0 - np.matmul(e.T, np.array([coeffs]).T) - epsilon)
                                coeffs += coeffs_update.T.squeeze()

                Q = (Q - np.outer(gamma, phi)) / c_lambda

            for j in range(order):
                if (not coeffs[j] == 0) and abs(coeffs_tmp[j] - coeffs[j]) / coeffs[j] > 0.0001: 
                    coeffs_tmp = coeffs
                    break
            if j == order - 1: 
                break
            
        return coeffs
    

    def getKpI(self, ki, R, L):
        return ki*(L/R)

    def getWnI(self, Ki, R, tau_d):
        return math.sqrt((2*Ki)/(R*tau_d))

    def getZetaI(self, Ki, R, tau_d):
        return (2*R - Ki*tau_d)/(2*math.sqrt(2*Ki*R*tau_d))

    def estimatePoI(self, zeta):
        if (zeta >= 1.0):
            po = 0
            print(" Warning: zeta_i value >= 1, therefore PO computation is not valid.")
        else:
            po = 100*math.exp( (-1)*( (zeta*math.pi) / ( math.sqrt(1-(zeta**2)) ) ) )
            print(" Estimated Current Closed Loop Percentage Overshoot = ", po, "%.")
        return po



    def calculateCurrentPiCoeffsDC(self, Rest, Lest, Tdelay=1.5, dc_gain=0.75):
        tau_d = Tdelay*self.Tsample # = 1 Tsample delay in PI compute block + 0.5 Tsample delay in PWM Block NOTE: Transport delay from Target to Actual data has been observed at 3 cycles.

        kp_calc = (Rest*dc_gain)/(1-dc_gain)
        ki_calc = kp_calc*Rest/Lest

        self.zeta_est = self.getZetaI(ki_calc, Rest, tau_d)
        self.w_natural = self.getWnI(ki_calc, Rest, tau_d)
        self.freq_natural = self.w_natural/(2*math.pi)
        
        # Scale Coefficients.
        kp_float    = kp_calc
        ki_float    = ki_calc
        ki_float_dt = ki_float * self.Tsample
        print(" Torque/Flux DC Gain Target = ", dc_gain)
        print(" Torque/Flux Computed Kp    = ", kp_float)
        print(" Torque/Flux Computed Ki    = ", ki_float)
        print(" Torque/Flux Computed Ki.dt = ", ki_float_dt)
        print(" Estimated Current Closed Loop Natural Frequency = ", self.freq_natural,"Hz.")

        return kp_float, ki_float, ki_float_dt

    def calculateCurrentPiCoeffsZeta(self, Rest, Lest, zeta_i=1, Tdelay=1.0):
        # Matlab Approach #1: Tune zeta_i directly. NOTE: The system is not exactly a 2nd order LTI system, but it's close.
        tau_d = Tdelay*self.Tsample # = 1 Tsample delay in PI compute block + 0.5 Tsample delay in PWM Block NOTE: Transport delay from Target to Actual data has been observed at 3 cycles.
        
        ki_calc = 1.0      * (2*Rest/tau_d)*(1 + 2*zeta_i**2 - 2*zeta_i*math.sqrt(1 + zeta_i**2))
        kp_calc = 1.0      * self.getKpI(ki_calc, Rest, Lest)

        self.zeta_est = self.getZetaI(ki_calc, Rest, tau_d)
        self.w_natural = self.getWnI(ki_calc, Rest, tau_d)
        self.freq_natural = self.w_natural/(2*math.pi)
        
        # Scale Coefficients.
        kp_float    = kp_calc # / 4.0 # NOTE: Temporary Hack to explore Motor noise and vibration.
        ki_float    = ki_calc # / 4.0
        ki_float_dt = ki_float * self.Tsample
        print(" Torque/Flux Input Zeta_i   = ", zeta_i)
        print(" Torque/Flux Computed Kp    = ", kp_float)
        print(" Torque/Flux Computed Ki    = ", ki_float)
        print(" Torque/Flux Computed Ki.dt = ", ki_float_dt)
        print(" Estimated Current Closed Loop Natural Frequency = ", self.freq_natural,"Hz.")

        return kp_float, ki_float, ki_float_dt

    def calculateCurrentPiCoeffsMatchBW(self, Rest, Lest, Tdelay=1.5):
        # Matlab Approach #2: omega_{G_W - 3dB} of plant = omega_0dB of open-loop. NOTE: Works well, a bit slow w.r.t. approach #1 and a zeta close to 1.
        tau_d = Tdelay*self.Tsample # = 1 Tsample delay in PI compute block + 0.5 Tsample delay in PWM Block NOTE: Transport delay from Target to Actual data has been observed at 3 cycles.
        
        ki_calc = 1.0      * (Rest**2)/Lest
        # ki_calc = (1/Rest) * (Rest**2)/Lest
        kp_calc = 1.0      * self.getKpI(ki_calc, Rest, Lest)

        self.zeta_est = self.getZetaI(ki_calc, Rest, tau_d)
        self.w_natural = self.getWnI(ki_calc, Rest, tau_d)
        self.freq_natural = self.w_natural/(2*math.pi)
        
        # Scale Coefficients.
        kp_float    = kp_calc
        ki_float    = ki_calc
        ki_float_dt = ki_float * self.Tsample
        print(" Torque/Flux Computed Kp    = ", kp_float)
        print(" Torque/Flux Computed Ki    = ", ki_float)
        print(" Torque/Flux Computed Ki.dt = ", ki_float_dt)
        print(" Estimated Current Closed Loop Natural Frequency = ", self.freq_natural,"Hz.")

        return kp_float, ki_float, ki_float_dt


    def calculateCurrentPiCoeffsGmPm(self, Rest, Lest, Tdelay=1.5, Gm=12, Pm=60):
        # Matlab Approach #3: Set the desired gain and phase margins. Choose the minimum Ki_i out of the two constraints so they are both respected.
        tau_d = Tdelay*self.Tsample # = 1 Tsample delay in PI compute block + 0.5 Tsample delay in PWM Block NOTE: Transport delay from Target to Actual data has been observed at 3 cycles.
        
        Gm_i_desired = 10**(Gm/20) # Set the gain margin you want. (unitless, not dB)
        Pm_i_desired = Pm*math.pi/180 # Set the phase margin you want. (radians)
        Ki_i_gm = math.pi*Rest/(2*Gm_i_desired*tau_d)
        Ki_i_pm = (Rest/tau_d)*(math.pi/2 - Pm_i_desired)
        print("# Ki_i based on Gain  Margin = ", Ki_i_gm)
        print("# Ki_i based on Phase Margin = ", Ki_i_pm)
        ki_calc = 1.0      * min(Ki_i_gm, Ki_i_pm)
        # ki_calc = (1/Rest) * min(Ki_i_gm, Ki_i_pm)
        kp_calc = 1.0      * self.getKpI(ki_calc, Rest, Lest)
        
        self.zeta_est = self.getZetaI(ki_calc, Rest, tau_d)
        self.w_natural = self.getWnI(ki_calc, Rest, tau_d)
        self.freq_natural = self.w_natural/(2*math.pi)
        
        
        # Scale Coefficients.
        kp_float    = kp_calc
        ki_float    = ki_calc
        ki_float_dt = ki_float * self.Tsample
        print(" Torque/Flux Computed Kp    = ", kp_float)
        print(" Torque/Flux Computed Ki    = ", ki_calc)
        print(" Torque/Flux Computed Ki.dt = ", ki_float)
        print(" Estimated Current Closed Loop Natural Frequency = ", self.freq_natural,"Hz.")

        return kp_float, ki_float, ki_float_dt


    def calculateCurrentPiCoeffsBWZeta(self, Rest, Lest, zeta_i=1, Tdelay=1.0, target_bw=None):
        # Matlab Approach #1: Tune zeta_i directly. NOTE: The system is not exactly a 2nd order LTI system, but it's close.
        tau_d = Tdelay*self.Tsample # = 1 Tsample delay in PI compute block + PWM Block - From the Trinamic Design Team.
        
        # Modified version of Matlab Approach #1: Tune the loop bandwidth directly and Zeta becomes a dependant variable. The zeta_i is treated as a minimum allowed zeta_i
        # If the computed zeta is below zeta_i, the Method #1 approach is used.

        # Setting the default closed loop bandwidth to be 15 x motor impedence corner and max closed loop bandwidth to be 15 X motor impedence.
        def_bw = self.freq3dB * 20
        max_bw = self.freq3dB * 40

        if target_bw == None:
            bw = def_bw
            print("# Setting Current Loop Bandwidth based on 10 X Motor impedance. Updated Target Bandwidth = ",bw,"Hz.")
        elif target_bw > max_bw:
            bw = max_bw
            print("# Warning: Target BW (",target_bw,"Hz) exceeds maximum recommended Current Loop Bandwidth. Updated Target Bandwidth = ",bw,"Hz.")
        elif target_bw < def_bw:
            bw = def_bw
            print("# Warning: Target BW (",target_bw,"Hz) is below minimum recommended Current Loop Bandwidth. Updated Target Bandwidth = ",bw,"Hz.")
        else:
            bw = target_bw
            print("# Setting Current Loop Bandwidth to ",bw,"Hz.")

        bw_rads = bw*2*math.pi
        ki_calc = (tau_d*Rest*bw_rads*bw_rads)/2
        kp_calc = 1.0      * self.getKpI(ki_calc, Rest, Lest)
       
        self.zeta_est = self.getZetaI(ki_calc, Rest, tau_d)
        self.w_natural = self.getWnI(ki_calc, Rest, tau_d)
        self.freq_natural = self.w_natural/(2*math.pi)
        
        # Scale Coefficients.
        kp_float    = kp_calc
        ki_float    = ki_calc
        ki_float_dt = ki_float * self.Tsample

        print(" Torque/Flux Target Bandwidth = ",bw,"Hz (",bw_rads,"rads/sec).")
        print(" Torque/Flux Effective Zeta   = ", self.zeta_est)
        print(" Torque/Flux Computed Kp      = ", kp_float)
        print(" Torque/Flux Computed Ki      = ", ki_float)
        print(" Torque/Flux Computed Ki.dt   = ", ki_float_dt)
        print(" Estimated Current Closed Loop Natural Frequency = ", self.freq_natural,"Hz.")

        return kp_float, ki_float, ki_float_dt


    def CalculateTorqueFlux_PICoeffs(self, Rest, Lest, zeta_i=1, Tdelay=1.5, use_approach=2, Gm=12, Pm=60, loop_bw=None):
        tau_d = Tdelay*self.Tsample # = 1 Tsample delay in PI compute block + PWM Block NOTE: Transport delay from Target to Actual data has been observed at 3 cycles.
       
        if use_approach == 1:
            print("# Using Torque/Flux Tuning Approach #1.")
            kp_float, ki_float, ki_float_dt = self.calculateCurrentPiCoeffsZeta(Rest, Lest, zeta_i, Tdelay)
            self.po = self.estimatePoI(self.zeta_est)
        elif use_approach == 2:
            print("# Using Torque/Flux Tuning Approach #2.")
            kp_float, ki_float, ki_float_dt = self.calculateCurrentPiCoeffsMatchBW(Rest, Lest, Tdelay)
            self.po = self.estimatePoI(self.zeta_est)
        elif use_approach == 3:
            print("# Using Torque/Flux Tuning Approach #3.")
            kp_float, ki_float, ki_float_dt = self.calculateCurrentPiCoeffsGmPm(Rest, Lest, Tdelay, Gm, Pm)
            self.po = self.estimatePoI(self.zeta_est)
        elif use_approach == 4:
            print("# Using Torque/Flux Tuning Approach #4.")
            kp_float, ki_float, ki_float_dt = self.calculateCurrentPiCoeffsDC(Rest, Lest, Tdelay, dc_gain=0.75 )
            self.po = self.estimatePoI(1.0)
        elif use_approach == 5:
            print("# Using Torque/Flux Tuning Approach #5.")
            kp_float, ki_float, ki_float_dt = self.calculateCurrentPiCoeffsBWZeta(Rest, Lest, zeta_i, Tdelay=1.0, target_bw=None)
            self.po = self.estimatePoI(self.zeta_est)
        else:
            print("Error: Need to specify the correct value of use_approach, must be 1, 2, 3 or 4.")
            exit(1)
        
        return kp_float, ki_float, ki_float_dt
        

    def ScaleTorqueFlux_PICoeffs(self, kp_float, ki_float):

        CoefficientScalingCorrectionFactor = 1.0/1.0 # Included for testing TM01 Hardware Results Versus Matlab Simulation.
        kp_scaling_q8p8, ki_scaling_q8p8, kp_scaling_q0p16, ki_scaling_q0p16 = self.TM01Setup.get_torque_flux_pidcoeffs_scaling()

        kp_int_q8p8  = int( CoefficientScalingCorrectionFactor * kp_float * kp_scaling_q8p8 )
        ki_int_q8p8  = int( CoefficientScalingCorrectionFactor * ki_float * ki_scaling_q8p8 )
        kp_int_q0p16 = int( CoefficientScalingCorrectionFactor * kp_float * kp_scaling_q0p16 )
        ki_int_q0p16 = int( CoefficientScalingCorrectionFactor * ki_float * ki_scaling_q0p16 )

        # Kp_int and Ki_int are limited to 15bits or a maximum value of 32767 - CURRENT_NORM_x = 1 => Q0.16, CURRENT_NORM_x = 0 => Q8.8
        max_pi_int = 32767

        if kp_int_q0p16 > max_pi_int:
            kp_current_int = kp_int_q8p8
            current_norm_p = 0
        else:
            kp_current_int = kp_int_q0p16
            current_norm_p = 1
        
        if ki_int_q0p16 > max_pi_int:
            ki_current_int = ki_int_q8p8
            current_norm_i = 0
        else:
            ki_current_int = ki_int_q0p16
            current_norm_i = 1
        
        print(" TM01 Quantized Current Loop Kp Coefficient   = ", kp_current_int, " bits.")
        print(" TM01 Quantized Current Loop Kp Normalization = ", current_norm_p)
        print(" TM01 Quantized Current Loop Ki Coefficient   = ", ki_current_int, " bits.")
        print(" TM01 Quantized Current Loop Ki Normalization = ", current_norm_i)
        
        return kp_current_int, ki_current_int, current_norm_p, current_norm_i


    def CalculateVelocity_SystemID(self, Kest, gain, tau):
        # Estimate Inertia (J) and Friction (b)
        b = 3*Kest/(2*gain)
        J = b*tau
        print(" Estimated Motor Constant (K) = ", Kest)
        print(" Estimated Inertia (J) = ", J)
        print(" Estimated Friction (b) = ", b)
        return J, b

    def CalculateVelocity_GainTau(self, Kest, Jest, best):
        # Estimate Gain and Tau Parameters.
        gain = 3*Kest/(2*best)
        tau = Jest/best
        print(" Estimated Gain = ", gain)
        print(" Estimated Tau  = ", tau )
        return gain, tau

    def GetMaxVelocity(self, Rest, Lest, Kest, poles, b, Vmax):
        a = (2*b*poles*Lest/(3*Kest))**2
        b = (2*b*Rest/(3*Kest) + Kest)**2
        c = -Vmax**2
        delta = b**2 - 4*a*c
        x = (-b + math.sqrt(delta))/(2*a)
        omega_max = math.sqrt(x)

        return omega_max


    def CalculateVelocity_PICoeffs(self, K_est, J_est, b_est, zeta_v=1.65, zeta_i=1.0):
        tau_d = 3.0*self.Tsample # = 1 Tsample delay in PI compute block + 0.5 Tsample delay in PWM Block.
        w_n   = 1/(2*zeta_i*tau_d)
        tau_i = 2*zeta_i/w_n # 2nd Order Current loop to 1st Order Approximation (1/(tau_i*s + 1))
        
        tau_v = 0 # 8.5*self.Tsample # Estimate Velocity Loop Delay - 1.5 Torque Loop, 4 in moving average filter, 1 in vel measurement(in frequency mode, this is highly variable in period mode), 1 in velocity biquad, 1 in vel pi control. 

        kp_calc = (J_est/(6*zeta_v*zeta_v*K_est*tau_i))
        ki_calc = (b_est/(6*zeta_v*zeta_v*K_est*tau_i))

        kp_float = kp_calc
        ki_float = ki_calc 
        ki_float_dt = ki_float * self.Tsample
        print(" Computed Velocity Kp    = ", kp_float)
        print(" Computed Velocity_Ki    = ", ki_float)
        print(" Computed Velocity_Ki.dt = ", ki_float_dt)

        return kp_float, ki_float, ki_float_dt
 

    def calculateVelocityPiCoeffsDC(self, Rest, Lest, K_est, J_est, b_est, dc_gain=0.75, Kp_v_init=None):
        
        if 0.5 <= dc_gain <= 0.99:
            dc_gain = dc_gain
            print("# Using Velocity PID Target input DC Gain of ",dc_gain)
        else:
            dc_gain = 0.75
            print("# WARNING: Input Velocity PID Target DC Gain is out of range. Using ",dc_gain)

        wn_mech = math.sqrt((2*Rest*b_est + 3*K_est**2)/(2*Lest*J_est))
        fn_vf = wn_mech/(2*math.pi)

        if Kp_v_init:
            kp_calc = Kp_v_init
            ki_calc = (Kp_v_init*b_est)/J_est
        else:
            kp_calc = ((2*b_est)/(3*K_est))*(1/(1-dc_gain))
            ki_calc = (kp_calc*b_est)/J_est  # Mechanical Plant Pole Cancellation.

        kp_float    = kp_calc
        ki_float    = ki_calc
        ki_float_dt = ki_calc * self.Tsample

        print("Velocity Mechanical Bandwidth = ", wn_mech, " rads/sec - ", fn_vf, " Hz.")
        print("Velocity DC Gain Limited Kp    =  ",kp_float )
        print("Velocity DC Gain Limited Ki    =  ",ki_float )
        print("Velocity DC Gain Limited Ki.dt =  ",ki_float_dt )

        return fn_vf, kp_float, ki_float, ki_float_dt


    def get_freq_of_plant_v_at_gain_using_binary_search(self, gain_in_dB, gain_pv, num_pv, den_pv):

        # Make sure the numerator is 2nd order and denominator 4th order
        assert len(num_pv) == 3, "num_py array length should be 3 (2nd Order)."
        assert len(den_pv) == 5, "den_pv array length should be 5 (4th Order)."

        # convert gain from dB to unitless
        gain_target = 10**(gain_in_dB/20)

        # Setup some utility variables
        a_npv = num_pv[1]
        b_npv = num_pv[0]
        a_dpv = den_pv[3]
        b_dpv = den_pv[2]
        c_dpv = den_pv[1]
        d_dpv = den_pv[0]

        # Setup the search bounds and precision
        w_lo = 1e0
        w_hi = 1e9
        eps = 1e-3
        w = (w_hi + w_lo)/2
        gain = gain_pv*math.sqrt(((a_npv*w)**2 + (1 - b_npv*w**2)**2)/((a_dpv*w - c_dpv*w**3)**2 + (1 - b_dpv*w**2 + d_dpv*w**4)**2))

        # Binary search for the frequency
        while (gain - gain_target)**2 > eps**2:
            if gain > gain_target:
                w_lo = w
            else:
                w_hi = w
            w = (w_hi + w_lo)/2
            gain = gain_pv*math.sqrt(((a_npv*w)**2 + (1 - b_npv*w**2)**2)/((a_dpv*w - c_dpv*w**3)**2 + (1 - b_dpv*w**2 + d_dpv*w**4)**2))
        
        return w


    def poly_eval(self, poly, x):
        val = 0
        n = len(poly)
        for i in range(n):
            val = val + poly[i]*x**(n - (i+1))
        return val


    # --------------------------------------------------
    def get_qr_decomposition_householder(self, matrix_A):
        """
        Function to compute QR decomposition of matrix A
                A = Q R, where  Q is an orthonormal matrix (Q.T Q = I)
                                R is an upper triangular matrix
        Usage   :   matrix_Q, matrix_R = get_qr_decomposition(matrix_A)
        Inputs  :   1. matrix_A : Square matrix A
        Outputs :   1. matrix_Q : Orthonormal matrix Q
                    2. matrix_R : Upper triangular matrix R
        """
        epsilon_threshold = 1e-10
        m = matrix_A.shape[0]
        n = matrix_A.shape[1]
        matrix_R = matrix_A.copy()
        matrix_Q = np.identity(m)

        def get_householder_reflector(matrix_R, column_number):
            e_vector = np.zeros((m,1))
            e_vector[column_iter] = 1
            vector = np.zeros((m,1))
            vector[column_iter:, 0] = matrix_R[column_iter:, column_iter].copy()
            if np.sign(vector[column_iter,0]) == 0:
                sign_vector = 1
            else:
                sign_vector = np.sign(vector[column_iter,0])
            u_vector = vector - sign_vector*np.linalg.norm(vector)*e_vector
            if np.linalg.norm(u_vector) < 1e-15:
                u_vector = vector + sign_vector*np.linalg.norm(vector)*e_vector
            v_vector = np.divide(u_vector, np.linalg.norm(u_vector)).copy()
            Householder_reflector = np.subtract(np.identity(m), 2*np.matmul(v_vector, np.transpose(v_vector)))
            return Householder_reflector

        for column_iter in range(n):
            Householder_reflector = get_householder_reflector(matrix_R, column_iter)
            matrix_R = np.matmul(Householder_reflector, matrix_R).copy()
            matrix_Q = np.matmul(matrix_Q, Householder_reflector).copy()

        matrix_R[np.abs(matrix_R) < epsilon_threshold] = 0
        return matrix_Q, matrix_R


    # --------------------------------------------------
    def get_companion_matrix(self, normalized_polynomial_coefficients):
        """
        Function to compute and return companion matrix of a polynomial p(x).
        Given a polynomial 
                        p(x) =  c_0 + c_1 x + c_2 x^2 + ... + c_{n-1} x^{n-1} + x^n, 
        normalized_polynomial_coefficients is an array of the form 
                                [ c_0 , c_1 , c_2 , ... , c_{n-1} ]
        For example     p(x) = 2 + 4x + 6x^2 + x^3 is represented as [2, 4, 6]

        Companion matrix for polynomial p(x) is given by:
                        [   0   0   0   ...     0   -c_0    ]
                        [   1   0   0   ...     0   -c_1    ]
                        [   0   1   0   ...     0   -c_2    ]
                        [   0   0   1   ...     0   -c_3    ]
                        [               ...     0           ]
                        [   0   0   0   ...     1   -c_n-1  ]
        Usage   :   companion_matrix = get_companion_matrix(normalized_polynomial_coefficients)
        Inputs  :   1.  normalized_polynomial_coefficients: Coefficients of the polynomial in 
                        the format given above in an array
        Outputs :   1.  companion_matrix: n \cross n matrix
        """
        dim_companion_matrix = len(normalized_polynomial_coefficients)
        companion_matrix = np.zeros((dim_companion_matrix, dim_companion_matrix))
        companion_matrix[1:, :dim_companion_matrix-1] = np.identity(dim_companion_matrix-1)
        companion_matrix[:, -1] = -1*normalized_polynomial_coefficients

        return companion_matrix


    # --------------------------------------------------
    def get_similar_matrix(self, matrix_A, epsilon_threshold=1e-7):
        """
        Function to compute similar matrix matrix_A_k that has the same eigenvalues as matrix_A
                A_1 = A (input matrix)
                A_1 = Q_1*R_1 (Q-R decomposition)
                A_2 = R_1*Q_1 = Q_1^T Q_1 R_1 Q_1 = Q_1^T A_1 Q_1
                A_2 = Q_2*R_2 
                A_3 = R_2*Q_2 = Q_2^T Q_2 R_2 Q_2 = Q_2^T A_2 Q_2
                Eventually A_k converges to an upper triangular matrix. Eigen values can be read
                off the diagonal of this matrix.
        Usage   :
                    matrix_A = get_similar_matrix(matrix_A, epsilon_threshold=1e-10)
                    matrix_A = get_similar_matrix(matrix_A)
        Inputs  :   1. matrix_A         :   A square matrix
                    2. epsilon_threshold:   Threshold to clean-up small numerical values in the 
                                            resultant matrix 
        Outputs :   1. matrix_A         :   Similar matrix to the input matrix (same eigenvalues),
                                            with an upper triangular or quasi upper triangular form      
        """
        epsilon_threshold = 1e-3
        matrix_A_first = matrix_A.copy()
        matrix_A_next = matrix_A.copy()
        for qr_iter in range(300):
            matrix_Q, matrix_R = self.get_qr_decomposition_householder(matrix_A_first)
            matrix_A_next = np.matmul(matrix_R, matrix_Q)
            # if (qr_iter+1)%50 == 0: 
                # print(f"Iteration {qr_iter+1}:")
                # print(matrix_A_next)
                # print(np.linalg.eigvals(matrix_A_next))
            matrix_A_first = matrix_A_next.copy()
        matrix_A_first[np.abs(matrix_A_first) < epsilon_threshold] = 0
        return matrix_A_first


    # --------------------------------------------------
    def get_eigenvalues_of_matrix(self, matrix_A):
        """
        Function to extract the eigenvalues from an upper triangular matrix A which is similar to 
        the companion matrix of the polynomial whose roots we are interested in computing.
        Usage   :   eigenvalues_of_matrix = get_eigenvalues_of_matrix(matrix_A)
        Inputs  :   1. matrix_A             : input matrix in (quasi) upper triangular form
        Outputs :   1. eigenvalues_of_matrix: eigenvalues of the matrix read from the diagonal
        """
        dim_companion_matrix = matrix_A.shape[0]
        eigenvalues_of_matrix = np.zeros(dim_companion_matrix, dtype=complex)
        a = complex(1,0)
        b = complex(1,0)
        c = complex(1,0)
        for column_iter in range(dim_companion_matrix):
            if column_iter > 0 and (matrix_A[column_iter:, column_iter-1] ==0).all()==False:
                continue
            elif (matrix_A[column_iter+1:, column_iter] ==0).all():
                print(f"Eigenvalue #{column_iter+1} is \t \t: {matrix_A[column_iter, column_iter]}")
                eigenvalues_of_matrix[column_iter] = matrix_A[column_iter, column_iter]
            else:
                assert((matrix_A[column_iter+2:, column_iter] ==0).all() == True)
                assert((matrix_A[column_iter+2:, column_iter+1] ==0).all() == True)
                matrix_slice = matrix_A[column_iter:column_iter+2, column_iter:column_iter+2]
                a_00 = matrix_slice[0,0]
                a_01 = matrix_slice[0,1]
                a_10 = matrix_slice[1,0]
                a_11 = matrix_slice[1,1]
                a = complex(1, 0)
                b = complex(-(a_11 + a_00), 0)
                c = complex(a_00*a_11 - a_01*a_10, 0)
                print(f"QR decomposition Eigenvalue #{column_iter+1} is \t \t: {(-1*b + np.sqrt(b**2 - 4*a*c))/(2*a)}")
                print(f"QR decomposition Eigenvalue #{column_iter+2} is \t \t: {(-1*b - np.sqrt(b**2 - 4*a*c))/(2*a)}")
                eigenvalues_of_matrix[column_iter] = (-1*b + np.sqrt(b**2 - 4*a*c))/(2*a)
                eigenvalues_of_matrix[column_iter+1] = (-1*b - np.sqrt(b**2 - 4*a*c))/(2*a)
        return eigenvalues_of_matrix


    # --------------------------------------------------
    def get_roots_of_polynomial_qr_technique(self, normalized_polynomial_coefficients):
        """
        Function to find all roots (real and complex) of a polynomial using the QR decomposition technique.
            In the following computation, polynomial p(x) is given by
                                p(x) = c_0 + c_1 x + c_2 x^2 + ... + c_{n-1} x^{n-1} + x^n 
        For example             p(x) = 2 + 4x + 6x^2 + x^3 is represented as            [2, 4, 6]

        Usage   :   roots_of_polynomial = get_roots_of_polynomial(normalized_polynomial_coefficients)
        Inputs  :   
                    1. normalized_polynomial_coefficients: Array of polynomial coefficients in the 
                    format as given in the example above
        Outputs :   None
        """
        
        matrix_A_companion = self.get_companion_matrix(normalized_polynomial_coefficients)
        matrix_A_similar = self.get_similar_matrix(matrix_A_companion)
        roots_of_polynomial = self.get_eigenvalues_of_matrix(matrix_A_similar)
        
        return roots_of_polynomial


    # --------------------------------------------------
    def get_closest_root_from_Newton_method_qr(self, poly, guess=0.0, eps=1e-6, max_iter=5000, verbose=False):

        # Validate the arguments and setup the defaults
        isinstance(poly, float)
        isinstance(guess, float)
        isinstance(eps,float)
        isinstance(max_iter, int)
        isinstance(verbose, bool)
        
        #Initialize the method, with the root at the guess and no iterations
        root = guess
        iter = 0
        n = len(poly)
        root_not_found = (self.poly_eval(poly, root))**2 > eps**2

        # Compute the coefficients of the derivative of the polynomial
        dpoly = np.zeros(n-1)
        for i in range(n-1):
            dpoly[i] = (n - (i+1))*poly[i]

        # Search for as long as we are too far from the solution and within our
        # acceptable number of iterations
        while root_not_found and iter < max_iter:
            # Build the numerator, poly(x = root), and the denominator, dpoly/dx(x = root)
            num = self.poly_eval(poly, root)
            den = self.poly_eval(dpoly, root)
            
            # Update the estimate of the root
            root = root - (num/den)
            #Prepare the next iteration
            iter = iter + 1
            root_not_found = np.abs(self.poly_eval(poly, root)) > eps**3
            if iter == 0:
                root_prev = root
            if (iter+1)%500 == 0:
                if iter+1 == max_iter and np.abs(root_prev - root)<1e-9 and np.abs(self.poly_eval(poly, root))<1e-9:
                    root_not_found = False
                else:
                    pass
                print("iteration = ", iter+1, "poly_eval(poly, root)",self.poly_eval(poly, root), " - root_not_found = ", root_not_found)
                root_prev = root

        # Set the root to NaN if we are still too far from the accepted error
        if root_not_found:
            root = float("NaN")

        return root


    # --------------------------------------------------
    def get_roots_of_polynomial(self, polynomial):
        """
        Function to find all roots (real and complex) of a polynomial using a mix of Newton's method
        (to find some/all real roots) and the QR decomposition technique to find the remaining real 
        roots and all complex roots.

        For the QR decomposition, the polynomial is wrtten differently. Please note:
        QR polynomial p(x) is given by
                                p(x) = c_0 + c_1 x + c_2 x^2 + ... + c_{n-1} x^{n-1} + x^n 
        For example             p(x) = 2 + 4x + 6x^2 + x^3 is represented as            [2, 4, 6]

        Usage   :   roots_of_polynomial = get_roots_of_polynomial(polynomial)
        Inputs  :   
                    1. polynomial           : Array of polynomial coefficients in the standard numpy format
        Outputs :   
                    1. roots_of_polynomial  : Array with all roots of the polynomial
        """

        def get_new_polynomial_forward(poly_array, root_p):
            """
            Funciton to compute new polynomial q(x), where p(x) = q(x) * (x - root), when p(x) and root are given.

            Usage   :   new_poly = get_new_polynomial_forward(poly_array, root_p)
            Inputs  :   1. poly_array   : Array of p(x) polynomial coefficients in the standard np.roots() format
                        2. root_p       : A root of the polynomial
            Outputs :   1. new_poly     : Array of q(x) polynomial coefficients in the standard np.roots() format
            """
            len_old_poly = len(poly_array)
            new_poly = np.zeros(len_old_poly -1)
            for new_poly_iter in range(len_old_poly -1):
                if new_poly_iter == 0:
                    new_poly[new_poly_iter] = poly_array[new_poly_iter]
                else:
                    new_poly[new_poly_iter] = poly_array[new_poly_iter] + root_p*new_poly[new_poly_iter-1]

            return new_poly

        def get_new_polynomial_backward(poly_array, root_p):
            """
            Funciton to compute new polynomial q(x), where p(x) = q(x) * (x - root), when p(x) and root are given.

            Usage   :   new_poly = get_new_polynomial_forward(poly_array, root_p)
            Inputs  :   1. poly_array   : Array of p(x) polynomial coefficients in the standard np.roots() format
                        2. root_p       : A root of the polynomial
            Outputs :   1. new_poly     : Array of q(x) polynomial coefficients in the standard np.roots() format

            Note: get_new_polynomial_forward() and get_new_polynomial_backward() compute the same array, but propogate 
            computation (and numerical errors) in different directions. If needed, an averaging technique can be 
            performed to ensure error is reduced.
            """
            len_old_poly = len(poly_array)
            new_poly = np.zeros(len_old_poly -1)
            for new_poly_iter in range(len_old_poly -2, -1, -1):
                if new_poly_iter == len_old_poly -2:
                    new_poly[new_poly_iter] = -1*(poly_array[new_poly_iter+1]/pole_v)
                else:
                    new_poly[new_poly_iter] = -1*(poly_array[new_poly_iter+1] - new_poly[new_poly_iter+1])/root_p

            return new_poly

        roots_of_polynomial = np.zeros(len(polynomial)-1, dtype=complex)
        root_number = 0

        iter_zero_roots = 0
        flag = 0
        while polynomial[-1 - iter_zero_roots] == 0:
            flag = 1
            roots_of_polynomial[root_number] = 0
            root_number = root_number + 1
            print(f"Root \t #{root_number} \t: 0")
            iter_zero_roots = iter_zero_roots + 1
        if flag == 1:
            polynomial = polynomial[:-iter_zero_roots]
        else: 
            pass

        new_poly_forward = polynomial
        new_poly_backward = polynomial
        pole_v = 1
        
        while pole_v != 0 and root_number < (len(polynomial)-1 -3) and not np.isnan(pole_v):
            pole_v = self.get_closest_root_from_Newton_method_qr(new_poly_forward)
            if pole_v != 0 and not np.isnan(pole_v):
                roots_of_polynomial[root_number] = pole_v
                root_number = root_number + 1
                print(f"Newton's method returns root \t #{root_number} \t:", pole_v)
                new_poly_forward = get_new_polynomial_forward(new_poly_forward, pole_v)
                new_poly_backward = get_new_polynomial_backward(new_poly_backward, pole_v)

        if root_number < (len(polynomial)-1 -3):
            space_search_len = 6
            search_iter = 0
            start = 0.0
            end = 6.0
            space_search = np.logspace(start, end, num=space_search_len)
            for iter_sign in range(1):
                if iter_sign == 1:
                    space_search = -1*space_search
                while search_iter < space_search_len and root_number < (len(polynomial)-1 -3):
                    get_possible_pole = self.get_closest_root_from_Newton_method_qr(new_poly_forward, space_search[search_iter])
                    if get_possible_pole == space_search[search_iter]:
                        pass
                    elif get_possible_pole == 0.0 or np.isnan(get_possible_pole):
                        pass
                    else:
                        roots_of_polynomial[root_number] = get_possible_pole
                        root_number = root_number + 1
                        print(f"Newton's method returns root \t #{root_number} \t:", get_possible_pole)
                        new_poly_forward = get_new_polynomial_forward(new_poly_forward, get_possible_pole)
                        new_poly_backward = get_new_polynomial_backward(new_poly_backward, get_possible_pole)
                        search_iter = 0
                    search_iter = search_iter + 1
        else:
            pass

        if root_number < (len(polynomial)-1):
            raw_polynomial_coefficients = list(reversed(new_poly_forward))
            normalized_polynomial_coefficients = np.divide(raw_polynomial_coefficients,raw_polynomial_coefficients[-1])[:-1]
            list_of_roots = self.get_roots_of_polynomial_qr_technique(normalized_polynomial_coefficients)
            roots_of_polynomial[-len(list_of_roots):] = list_of_roots
        else:
            pass
        return roots_of_polynomial

    
    # --------------------------------------------------
    def get_closest_root_from_Newton_method(self, poly, guess=0.0, eps=1e-6, max_iter=1000, verbose=False):

        # Validate the arguments and setup the defaults
        isinstance(poly, float)
        isinstance(guess, float)
        isinstance(eps,float)
        isinstance(max_iter, int)
        isinstance(verbose, bool)
        
        #Initialize the method, with the root at the guess and no iterations
        root = guess
        iter = 0
        n = len(poly)
        print("n = ",n)
        root_not_found = (self.poly_eval(poly, root))**2 > eps**2
        print("initial found root = ", (self.poly_eval(poly, root))**2)
        print("root_not_found = ",root_not_found)

        # Compute the coefficients of the derivative of the polynomial
        dpoly = np.zeros(n-1)
        for i in range(n-1):
            dpoly[i] = (n - (i+1))*poly[i]
    
        # Search for as long as we are too far from the solution and within our
        # acceptable number of iterations
        while root_not_found and iter < max_iter: 
            # Build the numerator, poly(x = root), and the denominator, dpoly/dx(x = root)
            num = self.poly_eval(poly, root)
            den = self.poly_eval(dpoly, root)
            

            # Update the estimate of the root
            root = root - num / den

            #Prepare the next iteration
            iter = iter + 1
            root_not_found = (self.poly_eval(poly, root))**2 > eps**4
            if (iter+1)%500 == 0:
                print("iter = ",iter, " - num = ",num, " - den = ",den, " - root = ", root, " - root_not_found = ", root_not_found)

            if (num / den)**2 < eps**4: 
                break

        # Set the root to NaN if we are still too far from the accepted error
        if root_not_found:
            root = float("NaN")

        return root


    def CalculateVelocity_PICoeffs_RLocus(self, R_est, L_est, T_delay, K_est, J_est, b_est, Kp_i, Ki_i):
        tau_d = T_delay*self.Tsample # = 1 Tsample delay in PI compute block + 0.5 Tsample delay in PWM Block.
        R = R_est
        L = L_est
        J = J_est
        bv = b_est
        K = K_est

        G_mech = 3*K/(2*R*bv + 3*K**2)
        wn_mech = math.sqrt((2*R*bv + 3*K**2)/(2*L*J))
        self.wn_mech = wn_mech
        zeta_mech = (L*bv + J*R)/math.sqrt(2*L*J*(2*R*bv + 3*K**2))
        print("G_mech = ",G_mech)
        print("wn_mech = ",wn_mech)
        print("zeta_mech = ",zeta_mech)

        b_npv = -(tau_d*Kp_i)/(2*Ki_i)
        a_npv = Kp_i/Ki_i - tau_d/2
        util_pv = 3*K/(2*bv*G_mech*Ki_i)

        d_dpv_1 = util_pv*tau_d/(2*wn_mech**2)
        c_dpv_1 = util_pv*(tau_d*zeta_mech*wn_mech + 1)/wn_mech**2 - (Kp_i*tau_d*J)/(2*Ki_i*bv)
        b_dpv_1 = util_pv*(tau_d/2 + 2*zeta_mech/wn_mech) - (Kp_i*tau_d)/(2*Ki_i) + (Kp_i*J)/(Ki_i*bv) - (tau_d*J)/(2*bv)
        a_dpv_1 = util_pv + Kp_i/Ki_i + J/bv - tau_d/2
        gain_pv = (3*K)/(2*bv)
        num_pv = [b_npv, a_npv, 1]
        den_pv = [d_dpv_1, c_dpv_1, b_dpv_1, a_dpv_1, 1]

        print("util_pv = ", util_pv)
        print("gain_pv = ", gain_pv)
        print("num_pv = ", num_pv)
        print("den_pv = ", den_pv)




        # Velocity filter tuning parameter
        # Get the frequency for which the magnitude of the velocity plant is at -3dB.
        # The two techniques below are valid, but the binary search might be more robust
        # There is no solution if the gain_pv is smaller than -3dB.
        # This is typically not a problem since bv is usually very small (~1e-4)
        wn_vf = self.get_freq_of_plant_v_at_gain_using_binary_search(-3, gain_pv, num_pv, den_pv)
        # self.calculateCurrentPiCoeffsZeta(R_est, L_est)
        # wn_vf = min(self.w_natural, wn_vf)/100
        
        wn_vf_ori = wn_vf
        start_division_factor = 10
        end_division_factor = 100
        step_division_factor = 2
        flag = 0
        acceptance_threshold = 1e-10
        division_factor_range_hold = list(range(start_division_factor, end_division_factor + 1, step_division_factor))
        division_factor_range = []
        division_factor_range.extend(division_factor_range_hold)
        # for division_factor in range(start_division_factor, end_division_factor + 1, step_division_factor): 
        print(f"Division factors are {division_factor_range}")
        for division_factor in division_factor_range: 
            print("#################################")
            print("division factor", division_factor)
            wn_vf = wn_vf_ori / division_factor
            fn_vf = wn_vf / (2*math.pi)
            zeta_vf = 1.0; # Keep the damping factor at 1, to prevent amplifying any freq
            print("Plant Frequency at Gain = ", wn_vf, "rads/sec")
            print("Plant Frequency at Gain = ", fn_vf, "Hz")
            
            # Velocity controller tuning parameter
            # Find the slowest pole of the plant, and update the plant's denominator
            # coefficients post zero-pole cancellation
            pole_v = self.get_closest_root_from_Newton_method(den_pv, guess=-J/bv)
            c_dpv_2 = -pole_v*d_dpv_1
            b_dpv_2 = -pole_v*(c_dpv_1 - c_dpv_2)
            a_dpv_2 = -pole_v*(b_dpv_1 - b_dpv_2)

            print("pole_v = ",pole_v)

            # Open-loop velocity polynomial parameters
            f_dolv = c_dpv_2/wn_vf**2
            e_dolv = 2*c_dpv_2/wn_vf + b_dpv_2/wn_vf**2
            d_dolv = c_dpv_2 + 2*b_dpv_2/wn_vf + a_dpv_2/wn_vf**2
            c_dolv = b_dpv_2 + 2*a_dpv_2/wn_vf + 1/wn_vf**2
            b_dolv = a_dpv_2 + 2/wn_vf
            a_dolv = 1
            den_olv = [f_dolv, e_dolv, d_dolv, c_dolv, b_dolv, a_dolv, 0]
            print("den_olv = ", den_olv)

            # Polynomial to find break-away point in the velocity root locus
            # polynomial p(s) = g_bp s^7 + f_bp s^6 + e_bp s^5 + d_bp s^4 + c_bp s^3 + b_bp s^2 + a_bp s^1 + 1
            g_bp = 4*f_dolv*b_npv
            f_bp = 5*f_dolv*a_npv + 3*e_dolv*b_npv
            e_bp = 6*f_dolv + 4*e_dolv*a_npv + 2*d_dolv*b_npv
            d_bp = 5*e_dolv + 3*d_dolv*a_npv + c_dolv*b_npv
            c_bp = 4*d_dolv + 2*c_dolv*a_npv
            b_bp = 3*c_dolv + b_dolv*a_npv - a_dolv*b_npv
            a_bp = 2*b_dolv
            poly_bp = [g_bp, f_bp, e_bp, d_bp, c_bp, b_bp, a_bp, 1]
            s_bp = 0
            s_bp = self.get_closest_root_from_Newton_method(poly_bp)
            print("s_bf is ", s_bp)

            """
            Caution! 
            In some parts of the qr decomposition computation, polynomial p(x) is given by
                                    p(x) = c_0 + c_1 x + c_2 x^2 + ... + c_{n-1} x^{n-1} + x^n 
            For example             p(x) = 2 + 4x + 6x^2 + x^3 is represented as            [2, 4, 6]
            Note for np.roots() the polynomial form is the other-way-around:
                                    r(x) = r_n + r_{n-1} x + r_{n-2} x^2 + ... + r_0 x^n
            And the polynomial      r(x) = 2 + 4x + 6x^2 + x^3 is represented as            [1, 6, 4, 2]
            """
            roots_of_polynomial = self.get_roots_of_polynomial(poly_bp)
            for iter_roots in range(len(roots_of_polynomial)):
                print(f"Root #{iter_roots}: \t {roots_of_polynomial[iter_roots]}")

            negative_roots = roots_of_polynomial[roots_of_polynomial.real < 0]
            closest_root_to_y_axis = -1*np.min(np.abs(negative_roots.real))
            if closest_root_to_y_axis in negative_roots:
                print(f"Closest negative root to the y-axis is real")
                flag_real_closest_pole = 1
            else:
                print(f"Closest negative roots to the y-axis is a complex pair")
                flag_real_closest_pole = 0

            print(":)")

            # Compute the PI coefficients
            Ki_v = -(1/gain_pv)*self.poly_eval(den_olv, s_bp)/self.poly_eval(num_pv, s_bp)
            Kp_v = -Ki_v/pole_v
            print("Kp, Ki are", Kp_v, Ki_v)
            if flag_real_closest_pole:
                break
           
            # print("Kp, Ki are", Kp_v, Ki_v)
            # if division_factor == start_division_factor: 
            #     Kp_v_prev = Kp_v
            #     Ki_v_prev = Ki_v
            # else: 
            #     if np.isnan(Kp_v) or np.isnan(Ki_v): 
            #         continue
            #     elif np.abs(Kp_v) < acceptance_threshold or np.abs(Ki_v) < acceptance_threshold: 
            #         continue
            #     else: 
            #         if np.abs((Kp_v - Kp_v_prev) / Kp_v)*100 < 15: 
            #             break
            #         Kp_v_prev = Kp_v
            #         Ki_v_prev = Ki_v
            # print("################################")
                
        kp_calc = Kp_v
        ki_calc = Ki_v

        kp_float = kp_calc
        ki_float = ki_calc
        ki_float_dt = ki_calc * self.Tsample

        print("Velocity Mechanical Bandwidth = ", wn_mech)
        print("Velocity Root-Locus Kp    =  ",kp_float )
        print("Velocity Root-Locus Limited Ki    =  ",ki_float )
        print("Velocity Root-Locus Limited Ki.dt =  ",ki_float_dt )

        return fn_vf, kp_float, ki_float, ki_float_dt


    def ScaleVelocity_PICoeffs(self, kp_float, ki_float):
        
        self.TM01Setup.SetupVelocityPIParams()

        verbose_print = False

        kp_vel_scaling_shft24, ki_vel_scaling_shft32 = self.TM01Setup.get_velocity_pidcoeffs_scaling_max()

        # Temporary Velocity PI Coefficient Scaling reduction by 1.0X
        kp_float = kp_float * 1.0
        ki_float = ki_float * 1.0

        print(" Velocity Kp Float Value to be Scaled = ", kp_float)
        print(" Velocity Ki Float Value to be Scaled = ", ki_float)

        if verbose_print:
            print(" Velocity Kp Scaling Factor (Shift24)  = ", kp_vel_scaling_shft24)
            print(" Velocity Ki Scaling Factor (Shift32)  = ", ki_vel_scaling_shft32)

        kp_vel_float_shft24  = kp_float * kp_vel_scaling_shft24
        ki_vel_float_shft32  = ki_float * ki_vel_scaling_shft32

        if verbose_print:
            print(" Initial Velocity Kp Value Factor (Shift24)  = ", kp_vel_float_shft24)
            print(" Initial Velocity Ki Value (Shift32)  = ", ki_vel_float_shft32)

        # Kp_int and Ki_int are limited to 15bits or a maximum value of 32767 - CURRENT_NORM_x = 1 => Q0.16, CURRENT_NORM_x = 0 => Q8.8
        max_pi_int = 32767

        if max_pi_int < kp_vel_float_shft24 <= 0:
            print("Velocity Kp value is zero, negative or too large. Kp Scaled = ", kp_vel_float_shft24)
            exit(1)
        else:
            kp_vel_tmp = kp_vel_float_shft24
            if verbose_print: print("Inital Kp_vel_tmp value = ",kp_vel_tmp)
            for i in reversed(range(0,4)):
 
                if kp_vel_tmp <= max_pi_int:
                    kp_velocity_int = int(kp_vel_tmp)
                    velocity_norm_p = i
                    if verbose_print: print("Loop itteration: ",i," Kp = ",kp_velocity_int, " vel_norm_p = ", velocity_norm_p)
                    break
                else:
                    kp_vel_tmp = kp_vel_tmp/(2**(8))
                    if verbose_print: print("Loop itteration: ",i," Next Kp_vel_tmp value = ",kp_vel_tmp)


        if max_pi_int < ki_vel_float_shft32 <= 0:
            print("Velocity Ki value is zero, negative or too large. Kp Scaled = ", ki_vel_float_shft32)
            exit(1)
        else:
            ki_vel_tmp = ki_vel_float_shft32
            if verbose_print: print("Inital Ki_vel_tmp value = ",ki_vel_tmp)
            for i in reversed(range(0,4)): 
                if ki_vel_tmp <= max_pi_int:
                    ki_velocity_int = int(ki_vel_tmp)
                    velocity_norm_i = i
                    if verbose_print: print("Loop itteration: ",i," Ki = ",ki_velocity_int, " vel_norm_i = ", velocity_norm_i)
                    break
                else:
                    ki_vel_tmp = ki_vel_tmp/(2**(8))
                    if verbose_print: print("Loop itteration: ",i," Next Ki_vel_tmp value = ",ki_vel_tmp)

        print(" Computed Velocity Kp (bits) = ", kp_velocity_int)
        print(" Computed Velocity Ki (bits) = ", ki_velocity_int)
        print(" Computed Kp Velocity Normalization =  ", velocity_norm_p)
        print(" Computed Ki Velocity Normalization =  ", velocity_norm_i)
        
        return kp_velocity_int, ki_velocity_int, velocity_norm_p, velocity_norm_i


    def CalculatePosition_PICoeffs(self, zeta_v, zeta_i):
        tau_d = 3.0*self.Tsample # Estimate Position Loop Delay - 3 Torque Loop, 4 in moving average filter, 1 in vel measurement, 1 in velocity biquad, 1 in encoder, 1 in vel pi control. 
        w_n = 1/(2*zeta_i*tau_d)
        tau_i = (1/w_n)*2*math.pi # 2nd Order Current loop to 1st Order Approximation (1/(tau_i*s + 1))
        w_nv = 1/(2*zeta_v*tau_i)
        w_cv = w_nv * math.sqrt( 1 - zeta_v**2 + math.sqrt( 2 - 4*zeta_v**2  + 4*zeta_v**4) )
        w_cp = w_cv/10

        kp_float = (w_cp/math.sqrt(2)) * math.sqrt( (1-((w_cp**2)/(w_nv**2)))**2 + ((2*zeta_v*w_cp)/w_nv)**2 )
        
        f_nv = w_nv/(2*math.pi)
        f_cv = w_cv/(2*math.pi)
        f_cp = w_cp/(2*math.pi)
    
        print(" Velocity Loop Natural Frequency = ", w_nv, " rads/sec.")
        print(" Velocity Loop Cut-off Frequency = ", w_cv, " rads/sec.")
        print(" Position Loop Cut-off Frequency = ", w_cp, " rads/sec.")
        print(" Velocity Loop Natural Frequency = ", f_nv, " Hz.")
        print(" Velocity Loop Cut-off Frequency = ", f_cv, " Hz.")
        print(" Position Loop Cut-off Frequency = ", f_cp, " Hz.")
        
        print(" Computed Position Kp = ", kp_float)
        
        return kp_float


    def ScalePosition_PICoeffs(self, kp_float ):

        self.TM01Setup.SetupPositionPIParams()

        verbose_print = False

        kp_pos_scaling_shft24, ki_pos_scaling_shft32 = self.TM01Setup.get_position_pidcoeffs_scaling_max()
        
        if verbose_print: print(" Position Kp Scaling Factor (Shift24)  = ", kp_pos_scaling_shft24)
        if verbose_print: print(" Position Ki Scaling Factor (Shift32)  = ", ki_pos_scaling_shft32)

        kp_pos_float_shft24  = kp_float * kp_pos_scaling_shft24

        # Kp_int and Ki_int are limited to 15bits or a maximum value of 32767 - CURRENT_NORM_x = 1 => Q0.16, CURRENT_NORM_x = 0 => Q8.8
        max_pi_int = 32767

        if max_pi_int < kp_pos_float_shft24 <= 0:
            print("Position Kp value is zero, negative or too large. Kp Scaled = ", kp_pos_float_shft24)
            exit(1)
        else:
            kp_pos_tmp = kp_pos_float_shft24
            print("Inital Kp_pos_tmp value = ",kp_pos_tmp)
            for i in reversed(range(0,4)):
 
                if kp_pos_tmp <= max_pi_int:
                    kp_position_int = int(kp_pos_tmp)
                    position_norm_p = i
                    print("Loop itteration: ",i," Kp = ",kp_position_int, " pos_norm_p = ", position_norm_p)
                    break
                else:
                    kp_pos_tmp = kp_pos_tmp/(2**(8))
                    print("Loop itteration: ",i," Next Kp_pos_tmp value = ",kp_pos_tmp)

        print(" Computed Position Kp (bits) = ", kp_position_int)
        print(" Kp Position Normalization =  ", position_norm_p)
        
        return kp_position_int, position_norm_p
    
    # Functions for FFT filter
    def plot_freq_domain_response(self, fshift, Xshift, Yshift, nfft):
        """
        Function to plot FFT spectrum of two frequency domain signals Xshift and Yshift

        Usage:      plot_freq_domain_response(fshift, Xshift, Yshift, nfft)

        Inputs:
                    1. fshift:  Array with frequency values for the x-axis
                    2. Xshift:  Array with FFT of signal 1 for the y-axis
                    3. Yshift:  Array with FFT of signal 2 for the y-axis
                    4. nfft:    length of the time domain signals x, y (they are assumed to be of the same length)
        Outputs:
                    None
        """
        fig, ax = plt.subplots()
        ax.scatter(fshift, 20 * np.log10(abs(Yshift / (nfft / 2))), label='Magnitude FFT(I_q in A) ', marker="x", alpha=0.95,
                    c='tab:blue')
        ax.scatter(fshift, 20 * np.log10(abs(Xshift / (nfft / 2))), label='Magnitude FFT(Omega in rad/s) ', marker="x", alpha=0.95,
                    c='tab:red')
        ax.set_xlabel('Frequency (in Hz)')
        ax.set_ylabel('Amplitude (in log scale)')
        ax.set_title("Magnitude FFT(Iq) and FFT(omega)")
        ax.grid(True)
        get_legends = plt.legend()
        leg_lines = get_legends.get_lines()
        leg_texts = get_legends.get_texts()
        plt.setp(leg_lines, linewidth=10)
        plt.setp(leg_texts, fontsize='x-large')
        plt.show(block=False)
        plt.pause(5)
        plt.close()


    def filter_FFT(self, x_dummy_var_new, y_dummy_var_new, timestep, zero_noisy_freq_comp=1, fraction_target_x = 0.05, fraction_target_y = 0.05, zero_high_freq_comp=1, expansion_factor=0.15):
        """
        Function to compute FFT, remove "noise", plot the spectrum and return *two* "denoised" time domain signals

        Usage:
            x_filtered, y_filtered = filter_FFT(x_dummy_var_new, y_dummy_var_new, timestep, 1, 0.05, 0.05, 1, 0.15)
            x_filtered, y_filtered = filter_FFT(x_dummy_var_new, y_dummy_var_new, timestep)
        
        Inputs: 
            1. x_dummy_var_new      :   Time domain signal 1 (x)
            2. y_dummy_var_new      :   Time domain signal 2 (y)
            3. timestep             :   Sampling interval of the signals (It is assumed that the two signals have the same sampling time)
            4. zero_noisy_freq_comp :   Boolean variable. Do you  want to mute the noisy fequencies below a particular signal strength? 
            5. fraction_target_x    :   What percentage of the signal 1 (2) strength in dB is considered signal? For e.g. fraction_target_x = 0.4 will keep
                                        signals whose strength is above average(signal in dB)*(1-0.40) and discard signals whose strength is below 
                                        average(signal in dB)*(1-0.40)
            6. fraction_target_y    :   Same as fraction_target_x for signal 2 (y)
            7. zero_high_freq_comp  :   Boolean variable. Do you want to mute high frequency components?
            8. expansion_factor     :   Expansion_factor % of frequencies centered at DC (0 Hz frequency) is retained.

        Outputs:

            1. x_filtered:          FFT filtered time domain signal 1 (x)
            2. y_filtered:          FFT filtered time domain signal 2 (y)
        """
        nfft = len(x_dummy_var_new)
        Y_s = np.fft.fft(y_dummy_var_new)
        X_s = np.fft.fft(x_dummy_var_new)

        fft_freq = np.fft.fftfreq(nfft, d=timestep)

        n = len(Y_s)
        fs_fft = 1/timestep
        f_fft = np.arange(n) * (fs_fft / n)

        fshift = np.fft.fftshift(fft_freq)
        Yshift = np.fft.fftshift(Y_s)
        Xshift = np.fft.fftshift(X_s)

        # plot_freq_domain_response(fshift, Xshift, Yshift, nfft) # X is omega; Y is Iq

        fraction_target_omega = fraction_target_x
        fraction_target_iq = fraction_target_y

        if zero_noisy_freq_comp:
            if np.mean(20*np.log10(abs(Xshift / (nfft / 2)))) < 0:
                omega_max_info = np.mean(20*np.log10(abs(Xshift / (nfft / 2))))*(1 - fraction_target_omega)
            else:
                omega_max_info = np.mean(20*np.log10(abs(Xshift / (nfft / 2))))*(1 + fraction_target_omega)

            if np.mean(20*np.log10(abs(Yshift / (nfft / 2)))) < 0:
                iq_max_info = np.mean(20*np.log10(abs(Yshift / (nfft / 2))))*(1 - fraction_target_iq)
            else:
                iq_max_info = np.mean(20*np.log10(abs(Yshift / (nfft / 2))))*(1 + fraction_target_iq)

            last_freq_omega = np.nonzero(20*np.log10(abs(Xshift / (nfft / 2))) > omega_max_info)[0][-1]
            last_freq_iq = np.nonzero(20*np.log10(abs(Yshift / (nfft / 2))) > iq_max_info)[0][-1]

            Xshift[np.where(20*np.log10(abs(Xshift / (nfft / 2))) < omega_max_info)] = 0
            Yshift[np.where(20*np.log10(abs(Yshift / (nfft / 2))) < iq_max_info)] = 0

        if zero_high_freq_comp:
            last_freq_omega = fshift[int(np.round((n/2) * (1 + expansion_factor)))]
            last_freq_iq = fshift[int(np.round((n/2) * (1 + expansion_factor)))]

            Xshift[np.where(np.abs(fshift) > last_freq_omega)] = 0
            Yshift[np.where(np.abs(fshift) > last_freq_iq)] = 0

        # plot_freq_domain_response(fshift, Xshift, Yshift, nfft) # X is omega; Y is Iq

        ifshift = np.fft.ifftshift(fshift)
        iYshift = np.fft.ifftshift(Yshift)
        iXshift = np.fft.ifftshift(Xshift)

        x_ifft = np.fft.ifft(iXshift)
        y_ifft = np.fft.ifft(iYshift)

        return x_ifft.real, y_ifft.real
