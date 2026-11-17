clc; clear; close all;
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, 'calibration'));
addpath(fullfile(here, '..', 'common', 'utils_dsp'));
addpath(fullfile(here, '..', 'common', 'parser'));

meas_dir = "C:\Users\Asus\Desktop\LAB_POMIARY\final\sfcw_sweep\sfcw_comparision_000";

noise_band = [10e3 12e3]

proc_base = SfcwProcessor(meas_dir);
proc_base = proc_base.average_tones();
proc_base = proc_base.remove_dc();

p_est = proc_base.range_profile();
R_search_min_cm = 50;
R_search_max_cm = 1000;
mask = (p_est.Rcm >= R_search_min_cm) & (p_est.Rcm <= R_search_max_cm);
[~, k_peak] = max(abs(p_est.Y(mask)));
Rcm_masked = p_est.Rcm(mask);
R_hat = Rcm_masked(k_peak) / 100;
fprintf('Estimated nominal target range: R_hat = %.4f m  (peak in %s)\n', ...
        R_hat, meas_dir);

phi_ideal = proc_base.ideal_phase(R_hat);

p       = proc_base.range_profile();
phase_i = unwrap(angle(p.V));
P_dB_i  = range_profile_power_rel_noise(abs(p.Y), p.Rcm, noise_band);

figure;
sgtitle('Raw (no clutter removal)');

subplot(2, 1, 1);
plot(p.Rcm / 100, P_dB_i, 'LineWidth', 1.2); grid on;
xlabel('Range [m]');
ylabel('Power rel. noise [dB]');
xlim([0 75]);
title('Range Profile');

subplot(2, 1, 2);
plot(p.f/1e9, phase_i, 'LineWidth', 1.2); hold on;
plot(p.f/1e9, phi_ideal, 'w--', 'LineWidth', 1.2);

offsets = cumsum(p.N_parts);
for k = 1:numel(offsets)-1
    xline(p.f(offsets(k))/1e9, '--r', sprintf('F%d', k), 'LineWidth', 0.8);
end
grid on;
xlabel('Frequency [GHz]');
ylabel('Phase [rad]');
title('Phase vs. Frequency');
legend('Measured', sprintf('Ideal (R=%.4f m)', R_hat), 'Location', 'best');

V_ideal = mean(abs(p.V)) * exp(1j * phi_ideal);

figure;
plot(real(p.V), imag(p.V), '.', 'MarkerSize', 8); hold on;
plot(real(V_ideal), imag(V_ideal), '.', 'MarkerSize', 8);
grid on; axis equal;
xlabel('Real'); ylabel('Imag');
title('Raw — Tone Constellation');
legend('Measured', sprintf('Ideal (R=%.4f m)', R_hat), 'Location', 'best');
