clc; clear; close all;
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', 'common', 'parser'));
addpath(fullfile(here, '..', 'common', 'utils_dsp'));
addpath(fullfile(here, '..', 'fmcw'));
addpath(fullfile(here, '..', 'sfcw'));

fmcw_folder = "C:\Users\Asus\Desktop\LAB_POMIARY\final\fmcw_sweep_2\fmcw_comparision_010";
sfcw_folder = "C:\Users\Asus\Desktop\LAB_POMIARY\final\sfcw_sweep\sfcw_comparision_010";

Fs = 7.2e6;                   % FMCW ADC rate [Hz]
noise_boundary = [10e2 12e2];
n_chirps = 128;               % 1 = single chirp / 128 = average of all

fmcw_raw = FmcwProcessor(fmcw_folder, Fs, 8192);
fmcw_raw = fmcw_raw.range_profile(@hamming);
n_avail = size(fmcw_raw.RP, 2);
n_use = min(n_chirps, n_avail);
fprintf('FMCW capture: %d chirps available, averaging %d\n', n_avail, n_use);
fmcw_raw.Y = mean(fmcw_raw.RP(:, 1:n_use), 2);
P_fmcw_raw = fmcw_raw.power_rel_noise(noise_boundary);

sfcw_raw = SfcwProcessor(sfcw_folder);
sfcw_raw = sfcw_raw.average_tones();
sfcw_raw = sfcw_raw.remove_dc();
sfcw_raw = sfcw_raw.range_profile(@hamming);
P_sfcw_raw = range_profile_power_rel_noise(abs(sfcw_raw.Y), sfcw_raw.Rcm, noise_boundary);

phase_sfcw_raw = unwrap(angle(sfcw_raw.V));
phase_sfcw_raw = phase_sfcw_raw - phase_sfcw_raw(1);
f_sfcw_ghz = sfcw_raw.f / 1e9;

phase_fmcw_raw = unwrap(angle(mean(fmcw_raw.X(:, 1:n_use), 2)));
phase_fmcw_raw = phase_fmcw_raw - phase_fmcw_raw(1);
t_fmcw_us = (0:fmcw_raw.N-1).' / Fs * 1e6;

col_fmcw = [0.00 0.45 0.74]; 
col_sfcw = [0.85 0.33 0.10];
xlim_m = [0 min([fmcw_raw.Rmax, sfcw_raw.cfg.Rmax, 15])];

figure('Name', 'Raw range profiles');
plot(fmcw_raw.R, P_fmcw_raw, 'LineWidth', 1.4, 'Color', col_fmcw); hold on;
plot(sfcw_raw.R, P_sfcw_raw, 'LineWidth', 1.4, 'Color', col_sfcw);
grid on; xlim(xlim_m);
xlabel('Range [m]'); ylabel('Power rel. noise [dB]');
title('Raw range profiles — FMCW vs SFCW');
legend('FMCW', 'SFCW', 'Location', 'best');

figure('Name', 'Raw phase');
subplot(2,1,1);
plot(t_fmcw_us, phase_fmcw_raw, 'LineWidth', 1.2, 'Color', col_fmcw);
grid on; xlabel('Time [\mus]'); ylabel('Phase [rad]');
title('FMCW beat phase (raw) — chirp 1');

subplot(2,1,2);
plot(f_sfcw_ghz, phase_sfcw_raw, 'LineWidth', 1.2, 'Color', col_sfcw);
grid on; xlabel('Frequency [GHz]'); ylabel('Phase [rad]');
title('SFCW tone phase (raw)');
