clc; clear; close all;
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'common', 'parser'));
addpath(fullfile(here, '..', 'common', 'utils_dsp'));

% Config
folder_path = fullfile(here, '..', '..', 'measurement_data', 'fmcw_sweep', 'fmcw_comparision_000');
Fs = 7.2e6;        % [Hz] ADC sample rate (not in metadata)
Nfft = 8192;
noise_cm = [1000 1200];  % noise-floor band for power-vs-noise normalization


wins = { @hamming, 'Hamming'; ...
         @hann,    'Hann' };

proc = FmcwProcessor(folder_path, Fs, Nfft);
proc.print_info();

nw = size(wins, 1);
P_raw = cell(nw, 1);
for i = 1:nw
    pw = proc.range_profile(wins{i, 1});
    if i == 1, R_axis = pw.R; end
    P_raw{i} = range_profile_power_rel_noise(abs(pw.Y), pw.Rcm, noise_cm);
end

figure('Name', 'FMCW range profile - window comparison');
hold on;
for i = 1:nw, plot(R_axis, P_raw{i}, 'LineWidth', 1.4); end
grid on; xlim([0 15]); xlabel('Range [m]'); ylabel('Power rel. noise [dB]');
legend(wins{:, 2}, 'Location', 'best');
title('Raw range profile (DC removed)');
