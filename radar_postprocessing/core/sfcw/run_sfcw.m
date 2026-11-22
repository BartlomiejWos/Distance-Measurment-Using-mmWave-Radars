clc; clear; close all;
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'common', 'utils_dsp'));
addpath(fullfile(here, '..', 'common', 'parser'));

% Config
rel_path = fullfile('..', '..', 'measurement_data', 'sfcw_sweep', 'sfcw_comparision_000');
folder_path = fullfile(here, rel_path);
Nfft = 8192;
R_target = 1.15;         % [m] nominal target distance
noise_cm = [1000 1200];  % noise-floor band for power-vs-noise normalization

sfcw = SfcwProcessor(folder_path, Nfft);
sfcw = sfcw.remove_dc();
sfcw = sfcw.range_profile(@hamming);

target_bin = peak_bin(sfcw.Y, sfcw.R, R_target);
R_peak = sfcw.R(target_bin);
fprintf('Estimated nominal target range: R_peak = %.4f m  (peak in %s)\n', ...
        R_peak, rel_path);

phase_ideal_rad = sfcw.ideal_phase(R_peak);
tone_phase_rad = unwrap(angle(sfcw.V));
power_dB = range_profile_power_rel_noise(abs(sfcw.Y), sfcw.Rcm, noise_cm);

figure;
sgtitle('Raw (no clutter removal)');

subplot(2, 1, 1);
plot(sfcw.R, power_dB, 'LineWidth', 1.2); grid on;
xlabel('Range [m]');
ylabel('Power rel. noise [dB]');
xlim([0 15]);
title('Range Profile');

subplot(2, 1, 2);
plot(sfcw.f/1e9, tone_phase_rad, 'LineWidth', 1.2); hold on;
plot(sfcw.f/1e9, phase_ideal_rad, 'w--', 'LineWidth', 1.2);

frame_ends = cumsum(sfcw.num_chirps);
for k = 1:numel(frame_ends)-1
    xline(sfcw.f(frame_ends(k))/1e9, '--r', sprintf('F%d', k), 'LineWidth', 0.8);
end
grid on;
xlabel('Frequency [GHz]');
ylabel('Phase [rad]');
title('Phase vs. Frequency');
legend('Measured', sprintf('Ideal (R=%.4f m)', R_peak), 'Location', 'best');

tones_ideal = mean(abs(sfcw.V)) * exp(1j * phase_ideal_rad);

figure;
plot(real(sfcw.V), imag(sfcw.V), '.', 'MarkerSize', 8); hold on;
plot(real(tones_ideal), imag(tones_ideal), '.', 'MarkerSize', 8);
grid on; axis equal;
xlabel('Real'); ylabel('Imag');
title('Raw — Tone Constellation');
legend('Measured', sprintf('Ideal (R=%.4f m)', R_peak), 'Location', 'best');
