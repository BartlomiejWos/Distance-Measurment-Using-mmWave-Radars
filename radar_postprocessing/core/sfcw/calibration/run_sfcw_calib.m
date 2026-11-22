clc; clear; close all;
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..'));
addpath(fullfile(here, '..', '..', 'common', 'utils_dsp'));
addpath(fullfile(here, '..', '..', 'common', 'parser'));

% Config
cal_dir = fullfile(here, '..', '..', '..', 'measurement_data', 'sfcw_sweep');
folder_indices = (0:65).';
prefix = 'sfcw_comparision';
displacement_mm = folder_indices * 0.1;
R_target = 1.15; % [m] nominal target distance

Nfft = 8192;
cal_save_path = fullfile(here, 'sfcw_calibration.mat');

clutter_method = 'ica';   % 'ica' | 'off'
ica_Rcut_cm = 90;
ica_ratio = 0.076;
win_fn = @hamming;
calc_iq_from_peak = true;
iq_from_raw = true;

% Load data
M = numel(displacement_mm);
[C_raw, proc1] = load_tone_captures(cal_dir, prefix, folder_indices, Nfft);
N = proc1.N;
f = proc1.f;
cfg = proc1.cfg;
Rmax = proc1.Rmax;
Rcm_axis = proc1.Rcm;
fprintf('Loaded %d positions x %d tones | f = [%.3f .. %.3f] GHz | df = %.1f kHz\n', ...
        M, N, f(1)/1e9, f(end)/1e9, cfg.df_hz/1e3);

% Clutter removal
switch clutter_method
    case 'ica'
        C_clean = remove_clutter_ica(C_raw, Nfft, Rmax, ica_Rcut_cm, ica_ratio);
        fprintf('Clutter removal: ICA (Rcut=%d cm, ratio>%.2f)\n', ica_Rcut_cm, ica_ratio);
    case 'off'
        C_clean = C_raw;
        fprintf('Clutter removal: OFF\n');
    otherwise
        error('run_sfcw_calib:clutterMethod', 'Unknown clutter_method: %s', clutter_method);
end

Y1 = ifft(C_clean(:, 1) - mean(C_clean(:, 1)), Nfft);
k_target = peak_bin(Y1, proc1.R, R_target);
R_peak = proc1.R(k_target);
fprintf('Target bin: idx %d, R = %.4f m\n', k_target, R_peak);

R_vec = (displacement_mm(:) - displacement_mm(1)) / 1000;   % relative robot motion [m]

if iq_from_raw
    C_iq = C_raw - mean(C_raw, 1);
    fprintf('IQ estimation: on RAW (pre-ICA) DC-removed tones\n');
else
    C_iq = C_clean;
end
if calc_iq_from_peak
    Y_iq = ifft(C_iq - mean(C_iq, 1), Nfft);
    Y_tb = Y_iq(k_target, :);
    [dg_est, dp_est] = correct_iq_differential(Y_tb);
    fprintf('IQ estimation: from the target bin (k=%d) across positions\n', k_target);
else
    [dg_est, dp_est] = correct_iq_differential(C_iq);
end
fprintf('IQ imbalance: dg = %.4f, dp = %.2f deg\n', dg_est, rad2deg(dp_est));

cal_est = calibrate_nguyen(C_clean, f, R_vec, dg_est, dp_est);
fprintf('Calibration extracted (Nguyen)\n');
fprintf('  |A_ref(f)|  range:  [%.3f, %.3f]\n', min(cal_est.A_ref), max(cal_est.A_ref));
fprintf('  beta(f)     range:  [%.2f, %.2f] deg\n', ...
        rad2deg(min(cal_est.beta)), rad2deg(max(cal_est.beta)));

cal_out = cal_est;
cal_out.f = f;
cal_out.cfg = cfg;
cal_out.timestamp = datetime('now');
cal_out.cal_dir = cal_dir;
cal_out.positions_mm = displacement_mm;
cal_out.clutter_method = clutter_method;
save(cal_save_path, '-struct', 'cal_out');
fprintf('Calibration saved: %s\n', cal_save_path);


win = win_fn(N);
unwrap_ref = @(V) unwrap(angle(V)) - angle(V(1));
norm_dB = @(Y) 20*log10(abs(Y)/max(abs(Y)) + eps);
phi_ideal = -(4*pi*R_peak/phys_const.C) * (f - f(1));
col_raw = [0.00 0.45 0.74];
col_clean = [0.85 0.33 0.10];
noise_cm = [1000 1200];


V_meas = C_raw(:,1) - mean(C_raw(:,1));
phi_raw = unwrap_ref(V_meas);
V_ideal_c = mean(abs(V_meas)) * exp(1j * phi_ideal);

figure('Name', 'Raw (pre-clutter) — phase + constellation vs ideal');
subplot(1,2,1);
plot(f/1e9, phi_raw,  'LineWidth', 1.4); hold on;
plot(f/1e9, phi_ideal, '--', 'LineWidth', 1.2, 'Color', [0.5 0.5 0.5]);
grid on; xlabel('Frequency [GHz]'); ylabel('Phase [rad]');
legend('Measured (raw)', sprintf('Ideal (R=%.4f m)', R_peak), 'Location', 'best');
title('Tone phase vs ideal — raw');
subplot(1,2,2);
plot(real(V_meas),    imag(V_meas),    '.', 'MarkerSize', 8); hold on;
plot(real(V_ideal_c), imag(V_ideal_c), '.', 'MarkerSize', 8);
grid on; axis equal; xlabel('Real'); ylabel('Imag');
legend('Measured (raw)', sprintf('Ideal (R=%.4f m)', R_peak), 'Location', 'best');
title('Tone constellation vs ideal — raw');


Y_raw1 = ifft(win .* (C_raw(:,1)   - mean(C_raw(:,1))),   Nfft);
Y_cln1 = ifft(win .* (C_clean(:,1) - mean(C_clean(:,1))), Nfft);
P_raw1 = range_profile_power_rel_noise(abs(Y_raw1), Rcm_axis, noise_cm);
P_cln1 = range_profile_power_rel_noise(abs(Y_cln1), Rcm_axis, noise_cm);

figure('Name', 'SFCW — range profile: before vs after clutter removal');
plot(Rcm_axis/100, P_raw1,    'LineWidth', 1.4, 'Color', col_raw); hold on;
plot(Rcm_axis/100, P_cln1, 'LineWidth', 1.4, 'Color', col_clean);
grid on; xlim([0 min(15, Rcm_axis(end)/100)]);
xlabel('Range [m]', 'FontSize', 14);
ylabel('Power rel. to noise [dB]', 'FontSize', 14);
title('SFCW Range profile - before vs after clutter removal', 'FontSize', 14);
legend('raw profile', 'profile after ica', 'Location', 'best');


ref = max(20*log10(abs(Y_raw1)));
figure('Name', 'SFCW — range profile: normalized amplitude (raw vs ICA)');
plot(Rcm_axis/100, 20*log10(abs(Y_raw1))    - ref, 'LineWidth', 1.4, 'Color', col_raw); hold on;
plot(Rcm_axis/100, 20*log10(abs(Y_cln1)) - ref, 'LineWidth', 1.4, 'Color', col_clean);
grid on; xlim([0 min(15, Rcm_axis(end)/100)]);
xlabel('Range [m]', 'FontSize', 14);
ylabel('Normalized amplitude [dB]', 'FontSize', 14);
title('SFCW Range profile - before vs after clutter removal', 'FontSize', 14);
legend('raw profile', 'profile after ica', 'Location', 'best');

% calibration coefficients
figure;
sgtitle('Calibration coefficients frequency response', 'FontSize', 20);
yyaxis left;
hA = plot(f/1e9, 20*log10(cal_est.A_ref), 'LineWidth', 2.0, 'Color', [0.00 0.45 0.74]);
ylabel('Amplitude deviation [dB]');
yyaxis right;
hB = plot(f/1e9, rad2deg(cal_est.beta), 'LineWidth', 2.0, 'Color', [0.85 0.33 0.10]);
ylabel('Non-linear phase error [deg]');
xlabel('Frequency [GHz]'); grid on;
legend([hA hB], 'Amplitude deviation', 'Non-linear phase error', 'Location', 'best');
ax = gca;
ax.YAxis(1).Color = 'k';
ax.YAxis(2).Color = 'k';
set(ax, 'FontSize', 16);
ax.XLabel.FontSize = 20;
ax.YAxis(1).Label.FontSize = 20;
ax.YAxis(2).Label.FontSize = 20;

m_val = round(M/2);
V_raw = C_raw(:, m_val);
V_clean = C_clean(:, m_val);
V_cal = apply_calib(V_clean, cal_est, true, true);
Y_raw = ifft(win .* (V_raw   - mean(V_raw)),   Nfft);
Y_cln = ifft(win .* (V_clean - mean(V_clean)), Nfft);
Y_cal = ifft(win .* (V_cal   - mean(V_cal)),   Nfft);
phi_clean = unwrap_ref(V_clean);
phi_cal = unwrap_ref(V_cal);

figure('Name', sprintf('Validation — R = %.3f m (disp %.1f mm)', R_peak, displacement_mm(m_val)));
subplot(2,1,1);
plot(Rcm_axis/100, norm_dB(Y_raw), 'LineWidth', 1.2); hold on;
plot(Rcm_axis/100, norm_dB(Y_cln), 'LineWidth', 1.2);
plot(Rcm_axis/100, norm_dB(Y_cal), 'LineWidth', 1.4);
grid on; xlim([0 5]); ylim([-60 0]);
xlabel('Range [m]'); ylabel('Normalized power [dB]');
title(sprintf('Range Profile — Raw / Clutter-removed / Calibrated (R = %.3f m)', R_peak));
legend('Raw', '+ clutter removal', '+ calibration', 'Location', 'best');
subplot(2,1,2);
plot(f/1e9, phi_clean, 'LineWidth', 1.2); hold on;
plot(f/1e9, phi_cal,   'LineWidth', 1.4);
plot(f/1e9, phi_ideal,  '--', 'LineWidth', 1.2, 'Color', [0.5 0.5 0.5]);
grid on; xlabel('Frequency [GHz]'); ylabel('Phase [rad]');
title('Tone phase — clutter-removed vs calibrated vs ideal');
legend('Clutter-removed', 'Calibrated', sprintf('Ideal (R=%.4f m)', R_peak), 'Location', 'best');

figure('Name', 'SFCW range profile — after ICA, before vs after calibration');
plot(Rcm_axis/100, norm_dB(Y_cln), 'LineWidth', 2.0, 'Color', col_raw); hold on;
plot(Rcm_axis/100, norm_dB(Y_cal), 'LineWidth', 2.0, 'Color', col_clean);
grid on; xlim([0 min(15, Rcm_axis(end)/100)]); ylim([-60 0]);
xlabel('Range [m]'); ylabel('Normalized power [dB]');
title('SFCW range profile - after ICA, before vs after calibration');
legend('After ICA (before calibration)', 'After ICA + calibration', 'Location', 'best');
ax = gca; set(ax, 'FontSize', 16);
set([ax.XLabel ax.YLabel ax.Title], 'FontSize', 20);

V_ideal_cln = mean(abs(V_clean)) * exp(1j * phi_ideal);
figure('Name', 'SFCW constellation — clutter-removed vs calibrated vs ideal');
subplot(1,2,1);
plot(real(V_clean),     imag(V_clean),     '.', 'MarkerSize', 8); hold on;
plot(real(V_ideal_cln), imag(V_ideal_cln), '.', 'MarkerSize', 8);
grid on; xlim([-10 10]); ylim([-10 10]); axis square; xlabel('Real'); ylabel('Imag');
legend('Clutter-removed', sprintf('Ideal (R=%.4f m)', R_peak), 'Location', 'best');
title('Constellation - clutter-removed vs ideal');
subplot(1,2,2);
plot(real(V_cal),       imag(V_cal),       '.', 'MarkerSize', 8); hold on;
plot(real(V_ideal_cln), imag(V_ideal_cln), '.', 'MarkerSize', 8);
grid on; xlim([-10 10]); ylim([-10 10]); axis square; xlabel('Real'); ylabel('Imag');
legend('Calibrated', sprintf('Ideal (R=%.4f m)', R_peak), 'Location', 'best');
title('Constellation — calibrated vs ideal');

fprintf('\nDone - calibration estimated and saved.\n');
fprintf('  -> %s\n', cal_save_path);
