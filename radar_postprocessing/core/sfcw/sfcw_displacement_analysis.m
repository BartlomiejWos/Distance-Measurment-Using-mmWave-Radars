clc; clear; close all;
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, 'calibration'));
addpath(fullfile(here, '..', 'common', 'parser'));
addpath(fullfile(here, '..', 'common', 'utils_dsp'));

cal_dir = fullfile(here, '..', '..', 'measurement_data', 'sfcw_sweep');
prefix = 'sfcw_comparision';
cal_file = fullfile(here, 'calibration', 'sfcw_calibration.mat');

% Config
idx_planned = 0:74;
robot_start_mm = -12; % robot position at the first capture [mm]
step_um = 100; % robot step [um]
R_target = 1.15; % [m] nominal target distance
Nfft = 8192;

win_fn = @hamming;
win_name = func2str(win_fn);

use_iq = false;
use_calib = true;
calib_use_iq = true;
calib_use_aref = true;
calib_iq_from_data = false;

% Load Data
folder_indices = detect_captures(cal_dir, prefix, idx_planned);
M = numel(folder_indices);
fprintf('Using %d of %d planned captures.\n', M, numel(idx_planned));

[C_raw, proc1] = load_tone_captures(cal_dir, prefix, folder_indices, Nfft);
N_tones = proc1.N; BW_Hz = proc1.BW; R_axis = proc1.R;
fc_Hz = proc1.f(1) + BW_Hz/2;
lambda_center = phys_const.C / fc_Hz;
fprintf('SFCW: N=%d tones | BW=%.3f GHz | fc=%.3f GHz | dR=%.1f cm\n', ...
        N_tones, BW_Hz/1e9, fc_Hz/1e9, proc1.Rres*100);
win = win_fn(N_tones);


% Pipeline
tones_raw = remove_dc_tones(C_raw);

have_iq = use_calib || use_iq;
if have_iq
    if ~exist(cal_file, 'file')
        error('sfcw_displacement:noCal', ...
              'Calibration file not found: %s (run run_sfcw_calib first).', cal_file);
    end
    cal = load(cal_file);
    if calib_iq_from_data
        [cal.dg, cal.dp] = correct_iq_differential(tones_raw);
        fprintf('IQ (from data): dg=%.4f, dp=%.2f deg\n', cal.dg, rad2deg(cal.dp));
    end

    tones_iq = remove_dc_tones(correct_iq_apply(C_raw, cal.dg, cal.dp)); % IQ only
    tones_full = remove_dc_tones(apply_calib(C_raw, cal, calib_use_aref, true)); % IQ+CALIB
    name_iq = 'raw+IQ';
    name_full = 'raw+IQ+CALIB';
end

if use_calib
    tones_cal = remove_dc_tones(apply_calib(C_raw, cal, calib_use_aref, calib_use_iq));
    correction = 'CALIB';
elseif use_iq
    tones_cal = remove_dc_tones(correct_iq_apply(C_raw, cal.dg, cal.dp));
    correction = 'IQ';
else
    tones_cal = tones_raw;
    correction = '';
end
name_raw = 'raw';
if isempty(correction), name_cal = name_raw; else, name_cal = ['raw+' correction]; end
if ~isempty(correction)
    fprintf('Correction: %s | dg=%.4f, dp=%.2f deg\n', correction, cal.dg, rad2deg(cal.dp));
end

range_profile_raw = ifft(win .* tones_raw, Nfft); % reference (DC only)
range_profile_cal = ifft(win .* tones_cal, Nfft); % main corrected (per toggles)
if have_iq
    range_profile_iq = ifft(win .* tones_iq, Nfft); % IQ-only
    range_profile_full = ifft(win .* tones_full, Nfft); % IQ+CALIB
end

gt_um = robot_start_mm*1000 + (folder_indices(:) - folder_indices(1)) * step_um;
phi_ideal = ideal_robot_phase(gt_um, lambda_center);
gt_mm = gt_um / 1000;

target_bin_raw = peak_bin(range_profile_raw, R_axis, R_target);
target_bin_cal = peak_bin(range_profile_cal, R_axis, R_target);
[d_abs, err, rmse, phi] = track_bin(range_profile_raw, target_bin_raw, lambda_center, +1, gt_um, robot_start_mm);
[d_abs_cal, err_cal, rmse_cal] = track_bin(range_profile_cal, target_bin_cal, lambda_center, +1, gt_um, robot_start_mm);
err_iq = []; err_full = []; d_abs_iqo = []; rmse_iq = NaN; rmse_full = NaN;
if have_iq
    target_bin_iq = peak_bin(range_profile_iq, R_axis, R_target);
    target_bin_full = peak_bin(range_profile_full, R_axis, R_target);
    [d_abs_iqo, err_iq, rmse_iq] = track_bin(range_profile_iq, target_bin_iq, lambda_center, +1, gt_um, robot_start_mm);
    [~, err_full, rmse_full] = track_bin(range_profile_full, target_bin_full, lambda_center, +1, gt_um, robot_start_mm);
end

fprintf('\n[%s] error vs robot [um]\n', win_name);
fprintf('  %-14s %7s %8s %7s\n', 'pipeline', 'RMSE', 'mean', 'sigma');
fprintf('  %-14s %7.1f %+8.1f %7.1f\n', name_raw, rmse, mean(err), std(err));
if have_iq
    fprintf('  %-14s %7.1f %+8.1f %7.1f\n', name_iq, rmse_iq, mean(err_iq), std(err_iq));
    fprintf('  %-14s %7.1f %+8.1f %7.1f\n', name_full, rmse_full, mean(err_full), std(err_full));
end

figure('Name', 'SFCW - measured position vs robot');
plot(gt_mm, d_abs/1000, 'o-','LineWidth', 1.2); hold on;
plot(gt_mm, gt_mm, '--', 'LineWidth', 1.0); grid on;
xlabel('Robot position [mm]'); ylabel('Measured [mm]');
legend(sprintf('%s (RMSE=%.1f um)', win_name, rmse), 'Robot (y=x)', 'Location', 'best');
title(sprintf('%s — fine position', win_name));


phi_cal = unwrap(angle(range_profile_cal(target_bin_cal,:).' .* conj(range_profile_cal(target_bin_cal,1))));
figure('Name', 'SFCW - robot vs raw vs calibrated signal phase');
hold on; grid on;
plot(gt_mm, phi_ideal, '--', 'LineWidth', 1.4, 'DisplayName', 'Robot phase (ideal)');
plot(gt_mm, phi, 'o-', 'LineWidth', 1.2, 'DisplayName', sprintf('Raw (%s)', name_raw));
plot(gt_mm, phi_cal, 's-', 'LineWidth', 1.2, 'DisplayName', sprintf('Calibrated (%s)', name_cal));
xlabel('Robot position [mm]'); ylabel('Phase [rad]'); legend('Location', 'best');
title('Robot vs raw vs calibrated signal phase');

figure('Name', sprintf('SFCW - residual vs robot (%s)', name_raw));
plot(gt_mm, err, 'o-','LineWidth', 1.2);
yline(0, ':'); grid on;
xlabel('Robot position [mm]'); ylabel('Residual [um]');
legend(sprintf('%s (RMSE=%.1f um)', win_name, rmse), 'Location', 'best');
title(sprintf('%s — %s', win_name, name_raw));

if ~isempty(correction)
    figure('Name', sprintf('SFCW — residual vs robot (%s)', name_cal));
    plot(gt_mm, err_cal, 'o-','LineWidth', 1.2);
    yline(0, ':'); grid on;
    xlabel('Robot position [mm]'); ylabel('Residual [um]');
    legend(sprintf('%s (RMSE=%.1f um)', win_name, rmse_cal), 'Location', 'best');
    title(sprintf('%s — %s', win_name, name_cal));
end

lambda_half_um = phys_const.C / (2*fc_Hz) * 1e6;
span_um = max(gt_um) - min(gt_um);
[Pf, ampF, Pk] = periodogram_fft(err, gt_um, step_um, span_um);
figure('Name', 'SFCW — residual FFT periodogram'); hold on; grid on;
plot(Pf/1000, ampF, 'LineWidth', 1.4, 'DisplayName', sprintf('%s (peak %.0f um)', name_raw, Pk));
if ~isempty(correction)
    [Pf_c, ampF_c, Pk_c] = periodogram_fft(err_cal, gt_um, step_um, span_um);
    plot(Pf_c/1000, ampF_c, 'LineWidth', 1.4, 'DisplayName', sprintf('%s (peak %.0f um)', name_cal, Pk_c));
end
xline(lambda_half_um/1000, '--', '\lambda/2');
xlabel('Period [mm]'); ylabel('FFT amplitude [um]'); legend('Location', 'best');
title(sprintf('Residual periodogram — \\lambda/2 = %.2f mm', lambda_half_um/1000));

cmp = struct('xmm', gt_mm, 'err', err, 'rmse', rmse, ...
             'mean_err', mean(err), 'sigma', std(err), ...
             'd_abs', d_abs, 'lambda_half_um', lambda_half_um);
cmp.Pf = Pf; cmp.ampF = ampF;
if ~isempty(correction)
    cmp.err_iq = err_cal; cmp.rmse_iq = rmse_cal; cmp.corr = correction;
    cmp.d_abs_iq = d_abs_cal;
    cmp.mean_err_iq = mean(err_cal); cmp.sigma_iq = std(err_cal);
    cmp.Pf_iq = Pf_c; cmp.ampF_iq = ampF_c;
end
if have_iq && ~isempty(err_iq) % IQ-only pipeline
    cmp.err_iq_only = err_iq; cmp.rmse_iq_only = rmse_iq;
    cmp.d_abs_iq_only = d_abs_iqo;
    cmp.mean_err_iq_only = mean(err_iq); cmp.sigma_iq_only = std(err_iq);
    [cmp.Pf_iqo, cmp.ampF_iqo] = periodogram_fft(err_iq, gt_um, step_um, span_um);
end
save(fullfile(here, '..', 'analysis', 'cmp_sfcw_displacement.mat'), '-struct', 'cmp');
fprintf('Saved SFCW comparison data -> analysis/cmp_sfcw_displacement.mat\n');


function phi = ideal_robot_phase(gt_um, lambda)
    gt_rel_um = gt_um - gt_um(1);
    phi = gt_rel_um*1e-6 * 4*pi / lambda;
end

function C = remove_dc_tones(C)
    C = C - mean(C, 1);
end
