clc; clear; close all;
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'common', 'parser'));
addpath(fullfile(here, '..', 'common', 'utils_dsp'));

cal_dir = "C:\Users\Asus\Desktop\LAB_POMIARY\final\sfcw_sweep";
prefix = 'sfcw_comparision';
cal_file = fullfile(here, 'sfcw_calibration.mat');

% Config
idx_planned = 0:74;
robot_start_mm = -12; % robot position at the first capture [mm]
step_um = 100; % robot step [um]
phase_sign = +1; % SFCW sign; opposite FMCW's -1
Nfft = 8192;
c_speed = 299792458;

fc_Hz = 77.5e9;
lambda_center = c_speed / fc_Hz;

win_fn = @hamming;
win_name = 'Hamming';
win_style = 'o-';

use_iq = false;
use_calib = true;
calib_use_iq = true;
calib_use_aref = true;

calib_dp_sign = 1;
calib_beta_sign = 1;
calib_iq_from_data = false;

% Load Data
folder_indices = detect_captures(cal_dir, prefix, idx_planned);
M = numel(folder_indices);
fprintf('Using %d of %d planned captures.\n', M, numel(idx_planned));

[C_raw, proc1] = load_tone_captures(cal_dir, prefix, folder_indices, Nfft);
N_tones = proc1.N; BW_Hz = proc1.B; R_axis = (0:Nfft-1).' * (proc1.cfg.Rmax / Nfft);
fprintf('SFCW: N=%d tones | BW=%.3f GHz | fc=%.3f GHz | dR=%.1f cm\n', ...
        N_tones, BW_Hz/1e9, fc_Hz/1e9, c_speed/(2*BW_Hz)*100);
w = win_fn(N_tones);


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
    tones_full = remove_dc_tones(apply_calib(C_raw, cal, calib_dp_sign, calib_beta_sign, calib_use_aref, true)); % IQ+CALIB
    name_iq = 'raw+IQ';
    name_full = 'raw+IQ+CALIB';
end

if use_calib
    tones_cal = remove_dc_tones(apply_calib(C_raw, cal, calib_dp_sign, calib_beta_sign, calib_use_aref, calib_use_iq));
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

RP_raw = zeros(Nfft, M); RP_cal = zeros(Nfft, M);
RP_iq = zeros(Nfft, M); RP_full = zeros(Nfft, M);
for k = 1:M
    RP_raw(:,k) = ifft(w .* tones_raw(:,k), Nfft); % reference (DC only)
    RP_cal(:,k) = ifft(w .* tones_cal(:,k), Nfft); % main corrected (per toggles)
    if have_iq
        RP_iq(:,k) = ifft(w .* tones_iq(:,k), Nfft); % IQ-only
        RP_full(:,k) = ifft(w .* tones_full(:,k), Nfft); % IQ+CALIB
    end
end

target_mask = (R_axis >= 0.5) & (R_axis <= 10); ia = find(target_mask);

gt_um = robot_start_mm*1000 + (folder_indices(:) - folder_indices(1)) * step_um;
phi_ideal = ideal_robot_phase(gt_um, lambda_center, phase_sign);
gt_mm = gt_um / 1000;

[~, kk] = max(abs(RP_raw(target_mask,1))); kt_raw = ia(kk); R_target = R_axis(kt_raw);
[~, kk] = max(abs(RP_cal(target_mask,1))); kt_cal = ia(kk);
[d_abs, err, rmse, phi] = track_bin(RP_raw, kt_raw, lambda_center, phase_sign, gt_um, robot_start_mm);
[d_abs_cal, err_cal, rmse_cal] = track_bin(RP_cal, kt_cal, lambda_center, phase_sign, gt_um, robot_start_mm);
err_iq = []; err_full = []; d_abs_iqo = []; rmse_iq = NaN; rmse_full = NaN;
if have_iq
    [~, kk] = max(abs(RP_iq(target_mask,1))); kt_iq = ia(kk);
    [~, kk] = max(abs(RP_full(target_mask,1))); kt_full = ia(kk);
    [d_abs_iqo, err_iq, rmse_iq] = track_bin(RP_iq, kt_iq, lambda_center, phase_sign, gt_um, robot_start_mm);
    [~, err_full, rmse_full] = track_bin(RP_full, kt_full, lambda_center, phase_sign, gt_um, robot_start_mm);
end

fprintf('\n[%s] error vs robot [um]\n', win_name);
fprintf('  %-14s %7s %8s %7s\n', 'pipeline', 'RMSE', 'mean', 'sigma');
fprintf('  %-14s %7.1f %+8.1f %7.1f\n', name_raw, rmse, mean(err), std(err));
if have_iq
    fprintf('  %-14s %7.1f %+8.1f %7.1f\n', name_iq, rmse_iq, mean(err_iq), std(err_iq));
    fprintf('  %-14s %7.1f %+8.1f %7.1f\n', name_full, rmse_full, mean(err_full), std(err_full));
end

figure('Name', 'SFCW - measured position vs robot');
plot(gt_mm, d_abs/1000, win_style, 'LineWidth', 1.2); hold on;
plot(gt_mm, gt_mm, '--', 'LineWidth', 1.0); grid on;
xlabel('Robot position [mm]'); ylabel('Measured [mm]');
legend(sprintf('%s (RMSE=%.1f um)', win_name, rmse), 'Robot (y=x)', 'Location', 'best');
title(sprintf('%s — fine position', win_name));


phi_cal = unwrap(angle(RP_cal(kt_cal,:).' .* conj(RP_cal(kt_cal,1))));
figure('Name', 'SFCW - robot vs raw vs calibrated signal phase');
hold on; grid on;
plot(gt_mm, phi_ideal, '--', 'LineWidth', 1.4, 'DisplayName', 'Robot phase (ideal)');
plot(gt_mm, phi, 'o-', 'LineWidth', 1.2, 'DisplayName', sprintf('Raw (%s)', name_raw));
plot(gt_mm, phi_cal, 's-', 'LineWidth', 1.2, 'DisplayName', sprintf('Calibrated (%s)', name_cal));
xlabel('Robot position [mm]'); ylabel('Phase [rad]'); legend('Location', 'best');
title('Robot vs raw vs calibrated signal phase');

figure('Name', sprintf('SFCW - residual vs robot (%s)', name_raw));
plot(gt_mm, err, win_style, 'LineWidth', 1.2);
yline(0, ':'); grid on;
xlabel('Robot position [mm]'); ylabel('Residual [um]');
legend(sprintf('%s (RMSE=%.1f um)', win_name, rmse), 'Location', 'best');
title(sprintf('%s — %s', win_name, name_raw));

if ~isempty(correction)
    figure('Name', sprintf('SFCW — residual vs robot (%s)', name_cal));
    plot(gt_mm, err_cal, win_style, 'LineWidth', 1.2);
    yline(0, ':'); grid on;
    xlabel('Robot position [mm]'); ylabel('Residual [um]');
    legend(sprintf('%s (RMSE=%.1f um)', win_name, rmse_cal), 'Location', 'best');
    title(sprintf('%s — %s', win_name, name_cal));
end

lambda_half_um = c_speed / (2*fc_Hz) * 1e6;
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

cmp = struct('radar', 'SFCW', 'xmm', gt_mm, 'err', err, 'rmse', rmse, ...
             'mean_err', mean(err), 'sigma', std(err), ...
             'd_abs', d_abs, 'phi', phi, 'phi_ideal', phi_ideal, ...
             'lambda_half_um', lambda_half_um, ...
             'gt_um', gt_um, 'step_um', step_um, 'span_um', span_um);
[cmp.Pf, cmp.ampF, cmp.Pk] = periodogram_fft(err, gt_um, step_um, span_um);
if ~isempty(correction)
    cmp.err_iq = err_cal; cmp.rmse_iq = rmse_cal; cmp.corr = correction;
    cmp.d_abs_iq = d_abs_cal;
    cmp.mean_err_iq = mean(err_cal); cmp.sigma_iq = std(err_cal);
    [cmp.Pf_iq, cmp.ampF_iq, cmp.Pk_iq] = periodogram_fft(err_cal, gt_um, step_um, span_um);
end
if have_iq && ~isempty(err_iq) % IQ-only pipeline
    cmp.err_iq_only = err_iq; cmp.rmse_iq_only = rmse_iq;
    cmp.d_abs_iq_only = d_abs_iqo;
    cmp.mean_err_iq_only = mean(err_iq); cmp.sigma_iq_only = std(err_iq);
    [cmp.Pf_iqo, cmp.ampF_iqo, cmp.Pk_iqo] = periodogram_fft(err_iq, gt_um, step_um, span_um);
end
save(fullfile(here, '..', 'analysis', 'cmp_sfcw_displacement.mat'), '-struct', 'cmp');
fprintf('Saved SFCW comparison data -> analysis/cmp_sfcw_displacement.mat\n');


function phi = ideal_robot_phase(gt_um, lambda, phase_sign)
    gt_rel_um = gt_um - gt_um(1);
    phi = phase_sign * gt_rel_um*1e-6 * 4*pi / lambda;
end

function C = remove_dc_tones(C)
    C = C - mean(C, 1);
end

function C = apply_calib(C, cal, dp_sign, beta_sign, use_aref, use_iq)
    if nargin < 5, use_aref = true; end
    if nargin < 6, use_iq = true; end
    if use_iq
        C = correct_iq_apply(C, cal.dg, dp_sign * cal.dp);
    end
    if use_aref
        C = C ./ cal.A_ref;
    end
    C = C .* exp(-1j * beta_sign * cal.beta);
end
