clc; clear; close all;
here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, '..', 'common', 'parser'));
addpath(fullfile(here, '..', 'common', 'utils_dsp'));

% Config
cal_dir = fullfile(here, '..', '..', 'measurement_data', 'fmcw_sweep');
prefix = 'fmcw_comparision';
idx_planned = 0:74;
robot_start_mm = -12;
step_um = 100;
R_target = 1.15; % [m] nominal target distance
n_chirps = 128;
Fs = 7.2e6;
Nfft = 8192;

win_fn = @hamming;
win_name = func2str(win_fn);

use_iq = true;
calc_iq_from_peak = false;

% Load Data
folder_indices = detect_captures(cal_dir, prefix, idx_planned);
M = numel(folder_indices);
fprintf('Using %d of %d planned captures.\n', M, numel(idx_planned));

[beat_avg, R_axis, N_adc, BW_Hz, f_center] = load_beat_captures(cal_dir, prefix, folder_indices, Fs, Nfft, n_chirps);
lambda_center = phys_const.C / f_center;
win = win_fn(N_adc);

% Pipeline
beat_raw = beat_avg - mean(beat_avg, 1); % reference: DC removed (after chirp averaging)

if calc_iq_from_peak
    range_profile_for_iq = beat_range_profile(beat_raw, win, Nfft);
    target_bin_for_iq = peak_bin(range_profile_for_iq, R_axis, R_target);
    [dg, dp] = correct_iq_differential(range_profile_for_iq(target_bin_for_iq, :));
    fprintf('IQ (from target bin k=%d): dg=%.4f, dp=%.2f deg\n', target_bin_for_iq, dg, rad2deg(dp));
else
    [dg, dp] = correct_iq_differential(beat_raw(:).');
    fprintf('IQ imbalance: dg=%.4f, dp=%.2f deg\n', dg, rad2deg(dp));
end

if use_iq
    beat_iq = correct_iq_apply(beat_raw, dg, dp); % corrected: DC + IQ
    correction = 'IQ';
else
    beat_iq = beat_raw;
    correction = '';
end
name_raw = 'raw';
if isempty(correction), name_iq = name_raw; else, name_iq = ['raw+' correction]; end

range_profile_raw = beat_range_profile(beat_raw, win, Nfft);
range_profile_iq = beat_range_profile(beat_iq, win, Nfft);

gt_um = robot_start_mm*1000 + (folder_indices(:) - folder_indices(1)) * step_um;
phi_ideal = ideal_robot_phase(gt_um, lambda_center);
xmm = gt_um / 1000;

target_bin_raw = peak_bin(range_profile_raw, R_axis, R_target);
target_bin_iq = peak_bin(range_profile_iq, R_axis, R_target);
[d_abs, err, rmse, phi] = track_bin(range_profile_raw, target_bin_raw, lambda_center, -1, gt_um, robot_start_mm);
[d_abs_iq, err_iq, rmse_iq] = track_bin(range_profile_iq, target_bin_iq, lambda_center, -1, gt_um, robot_start_mm);

fprintf('\n[%s] error vs robot [um]\n', win_name);
fprintf('  %-10s %7s %8s %7s\n', 'pipeline', 'RMSE', 'mean', 'sigma');
fprintf('  %-10s %7.1f %+8.1f %7.1f\n', name_raw, rmse, mean(err), std(err));
if use_iq
    fprintf('  %-10s %7.1f %+8.1f %7.1f\n', name_iq, rmse_iq, mean(err_iq), std(err_iq));
end

figure('Name', 'FMCW - measured position vs robot');
plot(xmm, d_abs/1000, 'o-', 'LineWidth', 1.2); hold on;
plot(xmm, xmm, '--', 'LineWidth', 1.0); grid on;
xlabel('Robot position [mm]'); ylabel('Measured [mm]');
legend(sprintf('%s (RMSE=%.1f um)', win_name, rmse), 'Robot (y=x)', 'Location', 'best');
title(sprintf('%s — fine position', win_name));

phi_iq = unwrap(angle(range_profile_iq(target_bin_iq,:).' .* conj(range_profile_iq(target_bin_iq,1))));
figure('Name', 'FMCW - robot vs raw vs corrected signal phase');
hold on; grid on;
plot(xmm, phi_ideal, '--', 'LineWidth', 1.4, 'DisplayName', 'Robot phase (ideal)');
plot(xmm, phi, 'o-', 'LineWidth', 1.2, 'DisplayName', sprintf('Raw (%s)', name_raw));
plot(xmm, phi_iq, 's-', 'LineWidth', 1.2, 'DisplayName', sprintf('Corrected (%s)', name_iq));
xlabel('Robot position [mm]'); ylabel('Phase [rad]'); legend('Location', 'best');
title('Robot vs raw vs corrected signal phase');

figure('Name', sprintf('FMCW - residual vs robot (%s)', name_raw));
plot(xmm, err, 'o-', 'LineWidth', 1.2);
yline(0, ':'); grid on;
xlabel('Robot position [mm]'); ylabel('Residual [um]');
legend(sprintf('%s (RMSE=%.1f um)', win_name, rmse), 'Location', 'best');
title(sprintf('%s — %s', win_name, name_raw));

if use_iq
    figure('Name', sprintf('FMCW - residual vs robot (%s)', name_iq));
    plot(xmm, err_iq, 'o-', 'LineWidth', 1.2);
    yline(0, ':'); grid on;
    xlabel('Robot position [mm]'); ylabel('Residual [um]');
    legend(sprintf('%s (RMSE=%.1f um)', win_name, rmse_iq), 'Location', 'best');
    title(sprintf('%s — %s', win_name, name_iq));
end

lambda_half_um = phys_const.C / (2*f_center) * 1e6;
span_um = max(gt_um) - min(gt_um);
[Pf, ampF, Pk] = periodogram_fft(err, gt_um, step_um, span_um);
figure('Name', 'FMCW - residual FFT periodogram'); hold on; grid on;
plot(Pf/1000, ampF, 'LineWidth', 1.4, 'DisplayName', sprintf('%s (peak %.0f um)', name_raw, Pk));
if use_iq
    [Pf_c, ampF_c, Pk_c] = periodogram_fft(err_iq, gt_um, step_um, span_um);
    plot(Pf_c/1000, ampF_c, 'LineWidth', 1.4, 'DisplayName', sprintf('%s (peak %.0f um)', name_iq, Pk_c));
end
xline(lambda_half_um/1000, '--', '\lambda/2');
xlabel('Period [mm]'); ylabel('FFT amplitude [um]'); legend('Location', 'best');
title(sprintf('Residual periodogram — \\lambda/2 = %.2f mm', lambda_half_um/1000));

cmp = struct('xmm', xmm, 'err', err, 'rmse', rmse, ...
             'mean_err', mean(err), 'sigma', std(err), ...
             'd_abs', d_abs, 'lambda_half_um', lambda_half_um);
cmp.Pf = Pf; cmp.ampF = ampF;
if use_iq
    cmp.err_iq = err_iq; cmp.rmse_iq = rmse_iq; cmp.corr = 'IQ';
    cmp.d_abs_iq = d_abs_iq;
    cmp.mean_err_iq = mean(err_iq); cmp.sigma_iq = std(err_iq);
    cmp.Pf_iq = Pf_c; cmp.ampF_iq = ampF_c;
end
save(fullfile(here, '..', 'analysis', 'cmp_fmcw_displacement.mat'), '-struct', 'cmp');
fprintf('Saved FMCW comparison data -> analysis/cmp_fmcw_displacement.mat\n');


function [beat_avg, R_axis, N_adc, BW_Hz, f_center] = load_beat_captures(cal_dir, prefix, folder_indices, Fs, Nfft, n_chirps)
    M = numel(folder_indices);
    beat_avg = [];
    for k = 1:M
        folder = fullfile(cal_dir, sprintf('%s_%03d', prefix, folder_indices(k)));
        proc = FmcwProcessor(folder, Fs, Nfft);
        if k == 1
            f_start = proc.f0; slope = proc.slope; N_adc = proc.N; BW_Hz = proc.BW;
            f_center = f_start + BW_Hz/2;
            R_axis = proc.R;
            fprintf('Radar: f_start=%.3f GHz | slope=%.2f MHz/us | N=%d | BW=%.3f GHz\n', ...
                    f_start/1e9, slope/1e12, N_adc, BW_Hz/1e9);
            beat_avg = zeros(N_adc, M);
        end
        nu = min(n_chirps, size(proc.X, 2));
        beat_avg(:, k) = mean(proc.X(:, 1:nu), 2);
    end
end

function RP = beat_range_profile(X, win, Nfft)
    M = size(X, 2);
    RP = zeros(Nfft/2, M);
    for k = 1:M
        h = fft(win .* X(:,k), Nfft);
        RP(:,k) = h(1:Nfft/2);
    end
end

function phi = ideal_robot_phase(gt_um, lambda)
    gt_rel_um = gt_um - gt_um(1);
    phi = -gt_rel_um*1e-6 * 4*pi / lambda;
end
