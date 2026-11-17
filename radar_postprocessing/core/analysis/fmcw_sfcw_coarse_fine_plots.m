% Plot-only script: loads the comparison .mat files written at:
% - core/fmcw/fmcw_displacement_analysis.m -> analysis/cmp_fmcw_displacement.mat
% - core/sfcw/sfcw_displacement_analysis.m -> analysis/cmp_sfcw_displacement.mat

clc; clear; close all;
here = fileparts(mfilename('fullpath'));

fmcw_mat = fullfile(here, 'cmp_fmcw_displacement.mat');
sfcw_mat = fullfile(here, 'cmp_sfcw_displacement.mat');
assert(isfile(fmcw_mat), 'Missing .mat %s — run fmcw_displacement_analysis.m first.', fmcw_mat);
assert(isfile(sfcw_mat), 'Missing .mat %s — run sfcw_displacement_analysis.m first.', sfcw_mat);
fmcw = load(fmcw_mat);
sfcw = load(sfcw_mat);

col_fmcw = [0.00 0.45 0.74];
col_sfcw = [0.85 0.33 0.10];
col_cal  = [0.47 0.67 0.19];

stages = {'raw'};
if isfield(fmcw, 'err_iq') && isfield(sfcw, 'err_iq')
    stages{end+1} = 'calibrated';
else
    warning(['No corrected data in one of the .mat files — plotting raw only. ' ...
             'Rerun the displacement scripts with corrections ON.']);
end

for ti = 1:numel(stages)
    stage = stages{ti};
    switch stage
        case 'raw'
            fmcw_err = fmcw.err;         sfcw_err = sfcw.err;
            fmcw_rmse = fmcw.rmse;        sfcw_rmse = sfcw.rmse;
            fmcw_mean = fmcw.mean_err;   sfcw_mean = sfcw.mean_err;
            fmcw_sigma = fmcw.sigma;      sfcw_sigma = sfcw.sigma;
            fmcw_meas = fmcw.d_abs/1000;  sfcw_meas = sfcw.d_abs/1000;
            fmcw_ampF = fmcw.ampF;   sfcw_ampF = sfcw.ampF;   fmcw_Pf = fmcw.Pf;  sfcw_Pf = sfcw.Pf;
            fmcw_Pk = fmcw.Pk;     sfcw_Pk = sfcw.Pk;
            fmcw_lab = 'FMCW';  sfcw_lab = 'SFCW';
        case 'calibrated'
            fmcw_err = fmcw.err_iq;      sfcw_err = sfcw.err_iq;
            fmcw_rmse = fmcw.rmse_iq;     sfcw_rmse = sfcw.rmse_iq;
            fmcw_mean = fmcw.mean_err_iq;  sfcw_mean = sfcw.mean_err_iq;
            fmcw_sigma = fmcw.sigma_iq;     sfcw_sigma = sfcw.sigma_iq;

            fmcw_meas = fmcw.d_abs_iq/1000;   sfcw_meas = sfcw.d_abs_iq/1000;
            fmcw_ampF = fmcw.ampF_iq; sfcw_ampF = sfcw.ampF_iq; fmcw_Pf = fmcw.Pf_iq; sfcw_Pf = sfcw.Pf_iq;
            fmcw_Pk = fmcw.Pk_iq;  sfcw_Pk = sfcw.Pk_iq;
            fmcw_lab = sprintf('FMCW + %s', fmcw.corr);   sfcw_lab = sprintf('SFCW + %s', sfcw.corr);
    end
    fprintf('%-12s %-14s RMSE=%5.1f mean=%+6.1f sigma=%5.1f um  |  %-14s RMSE=%5.1f mean=%+6.1f sigma=%5.1f um\n', ...
            sprintf('[%s]', stage), [fmcw_lab ':'], fmcw_rmse, fmcw_mean, fmcw_sigma, [sfcw_lab ':'], sfcw_rmse, sfcw_mean, sfcw_sigma);

    % 1) Residual vs robot position
    figure('Name', sprintf('Displacement residuals (%s)', stage));
    plot(fmcw.xmm, fmcw_err, '.-', 'LineWidth', 1.0, 'MarkerSize', 8, 'Color', col_fmcw); hold on;
    plot(sfcw.xmm, sfcw_err, '.-', 'LineWidth', 1.0, 'MarkerSize', 8, 'Color', col_sfcw);
    yline(0, ':'); grid on;
    xlabel('Robot arm position [mm]', 'FontSize', 14);
    ylabel('Residual error [um]', 'FontSize', 14);
    title(sprintf('Displacement residuals - FMCW vs SFCW (%s)', stage), 'FontSize', 14);
    legend(fmcw_lab, sfcw_lab, 'Location', 'best');

    % 2) Measured position vs robot
    figure('Name', sprintf('Measured positions (%s)', stage));
    plot(fmcw.xmm, fmcw_meas, '.-', 'LineWidth', 1.0, 'MarkerSize', 8, 'Color', col_fmcw); hold on;
    plot(sfcw.xmm, sfcw_meas, '.-', 'LineWidth', 1.0, 'MarkerSize', 8, 'Color', col_sfcw);
    plot(fmcw.xmm, fmcw.xmm, '--', 'LineWidth', 1.0, 'Color', [0.4 0.4 0.4]);
    grid on;
    xlabel('Robot arm position [mm]', 'FontSize', 14);
    ylabel('Measured position [mm]', 'FontSize', 14);
    title(sprintf('Measured positions - FMCW vs SFCW (%s)', stage), 'FontSize', 14);
    legend(fmcw_lab, sfcw_lab, 'Robot (y = x)', 'Location', 'best');

    % 3) Residual FFT periodogram
    figure('Name', sprintf('Residual periodograms (%s)', stage));
    plot(fmcw_Pf/1000, fmcw_ampF, 'LineWidth', 1.4, 'Color', col_fmcw); hold on;
    plot(sfcw_Pf/1000, sfcw_ampF, 'LineWidth', 1.4, 'Color', col_sfcw);
    lam_half = mean([fmcw.lambda_half_um sfcw.lambda_half_um]) / 1000;   % lambda/2 [mm]
    xline(lam_half,   '--', '\lambda/2', 'Color', 'k', 'LineWidth', 2, ...
          'FontSize', 16, 'HandleVisibility', 'off');
    xline(lam_half/2, '--', '\lambda/4', 'Color', 'k', 'LineWidth', 2, ...
          'FontSize', 16, 'HandleVisibility', 'off');
    grid on;
    xlabel('Oscillation period [mm]', 'FontSize', 14);
    ylabel('FFT amplitude [um]', 'FontSize', 14);
    title(sprintf('Residual periodograms - FMCW vs SFCW (%s)', stage), 'FontSize', 14);
    legend(fmcw_lab, sfcw_lab, 'Location', 'best');
end

% IQ only print
if isfield(fmcw, 'err_iq') && isfield(sfcw, 'err_iq_only')
    fprintf('%-12s %-14s RMSE=%5.1f mean=%+6.1f sigma=%5.1f um  |  %-14s RMSE=%5.1f mean=%+6.1f sigma=%5.1f um\n', ...
            '[iq]', 'FMCW + IQ:', fmcw.rmse_iq, fmcw.mean_err_iq, fmcw.sigma_iq, ...
            'SFCW + IQ:', sfcw.rmse_iq_only, sfcw.mean_err_iq_only, sfcw.sigma_iq_only);
end


if isfield(fmcw, 'err_iq') && isfield(sfcw, 'err_iq_only') && isfield(sfcw, 'err_iq')
    % residual
    figure('Name', 'FMCW+IQ / SFCW+IQ / SFCW+CALIB — residual');
    plot(fmcw.xmm, fmcw.err_iq,      '.-', 'LineWidth', 1.0, 'MarkerSize', 8, 'Color', col_fmcw); hold on;
    plot(sfcw.xmm, sfcw.err_iq_only, '.-', 'LineWidth', 1.0, 'MarkerSize', 8, 'Color', col_sfcw);
    plot(sfcw.xmm, sfcw.err_iq,      '.-', 'LineWidth', 1.0, 'MarkerSize', 8, 'Color', col_cal);
    yline(0, ':'); grid on;
    xlabel('Robot arm position [mm]', 'FontSize', 14);
    ylabel('Residual error [um]', 'FontSize', 14);
    title('Displacement residuals - FMCW+IQ vs SFCW+IQ vs SFCW+CALIB', 'FontSize', 14);
    legend('FMCW + IQ', 'SFCW + IQ', 'SFCW + CALIB', 'Location', 'best');

    % measured position
    figure('Name', 'FMCW+IQ / SFCW+IQ / SFCW+CALIB — measured position');
    plot(fmcw.xmm, fmcw.d_abs_iq/1000,      '.-', 'LineWidth', 1.0, 'MarkerSize', 8, 'Color', col_fmcw); hold on;
    plot(sfcw.xmm, sfcw.d_abs_iq_only/1000, '.-', 'LineWidth', 1.0, 'MarkerSize', 8, 'Color', col_sfcw);
    plot(sfcw.xmm, sfcw.d_abs_iq/1000,      '.-', 'LineWidth', 1.0, 'MarkerSize', 8, 'Color', col_cal);
    plot(fmcw.xmm, fmcw.xmm, '--', 'LineWidth', 1.0, 'Color', [0.4 0.4 0.4]);
    grid on;
    xlabel('Robot arm position [mm]', 'FontSize', 14);
    ylabel('Measured position [mm]', 'FontSize', 14);
    title('Measured positions - FMCW+IQ vs SFCW+IQ vs SFCW+CALIB', 'FontSize', 14);
    legend('FMCW + IQ', 'SFCW + IQ', 'SFCW + CALIB', 'Robot (y = x)', 'Location', 'best');

    % periodogram
    figure('Name', 'FMCW+IQ / SFCW+IQ / SFCW+CALIB — periodogram');
    plot(fmcw.Pf_iq/1000,  fmcw.ampF_iq,  'LineWidth', 1.4, 'Color', col_fmcw); hold on;
    plot(sfcw.Pf_iqo/1000, sfcw.ampF_iqo, 'LineWidth', 1.4, 'Color', col_sfcw);
    plot(sfcw.Pf_iq/1000,  sfcw.ampF_iq,  'LineWidth', 1.4, 'Color', col_cal);
    lam_half = mean([fmcw.lambda_half_um sfcw.lambda_half_um]) / 1000;
    xline(lam_half,   '--', '\lambda/2', 'Color', 'k', 'LineWidth', 2, ...
          'FontSize', 16, 'HandleVisibility', 'off');
    xline(lam_half/2, '--', '\lambda/4', 'Color', 'k', 'LineWidth', 2, ...
          'FontSize', 16, 'HandleVisibility', 'off');
    grid on;
    xlabel('Oscillation period [mm]', 'FontSize', 14);
    ylabel('FFT amplitude [um]', 'FontSize', 14);
    title('Residual periodograms - FMCW+IQ vs SFCW+IQ vs SFCW+CALIB', 'FontSize', 14);
    legend('FMCW + IQ', 'SFCW + IQ', 'SFCW + CALIB', 'Location', 'best');
end


for fg = findall(0, 'Type', 'figure')'
    ax = findall(fg, 'Type', 'axes');
    set(ax, 'FontSize', 16);
    set(findall(fg, 'Type', 'line'), 'LineWidth', 2.0);
    for a = ax(:)'
        set([a.XLabel a.YLabel a.Title], 'FontSize', 20);
    end
end
