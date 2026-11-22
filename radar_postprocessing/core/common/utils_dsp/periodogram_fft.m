function [period_um, amplitude_um, peak_period_um] = periodogram_fft(res_um, robot_position_um, step_um, span_um)
    [pos_sorted_um, sort_idx] = sort(robot_position_um(:));
    if any(abs(diff(pos_sorted_um) - abs(step_um)) > 1e-6)
        error('periodogram_fft:gap', ...
              'Robot positions are not uniformly spaced by %g um (missing capture?)', abs(step_um));
    end
    res_detrended_um = detrend(res_um(sort_idx));
    hann_window = hann(numel(res_detrended_um));
    nfft = 2^nextpow2(32*numel(res_detrended_um));
    spectrum = fft(res_detrended_um .* hann_window, nfft);
    amplitude_all_um = 2*abs(spectrum(1:floor(nfft/2))) / sum(hann_window);
    bin_idx = (0:floor(nfft/2)-1).';
    period_all_um = 1 ./ max(bin_idx/(abs(step_um)*nfft), eps);
    valid = (period_all_um <= 0.9*span_um) & (period_all_um >= 2.2*abs(step_um));
    [period_um, order] = sort(period_all_um(valid));
    amplitude_valid_um = amplitude_all_um(valid);
    amplitude_um = amplitude_valid_um(order);
    [~, peak_idx] = max(amplitude_um);
    peak_period_um = period_um(peak_idx);
end
