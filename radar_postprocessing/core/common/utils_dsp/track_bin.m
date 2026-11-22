function [position_um, error_um, rmse_um, phase_rad] = track_bin(range_profile, target_bin, ...
                                                                 wavelength, phase_sign, ...
                                                                 robot_position_um, robot_start_mm)
    target_signal = range_profile(target_bin, :).';
    phase_rad = unwrap(angle(target_signal .* conj(target_signal(1))));
    displacement_um = phase_sign * phase_rad * wavelength / (4*pi) * 1e6;
    position_um = robot_start_mm*1000 + displacement_um;
    error_um = position_um - robot_position_um;
    rmse_um = sqrt(mean(error_um.^2));
end
