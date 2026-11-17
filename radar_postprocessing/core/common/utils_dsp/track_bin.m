function [d_abs, err, rmse, phi] = track_bin(RP, k_target, lambda, phase_sign, gt_um, robot_start_mm)
    Y     = RP(k_target, :).';
    phi   = unwrap(angle(Y .* conj(Y(1))));
    d_rel = phase_sign * phi * lambda / (4*pi) * 1e6;
    d_abs = robot_start_mm*1000 + d_rel;
    err   = d_abs - gt_um;
    rmse  = sqrt(mean(err.^2));
end
