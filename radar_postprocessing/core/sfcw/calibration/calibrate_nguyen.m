function cal = calibrate_nguyen(C, f, R_vec, dg, dp)
    [N, M] = size(C);

    % Step 1: Correct differential I/Q errors
    C_corr = correct_iq_apply(C, dg, dp);

    % Step 2: Common amplitude A_ref
    A_ref = mean(abs(C_corr), 2);
    A_ref = A_ref / mean(A_ref);

    % Step 3: Common phase error
    beta_all = zeros(N, M);
    f_centered = f - mean(f);
    for m = 1:M
        phi_m = unwrap(angle(C_corr(:, m)));
        phi_ideal = -(4*pi*R_vec(m)/phys_const.C) * (f - f(1));
        residual = phi_m - phi_ideal;
        p = polyfit(f_centered, residual, 1);
        beta_all(:, m) = residual - polyval(p, f_centered);
    end
    beta = mean(beta_all, 2);
    cal = struct('dg', dg, 'dp', dp, 'A_ref', A_ref, 'beta', beta);
end
