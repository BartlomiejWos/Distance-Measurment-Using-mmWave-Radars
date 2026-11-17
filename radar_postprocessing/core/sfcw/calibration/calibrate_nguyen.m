function cal = calibrate_nguyen(C, f, R_vec, dg, dp)
    C_speed = 299792458;
    [N, M] = size(C);

    % Step 1: Correct differential I/Q errors
    I_c = real(C);
    Q_c = imag(C);
    I_corr = I_c;
    Q_corr = (Q_c / (1 + dg) + I_c * sin(dp)) / cos(dp);
    C_corr = I_corr + 1j * Q_corr;

    % Step 2: Common amplitude A_ref
    A_ref = mean(abs(C_corr), 2);
    A_ref = A_ref / mean(A_ref);

    % Step 3: Common phase error
    alpha_all = zeros(M, 1);
    beta_all  = zeros(N, M);

    for m = 1:M
        phi_m = unwrap(angle(C_corr(:, m)));
        phi_ideal = -(4*pi*R_vec(m)/C_speed) * (f - f(1));
        residual = phi_m - phi_ideal;
        f_centered = f - mean(f);
        p = polyfit(f_centered, residual, 1);
        alpha_all(m) = p(1) / (2*pi);
        beta_all(:, m) = residual - polyval(p, f_centered);
    end
    alpha = mean(alpha_all);
    beta  = mean(beta_all, 2);
    cal = struct('dg', dg, 'dp', dp, 'A_ref', A_ref, 'beta', beta, 'alpha', alpha);
end
