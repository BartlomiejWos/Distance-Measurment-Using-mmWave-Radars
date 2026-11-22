function C_clean = remove_clutter_ica(C, Nfft, Rmax, Rcut_cm, ratio_thresh)
    [N, M] = size(C);
    n_comp = min(M - 1, 5);  % number of ICA components

    % Step 1: Stack real/imag → real matrix [2N x M]
    X = [real(C); imag(C)];
    X_mean = mean(X, 2);
    Xc = X - X_mean;

    % Step 2: SVD for dimensionality reduction ---
    [U, Sigma, ~] = svd(Xc, 'econ');
    d = diag(Sigma);
    Z = diag(1 ./ d(1:n_comp)) * U(:, 1:n_comp)' * Xc;  % [n_comp x M]

    % Step 3: FastICA (deflation, tanh nonlinearity)
    W_ica = zeros(n_comp, n_comp);
    rng_state = rng;   % fixed seed for reproducible ICA, restored afterwards
    rng(42);
    for p = 1:n_comp
        w = randn(n_comp, 1);
        w = w / norm(w);

        for iter = 1:200
            u = w' * Z;
            g = tanh(u);
            gp = 1 - g.^2;

            w_new = Z * g' / M - mean(gp) * w;

            % Deflation: orthogonalize against previous components
            for j = 1:p-1
                w_new = w_new - dot(w_new, W_ica(j,:)') * W_ica(j,:)';
            end

            w_new = w_new / norm(w_new);

            if abs(abs(dot(w_new, w)) - 1) < 1e-8
                break;
            end
            w = w_new;
        end
        W_ica(p, :) = w_new';
    end
    rng(rng_state);

    % Step 4: Sources and mixing matrix
    S = W_ica * Z;
    A = U(:, 1:n_comp) * diag(d(1:n_comp)) * W_ica';

    % Step 5: Identify clutter by range profile energy ---
    R_axis_cm = (0:Nfft-1)' * (Rmax / Nfft) * 100;

    is_clutter = false(n_comp, 1);
    ratio_all = zeros(n_comp, 1);
    for p = 1:n_comp
        % Reconstruct this component's contribution to the signal
        X_p = A(:, p) * S(p, :);  % [2N x M]
        % Use first position as representative
        V_p = X_p(1:N, 1) + 1j * X_p(N+1:end, 1);

        % Range profile
        Y_p = ifft(V_p, Nfft);
        Y_half = Y_p(1:Nfft/2);
        R_half = R_axis_cm(1:Nfft/2);

        energy_total = sum(abs(Y_half).^2);
        energy_near = sum(abs(Y_half(R_half < Rcut_cm)).^2);
        ratio = energy_near / energy_total;
        ratio_all(p) = ratio;

        fprintf('  IC %d: near-range energy = %.1f%%\n', p, ratio * 100);

        if ratio > ratio_thresh
            is_clutter(p) = true;
        end
    end

    % Rank guard: keep >=2 most target-like components
    min_keep = min(2, n_comp);
    n_survive = sum(~is_clutter);
    if n_survive < min_keep
        [~, order] = sort(ratio_all, 'ascend');   % most target-like first
        is_clutter(order(1:min_keep)) = false;
        fprintf(['  ICA guard: only %d component(s) would survive; forcing the ' ...
                 '%d most target-like to be kept (avoids rank-1 IQ collapse / dp=+-90)\n'], ...
                 n_survive, min_keep);
    end

    fprintf('ICA: %d components, %d identified as clutter (Rcut=%d cm)\n', ...
        n_comp, sum(is_clutter), Rcut_cm);

    % Step 6: Reconstruct without clutter ---
    target_idx = find(~is_clutter);
    X_clean = zeros(2*N, M);
    for j = target_idx'
        X_clean = X_clean + A(:, j) * S(j, :);
    end

    C_clean = X_clean(1:N, :) + 1j * X_clean(N+1:end, :);
end
