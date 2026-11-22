function [dg, dp] = correct_iq_differential(C)
    [N, M] = size(C);

    if M < 4
        error('Need at least 4 positions for Gram-Schmidt (have %d)', M);
    end

    E1_all = zeros(N, 1);
    P_all = zeros(N, 1);

    for k = 1:N
        % DC removal
        I2 = real(C(k, :)).' - mean(real(C(k, :)));
        Q2 = imag(C(k, :)).' - mean(imag(C(k, :)));

        % P: projection of Q onto I, negated
        P_k = -dot(I2, Q2) / dot(I2, I2);

        % Q3: orthogonalized Q
        Q3 = P_k * I2 + Q2;

        % E1: amplitude ratio
        E1_k = norm(Q3) / norm(I2);

        E1_all(k) = E1_k;
        P_all(k) = P_k;
    end

    % Scalar constants
    E1 = median(E1_all);
    P = median(P_all);

    dp = atan(P / E1);
    dg = sqrt(E1^2 + P^2) - 1;
end
