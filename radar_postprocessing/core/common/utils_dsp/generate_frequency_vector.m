function [f] = generate_frequency_vector(f0_first, N, N_parts, df_Hz)
    f = zeros(N, 1);
    idx0 = 1;
    for k = 1:numel(N_parts)
        N_k = N_parts(k);
        if k == 1
            f(idx0:idx0+N_k-1) = f0_first + (0:N_k-1)' * df_Hz;
        else
            f(idx0:idx0+N_k-1) = f(idx0-1) + (1:N_k)' * df_Hz;
        end
        idx0 = idx0 + N_k;
    end
end

