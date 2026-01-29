function [f] = generate_frequency_vector(f0_first, files, N, N_parts, df_Hz)
    f0_first = 77e9;
    f = zeros(N,1);
    idx0 = 1;
    for k = 1:numel(files)
        N_k = N_parts(k);
        if k == 1
            f(idx0:idx0+N_k-1) = f0_first + (0:N_k-1)'*df_Hz;
        else
            f(idx0:idx0+N_k-1) = f(idx0-1) + (1:N_k)'*df_Hz;  % ciągłość co df
        end
        idx0 = idx0 + N_k;
    end
end

