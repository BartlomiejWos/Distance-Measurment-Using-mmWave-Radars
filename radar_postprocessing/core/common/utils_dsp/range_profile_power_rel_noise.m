function P_dB = range_profile_power_rel_noise(magY, Rcm, noise_range)
    P = abs(magY).^2;
    noise_idx = (Rcm > noise_range(1)) & (Rcm < noise_range(2));
    P0 = mean(P(noise_idx));
    P_dB = 10*log10(P ./ (P0 + eps) + eps);
end