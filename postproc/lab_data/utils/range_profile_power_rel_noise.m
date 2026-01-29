function P_dB = range_profile_power_rel_noise(magY, Rcm, noise_range)
% RANGEPROFILEPOWERRELNOISE
% Oblicza range profile jako power [dB] względem noise floor
%
% wejścia:
%   magY         - amplituda (np. |FFT|)
%   Rcm          - oś odległości [cm]
%   noise_range  - [Rmin Rmax] zakres szumu w cm (np. [12000 14000])
%   files        - lista plików (do title)
%   N            - N FFT / chirpów
%   Rmax         - maksymalny zasięg [m]
%
% wyjście:
%   P_dB         - power relative to noise [dB]

    % --- power ---
    P = abs(magY).^2;

    % --- noise floor ---
    noise_idx = (Rcm > noise_range(1)) & (Rcm < noise_range(2));
    P0 = mean(P(noise_idx));

    % --- relative power in dB ---
    P_dB = 10*log10(P ./ (P0 + eps) + eps);



end