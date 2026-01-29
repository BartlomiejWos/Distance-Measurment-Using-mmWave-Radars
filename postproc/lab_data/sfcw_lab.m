%% SFCW (1xTX): profil odległości i faza z wielu plików (tu 3)
clc; clear; close all;

% =========================
% 0) Konfiguracja
% =========================
files = { ...
    "sfcw_measurment_77-77_5GHz-500_chirps-RF_f000_Raw_0.bin", ...
    "sfcw_measurment_77-77_5GHz-500_chirps-RF_f001_Raw_0.bin", ...
};

numADCSamples = 256;    % próbek ADC na ton
numADCBits    = 16;     % bity ADC
numRX         = 4;      % liczba odbiorników
isReal        = false;  % dane zespolone?

C      = 299792458;     % [m/s]
df_Hz  = 1e6;           % krok częstotliwości [Hz]
Nfft   = 8192;          % rozmiar IFFT (gęstość siatki R)

% =========================
% 1) Wczytanie i uśrednienie po „fast-time”
% =========================
V_parts = cell(numel(files),1);
N_parts = zeros(numel(files),1);

for k = 1:numel(files)
    [rawk, Nchirpk] = rawDataParser(files{k}, numADCSamples, numADCBits, numRX, isReal);
    rx_sum_k = sum(rawk, 1) / numRX;                         % stack RX
    rx_mat_k = reshape(rx_sum_k, [numADCSamples, Nchirpk]);  % [Ns x Ntones]
    V_parts{k} = mean(rx_mat_k, 1).';                        % [Ntones_k x 1]
    N_parts(k) = numel(V_parts{k});
end

% Zlepienie wszystkich części
V = vertcat(V_parts{:});

% usunięcie DC
V = V - mean(V);

% =========================
% 2) Profil odległości (IFFT po f)
% =========================
N = numel(V);
Rmax = C / (2*df_Hz);      % z df
B    = N * df_Hz;          % całkowite pasmo
Rres = C / (2*B);          % rozdzielczość zasięgowa

w     = hann(N);
Y     = ifft(w .* V, Nfft);
magY  = abs(Y);
R     = (0:Nfft-1).' * (Rmax / Nfft);
Rcm   = R * 100;


fprintf('Plików: %d | Tónów: %d\n', numel(files), N);
fprintf('Rmax   = %.3f m\n', Rmax);
fprintf('Rres   = %.2f cm\n', Rres*100);

P_dB = range_profile_power_rel_noise( ...
    magY, ...
    Rcm, ...
    [12e3 14e3]);

figure;
plot(Rcm / 100, P_dB, 'LineWidth', 1.2); grid on;
xlabel('Range [m]');
ylabel('Power relative to noise [dB]');
xlim([0 10])
title(sprintf('Range Profile (SFCW) — %d Files, N=%d', numel(files), N));

% =========================
% 3) Faza vs częstotliwość 
% =========================
f = generate_frequency_vector(77e9,files,N, N_parts, df_Hz);
phase_unw = unwrap(angle(V));

% dphi = angle(V(2:end).*conj(V(1:end-1)));
% dphi_unw = unwrap(dphi);
% phi_unw = [0; cumsum(dphi_unw)];

R0 = 1.7223;
f0 = f(1);
phi_ideal = -(4*pi*R0/C) * (f - f0);

figure;
k = 450;
x_sep = (f(k))/1e9;
xline(x_sep, '--', 'LineWidth', 1.2); hold on;
plot(f/1e9, phase_unw, 'LineWidth', 1.2); 
plot(f/1e9, phi_ideal, 'k--', 'LineWidth', 1.2); grid on;
xlabel('Frequency [GHz]'); ylabel('Phase [rad]');
title(sprintf('Phase vs. f - Full Range'));
legend('Frame Separator','Measured', 'Ideal', 'Location','best');

%%
% ====== Phase Isolated (range-gate) ======

Vw = V(:).*w;
Yw = ifft(Vw, Nfft);

% range gate: 1 – 15 m
Rmin = 1.2;
Rmax = 100.0;
gate = (R >= Rmin) & (R <= Rmax);

Yg = zeros(size(Yw));
Yg(gate) = Yw(gate);

Vg = fft(Yg, Nfft);
Vg = Vg(1:N);

phase_unw_iso = unwrap(angle(Vg));
phase_unw_iso_final = phase_unw_iso - phase_unw_iso(1);

figure;
k = 450;
x_sep = (f(k))/1e9;
xline(x_sep, '--', 'LineWidth', 1.2); hold on;
plot(f/1e9, phase_unw_iso_final, 'LineWidth', 1.2); 
plot(f/1e9, phi_ideal, 'k--', 'LineWidth', 1.2); grid on;
xlabel('Frequency [GHz]'); ylabel('Phase [rad]');
title(sprintf('Phase vs. f - Isolated Range'));
legend('Frame Separator','Measured', 'Ideal', 'Location','best');


%% Apply HFP in Range Domain

Y_hp = range_highpass(Y, Rcm, 95, 105);
magY_hp = abs(Y_hp);

P_dB = range_profile_power_rel_noise( ...
    magY_hp, ...
    Rcm, ...
    [12e3 14e3]);
figure;
plot(Rcm / 100, P_dB, 'LineWidth', 1.2); grid on;
xlabel('Range [m]');
ylabel('Power relative to noise [dB]');
xlim([0 10])
title(sprintf('Range Profile (SFCW) + HFP — %d Files, N=%d', numel(files), N));
