%% SFCW (1xTX): profil odległości i faza z wielu plików (tu 3)
clc; clear; close all;

% =========================
% 0) Konfiguracja
% =========================
files = { ...
    "sfcw_512_0875MHz_part1_Raw_0.bin", ...
     "sfcw_512_0875MHz_part2_Raw_0.bin", ...
    % "sfcw_512_0875MHz_part3_Raw_0.bin" ...
};

numADCSamples = 256;    % próbek ADC na ton
numADCBits    = 16;     % bity ADC
numRX         = 4;      % liczba odbiorników
isReal        = false;  % dane zespolone?

C      = 299792458;     % [m/s]
df_Hz  = 0.875e6;       % krok częstotliwości [Hz]

% --- Startowe f0 dla części
% tryb 1 (domyślny): jedna wartość -> kolejne pliki kontynuują o df_Hz
f0_first = 77e9;

% tryb 2 (opcjonalny): jawnie różne f0 dla każdej części (odkomentuj i wpisz)
% f0_parts = [77.000e9, 77.448e9, 77.896e9];  % przykładowo

Ns_use = 128;           % ile próbek „fast-time” użyć do uśredniania
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
    rx_mat_k = rx_mat_k(1:128, :);                        % ewentualne skrócenie
    V_parts{k} = mean(rx_mat_k, 1).';                        % [Ntones_k x 1]
    N_parts(k) = numel(V_parts{k});
end

% Zlepienie wszystkich części
V = vertcat(V_parts{:});
N = numel(V);

% usunięcie DC
V = V - mean(V);

% =========================
% 2) Oś częstotliwości
% =========================
% a) Automatyczna kontynuacja od f0_first co df_Hz
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

% b) (Opcjonalnie) Jawne f0 dla każdej części:
% jeśli zdefiniujesz f0_parts (wektor długości = liczbie plików), użyj:
% if exist('f0_parts','var') && numel(f0_parts)==numel(files)
%     f = zeros(N,1);
%     idx0 = 1;
%     for k = 1:numel(files)
%         N_k = N_parts(k);
%         f(idx0:idx0+N_k-1) = f0_parts(k) + (0:N_k-1)'*df_Hz;
%         idx0 = idx0 + N_k;
%     end
% end

% =========================
% 3) Profil odległości (IFFT po f)
% =========================
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

% =========================
% 4) Faza vs częstotliwość + estymacja R z nachylenia
% =========================
phase_unw = unwrap(angle(V));

% dopasowanie prostej: phase ≈ a*f + b  => R_hat = -a*C/(4*pi)
% (używamy ujemnego nachylenia; dopasowanie robustyczne, a potem LSQ fallback)
try
    % Robust fit (wymaga Statistics Toolbox: fitlm/robustfit)
    X = [f, ones(size(f))];
    a = robustfit(f, phase_unw);   % zwraca [bias; slope]
    slope = a(2);
catch
    % fallback zwykłe LSQ
    p = polyfit(f, phase_unw, 1);
    slope = p(1);
end
R_hat = - slope * C / (4*pi);

% linia referencyjna dla R_hat (dla czytelności odejmijmy fazę w f(1))
phi_lin = (slope*(f - f(1)));

% =========================
% 5) Wykresy
% =========================
figure; 
plot(Rcm, magY, 'LineWidth', 1.2); grid on;
xlabel('Range [cm]'); ylabel('|IFFT(V)|');
title(sprintf('Range Profile (SFCW) — %d Files, N=%d', numel(files), N));
xlim([0, min(400, Rmax*100)]); % dopasuj do sceny

figure;
plot(f/1e9, phase_unw, 'LineWidth', 1.2); hold on;
plot(f/1e9, phi_lin + (phase_unw(1)), 'k--', 'LineWidth', 1.2); grid on;
xlabel('Frequency [GHz]'); ylabel('Phase [rad]');
title(sprintf('Phase vs. f   |   R_{hat}=%.3f m', R_hat));
legend('Measured', 'Ideal', 'Location','best');

% =========================
% 6) (Opcjonalnie) podgląd pojedynczej „czasówki” z części 1
% =========================
% [raw1, Nchirp1] = rawDataParser(files{1}, numADCSamples, numADCBits, numRX, isReal);
% rx_sum1 = sum(raw1, 1)/numRX; rx_mat1 = reshape(rx_sum1, [numADCSamples, Nchirp1]);
% s = rx_mat1(:, 10);
% figure; plot(real(s)); hold on; plot(imag(s)); grid on;
% xlabel('próbka'); ylabel('ampl.'); title('przebieg (część 1, ton #10)');

%%

function Y_hp = range_highpass(Y, R, Rstop, Rpass)
% RANGE_HIGHPASS  Górnoprzepustowy filtr w domenie zasięgu (range-domain)
%   Y_hp = range_highpass(Y, R, Rstop, Rpass)
%   Y     - wektor po IFFT (kompleksowy profil zasięgu)
%   R     - oś zasięgu [m], jak (0:Nfft-1)*(Rmax/Nfft)
%   Rstop - koniec pasma zaporowego [m] (tu np. 0.30)
%   Rpass - początek pasma przepustowego [m] (np. 0.40 dla 10 cm przejścia)

    N = numel(Y);
    dr = R(2) - R(1);

    % Indeksy granic (pilnuj zakresów)
    Kstop = max(0, floor(Rstop/dr));
    Kpass = max(Kstop+1, ceil(Rpass/dr));
    Kstop = min(Kstop, floor(N/2));
    Kpass = min(Kpass, floor(N/2));

    % Długość strefy przejściowej
    Ktrans = max(1, Kpass - Kstop);

    % Maska: 0..Kstop = 0, potem rampa kosinusowa do 1, środek = 1,
    % na ogonie symetrycznie (bo IFFT jest cykliczne).
    mask = ones(N,1);
    if Kstop > 0
        mask(1:Kstop) = 0;
        mask(end-Kstop+1:end) = 0;
    end

    ramp = 0.5 - 0.5*cos(pi*(0:Ktrans-1)'/(Ktrans));  % 0->1
    mask(Kstop+1:Kpass) = ramp;                       % przód
    mask(end-Kpass+1:end-Kstop) = flipud(ramp);       % ogon

    % Na wypadek N < 2*Kpass – asekuracja indeksów
    mask = mask(1:N);

    % Zastosuj filtr
    Y_hp = Y .* mask;
end

% --- Oś zasięgu
R = (0:Nfft-1).' * (C/(2*df_Hz)/Nfft);   % [m]

% --- Filtr HP: wytnij do 0.30 m z 10 cm rampą (0.30->0.40 m)
Y_hp = range_highpass(Y, R, 0.95, 1.05);

% --- Rysunek w dB
magY_dB = 20*log10(abs(Y_hp)/max(abs(Y_hp)));
figure; plot(R*100, abs(Y_hp), 'LineWidth', 1.2); grid on;
title('Range Profile (SFCW) — HFP Applied');
xlim([0 1000]);
xlabel('Range [cm]'); ylabel('Mag [dB]');
% title('Profil po filtrze górnoprzepustowym (R_{stop}=30 cm)');