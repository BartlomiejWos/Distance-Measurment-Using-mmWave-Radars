%% SFCW: profil odległości i faza z 2 plików (1xTX)
clc; clear; close all;

% --- Pliki wejściowe (dwie części pomiaru)
filename1 = "sfcw_512_0875MHz_part1_Raw_0.bin";
filename2 = "sfcw_512_0875MHz_part2_Raw_0.bin";

% --- Parametry akwizycji
numADCSamples = 256;   % próbek ADC na chirp (okres czasu dla pojedynczej częstotliwości)
numADCBits    = 16;    % bity ADC
numRX         = 4;     % liczba odbiorników
isReal        = false; % dane zespolone?

% --- Parametry SFCW / systemowe
C      = 299792458;       % prędkość światła [m/s]
df_Hz  = 0.875e6;         % krok częstotliwości [Hz]
Ns_use = 128;             % ile próbek z "fast-time" brać do uśredniania (jak w Twoim kodzie)
f0     = 77e9;            % częstotliwość startowa części 1 [Hz]
Nfft   = 4096;            % rozmiar IFFT (możesz zwiększyć dla gęstszej siatki R)
R_guess = 1.3;            % (opcjonalnie) odległość do prostej referencyjnej fazy [m] (do porównania)

% =========================
% 1) Parsowanie obu plików
% =========================
[retval1, numChirps1] = rawDataParser(filename1, numADCSamples, numADCBits, numRX, isReal);
[retval2, numChirps2] = rawDataParser(filename2, numADCSamples, numADCBits, numRX, isReal);

% Suma z wielu RX (możesz też wybrać konkretny RX, np. retval1(1,:) itd.)
rx_sum1 = sum(retval1, 1) / numRX;
rx_sum2 = sum(retval2, 1) / numRX;

% =========================
% 2) Uporządkowanie do macierzy [Ns x Nchirp] i uśrednienie po czasie
%    (dla SFCW: uśredniamy po próbkach czasowych -> 1 wartość/ton)
% =========================
rx_mat1 = reshape(rx_sum1, [numADCSamples, numChirps1]);
rx_mat2 = reshape(rx_sum2, [numADCSamples, numChirps2]);

% (opcjonalnie) użyj tylko pierwszych Ns_use próbek jak w Twoim przykładzie
rx_mat1 = rx_mat1(1:Ns_use, :);
rx_mat2 = rx_mat2(1:Ns_use, :);

% Jedna reprezentatywna próbka zespolona na częstotliwość (uśrednienie po "fast-time")
V1 = mean(rx_mat1, 1).';   % [N1 x 1]
V2 = mean(rx_mat2, 1).';   % [N2 x 1]

% Połącz część 1 i 2 w jedną sekwencję częstotliwości
V = [V1; V2];               % [N x 1]
N1 = numel(V1);
N2 = numel(V2);
N  = N1 + N2;

% (opcjonalnie) usuń DC (przesunięcie) — zwykle pomaga dla profilu zasięgu
V = V - mean(V);

% =========================
% 3) Oś częstotliwości dla obu części
%    Zakładamy, że part2 kontynuuje krok df_Hz od końca part1.
% =========================
f_idx = (0:N-1).';               % indeksy tonów
f     = f0 + f_idx * df_Hz;      % wektor częstotliwości [Hz]

% =========================
% 4) Profil odległości (IFFT po osi f)
% =========================
% Unambiguous range (dla SFCW): Rmax = C / (2*df), rozdzielczość ~ C/(2*B), gdzie B=N*df
Rmax = C / (2*df_Hz);
B    = N * df_Hz;
Rres = C / (2*B);

% okno + IFFT
w   = hann(N);
% Y   = ifft(w .* V, Nfft);
Y   = ifft(V, Nfft);

magY = abs(Y);                       % amplituda profilu
R    = (0:Nfft-1).' * (Rmax / Nfft); % oś zasięgu [m]
Rcm  = R * 100;                      % [cm]

fprintf('Rmax  = %.3f m\n', Rmax);
fprintf('Rres  = %.2f cm (≈ dokładność w zasięgu)\n', Rres*100);

% =========================
% 5) Faza vs częstotliwość (unwrapped)
% =========================
phase_meas = unwrap(angle(V));

% (opcjonalnie) linia referencyjna fazy ~ -4*pi*R/C * f
phi_lin = -4*pi*R_guess/C * (f - f0);   % odejmuję f0 dla czytelności (przesunięcie fazy)

% =========================
% 6) Wykresy
% =========================
figure; 
plot(Rcm, magY, 'LineWidth', 1.2); grid on;
xlabel('Zasięg [cm]'); ylabel('|IFFT(V)|');
title('Profil odległości (SFCW, part1+part2)');
% xlim([0, min(400, Rmax*100)]); % dostosuj pod swoje sceny

figure;
plot(f/1e9, phase_meas, 'r', 'LineWidth', 1.2); hold on;
plot(f/1e9, phi_lin, 'k--', 'LineWidth', 1.2); grid on;
xlabel('Częstotliwość [GHz]'); ylabel('Faza [rad]');
title('Faza rozpakowana vs. częstotliwość');
legend('Zmierzona faza', sprintf('Faza liniowa dla R=%.2f m', R_guess), 'Location','best');

% (opcjonalnie) podejrzenie pojedynczego tonu/czasówki
% chirp_1 = rx_mat1(:,10);
% figure; plot(real(chirp_1)); hold on; plot(imag(chirp_1)); grid on;
% xlabel('Próbka'); ylabel('Amplituda'); title('Pojedynczy ton (part1), próbka czasowa');