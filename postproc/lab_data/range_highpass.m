function Y_hp = range_highpass(Y, R, Rstop, Rpass)
% RANGE_HIGHPASS  Górnoprzepustowy filtr w domenie zasięgu (range-domain)
%   Y_hp = range_highpass(Y, R, Rstop, Rpass)
%   Y     - wektor po IFFT (kompleksowy profil zasięgu)
%   R     - oś zasięgu [m], jak (0:Nfft-1)*(Rmax/Nfft)
%   Rstop - koniec pasma zaporowego [m] (tu np. 0.30)
%   Rpass - początek pasma przepustowego [m] (np. 0.40 dla 10 cm przejścia)

    N  = numel(Y);
    dr = R(2) - R(1);
    
    Att_dB   = 40;                 % <-- ile chcesz tłumić w stopband [dB]
    mask_min = 10^(-Att_dB/20);    % <-- podłoga maski (amplitudowa)
    
    Kstop = max(0, floor(Rstop/dr));
    Kpass = max(Kstop+1, ceil(Rpass/dr));
    Kstop = min(Kstop, floor(N/2));
    Kpass = min(Kpass, floor(N/2));
    
    Ktrans = max(1, Kpass - Kstop);
    
    mask = ones(N,1);
    
    % zamiast 0 daj mask_min
    if Kstop > 0
        mask(1:Kstop) = mask_min;
        mask(end-Kstop+1:end) = mask_min;
    end
    
    % rampa od mask_min -> 1 (zamiast 0 -> 1)
    ramp01 = 0.5 - 0.5*cos(pi*(0:Ktrans-1)'/(Ktrans));   % 0->1
    ramp   = mask_min + (1-mask_min)*ramp01;             % mask_min -> 1
    
    mask(Kstop+1:Kpass) = ramp;
    mask(end-Kpass+1:end-Kstop) = flipud(ramp);
    
    mask = mask(1:N);
    
    Y_hp = Y .* mask;
end