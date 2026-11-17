function Xc = correct_iq_apply(X, dg, dp)
    I = real(X);  Q = imag(X);
    Qc = (Q ./ (1 + dg) + I .* sin(dp)) ./ cos(dp);
    Xc = I + 1j*Qc;
end
