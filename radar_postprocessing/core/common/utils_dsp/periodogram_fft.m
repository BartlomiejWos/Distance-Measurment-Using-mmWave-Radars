function [Pf_s, ampF_s, P_fft] = periodogram_fft(resid, gt_um, step_um, span_um)
    [gs, si] = sort(gt_um(:));
    rs       = detrend(resid(si));
    d_uni    = (gs(1):abs(step_um):gs(end)).';
    r_uni    = interp1(gs, rs, d_uni, 'linear');  r_uni = r_uni - mean(r_uni);
    win      = hann(numel(r_uni));
    Nz       = 2^nextpow2(32*numel(r_uni));
    R        = fft(r_uni .* win, Nz);
    ampF     = 2*abs(R(1:floor(Nz/2))) / sum(win);
    kbin     = (0:floor(Nz/2)-1).';
    Pg       = 1 ./ max(kbin/(abs(step_um)*Nz), eps);
    vv       = (Pg <= 0.9*span_um) & (Pg >= 2.2*abs(step_um));
    [Pf_s, os] = sort(Pg(vv));  av = ampF(vv);  ampF_s = av(os);
    [~, jf] = max(ampF_s);  P_fft = Pf_s(jf);
end
