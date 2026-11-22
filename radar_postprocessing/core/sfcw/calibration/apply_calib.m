function C = apply_calib(C, cal, use_aref, use_iq)
    if use_iq
        C = correct_iq_apply(C, cal.dg, cal.dp);
    end
    if use_aref
        C = C ./ cal.A_ref;
    end
    C = C .* exp(-1j * cal.beta);
end
