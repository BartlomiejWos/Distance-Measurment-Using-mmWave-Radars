function k = peak_bin(RP, R_axis, R_target)
    near = find(abs(R_axis - R_target) <= 0.2);
    [~, i] = max(abs(RP(near, 1)));
    k = near(i);
end
