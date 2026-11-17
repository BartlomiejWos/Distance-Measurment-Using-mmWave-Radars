function folder_indices = detect_captures(cal_dir, prefix, idx_planned)
    valid = false(numel(idx_planned), 1);
    for k = 1:numel(idx_planned)
        f = fullfile(cal_dir, sprintf('%s_%03d', prefix, idx_planned(k)));
        valid(k) = exist(f, 'dir') && exist(fullfile(f, 'metadata.json'), 'file') ...
                   && ~isempty(dir(fullfile(f, '*_Raw_0.bin')));
    end
    folder_indices = idx_planned(valid);
end
