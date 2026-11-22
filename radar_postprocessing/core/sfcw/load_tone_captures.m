function [C, proc1] = load_tone_captures(cal_dir, prefix, folder_indices, Nfft)
    M = numel(folder_indices);
    C = [];
    proc1 = [];
    for k = 1:M
        folder = fullfile(cal_dir, sprintf('%s_%03d', prefix, folder_indices(k)));
        if ~exist(folder, 'dir')
            error('load_tone_captures:missing', 'Missing capture folder: %s', folder);
        end
        proc = SfcwProcessor(folder, Nfft);
        if k == 1
            proc1 = proc;
            C = zeros(proc.N, M);
        elseif numel(proc.V) ~= proc1.N
            error('load_tone_captures:tones', '%s: %d tones, expected %d', ...
                  folder, numel(proc.V), proc1.N);
        end
        C(:, k) = proc.V;
    end
end
