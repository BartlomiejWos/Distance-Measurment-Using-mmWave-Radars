function [data, num_chirps] = load_measurement(folder_path, cfg)
    bins = dir(fullfile(folder_path, '*_Raw_0.bin'));
    if isempty(bins)
        error('load_measurement:noFiles', ...
              'No *_Raw_0.bin files found in: %s', folder_path);
    end
    bin_names = sort({bins.name});

    parts = cell(numel(bin_names), 1);
    num_chirps = zeros(numel(bin_names), 1);

    for k = 1:numel(bin_names)
        filepath = fullfile(folder_path, bin_names{k});
        [rawk, nck] = rawDataParser(filepath, ...
            cfg.num_adc_samples, cfg.num_rx, cfg.is_real);
        parts{k} = rawk;
        num_chirps(k) = nck;
    end

    data = horzcat(parts{:});
end
