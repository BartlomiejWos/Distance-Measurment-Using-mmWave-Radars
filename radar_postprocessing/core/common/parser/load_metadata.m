function cfg = load_metadata(folder_path)
    json_path = fullfile(folder_path, 'metadata.json');
    if ~isfile(json_path)
        error('load_metadata:notFound', ...
              'metadata.json not found in: %s', folder_path);
    end

    cfg = jsondecode(fileread(json_path));

    if isfield(cfg, 'start_freq_hz') && ~isfield(cfg, 'start_freq_ghz')
        cfg.start_freq_ghz = cfg.start_freq_hz / 1e9;
        cfg = rmfield(cfg, 'start_freq_hz');
    end
    if isfield(cfg, 'end_freq_hz') && ~isfield(cfg, 'end_freq_ghz')
        cfg.end_freq_ghz = cfg.end_freq_hz / 1e9;
        cfg = rmfield(cfg, 'end_freq_hz');
    end

    % derived constants
    cfg.C    = 299792458;
    if isfield(cfg, 'df_hz')
        cfg.Rmax = cfg.C / (2 * cfg.df_hz);
    end
end
