function cfg = load_metadata(folder_path)
    json_path = fullfile(folder_path, 'metadata.json');
    if ~isfile(json_path)
        error('load_metadata:notFound', ...
              'metadata.json not found in: %s', folder_path);
    end

    cfg = jsondecode(fileread(json_path));
end
