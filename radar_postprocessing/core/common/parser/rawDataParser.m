function [retVal, numCh, N_actual] = rawDataParser(fileName, ...
                                                   numADCSamples, ...
                                                   numRX, isReal, ...
                                                   expectedChirps)
% RAWDATAPARSER  Parse a DCA1000 raw bin (FMCW or SFCW), auto-clamping N to the
%   actual file size.
%
%   retVal                    = rawDataParser(fileName, numADCSamples, numRX, isReal)
%   [retVal, numCh]           = rawDataParser(fileName, numADCSamples, numRX, isReal)
%   [retVal, numCh, N_actual] = rawDataParser(..., expectedChirps)
%
%   Inputs:
%     fileName        path to *_Raw_0.bin
%     numADCSamples   metadata N (starting guess)
%     numRX           RX channel count
%     isReal          true = real (2 B/sample), false = complex (4 B/sample)
%     expectedChirps  optional, default 1 (used to derive N when truncated)
%
%   Outputs:
%     retVal    [numRX x (numCh * N_actual)] IQ data (complex or real)
%     numCh     chirps recovered from the file
%     N_actual  samples per chirp actually in the file (<= numADCSamples)

    if nargin < 5 || isempty(expectedChirps)
        expectedChirps = 1;
    end

    bytes_per_sample = 2;            % int16, real
    if ~isReal
        bytes_per_sample = 4;        % I + Q, int16 each
    end

    info = dir(fileName);
    assert(~isempty(info), 'rawDataParser:notFound', 'File not found: %s', fileName);
    file_bytes  = info.bytes;
    bytes_chirp = numADCSamples * numRX * bytes_per_sample;

    if mod(file_bytes, bytes_chirp) == 0
        N_actual = numADCSamples;
        numCh    = file_bytes / bytes_chirp;
    else
        % DCA1000 truncated the last packet -> derive N from the actual size.
        N_actual = floor(file_bytes / bytes_per_sample / numRX / expectedChirps);
        numCh    = expectedChirps;
        warning('rawDataParser:truncated', ...
            ['File %d B doesn''t fit N=%d, numRX=%d (needs %d B/chirp). ' ...
             'Clamping N: %d -> %d (assuming %d chirp%s).'], ...
            file_bytes, numADCSamples, numRX, bytes_chirp, ...
            numADCSamples, N_actual, numCh, repmat('s', 1, numCh > 1));
    end

    fid = fopen(fileName, 'r');          % open file for binary read access
    adcData = fread(fid, 'int16');       % read .bin file
    fclose(fid);

    if isReal
        % real data: 2 bytes per sample
        LVDS = reshape(adcData, N_actual * numRX, numCh).';   % each row = one chirp
    else
        % complex data: 4 bytes per sample (2I followed by 2Q)
        fileSize = 2 * N_actual * numRX * numCh;
        LVDS = zeros(1, fileSize / 2);
        counter = 1;
        for i = 1:4:fileSize-1
            LVDS(1, counter)     = adcData(i)     + 1j * adcData(i+2);
            LVDS(1, counter + 1) = adcData(i + 1) + 1j * adcData(i+3);
            counter = counter + 2;
        end
        LVDS = reshape(LVDS, N_actual * numRX, numCh).';      % each row = one chirp
    end

    % organize data per RX
    retVal = zeros(numRX, numCh * N_actual);
    for row = 1:numRX
        for i = 1:numCh
            retVal(row, (i-1)*N_actual+1 : i*N_actual) = ...
                LVDS(i, (row-1)*N_actual+1 : row*N_actual);
        end
    end
end
