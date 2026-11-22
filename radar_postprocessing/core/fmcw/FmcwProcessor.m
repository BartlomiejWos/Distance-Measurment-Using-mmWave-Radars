classdef FmcwProcessor
    properties
        cfg           % struct from metadata.json
        num_chirps   % total chirps across all bin files
        X             % [N x numChirps] complex - RX-averaged, fast-time x slow-time
        Nfft          % Number of FFT bins
        RP            % [Nfft/2 x numChirps] complex - per-chirp range profile
        Y             % [Nfft/2 x 1] complex - chirp-averaged range profile
        R             % [Nfft/2 x 1] range axis [m]
        Rcm           % [Nfft/2 x 1] range axis [cm]
        Fs            % ADC sample rate [Hz]
        slope         % chirp slope [Hz/s]
        f0            % start freq [Hz]
        BW            % sampled bandwidth [Hz]
        Rres          % range resolution [m]
        Rmax          % max unambiguous range [m]
        N             % samples per chirp
    end

    methods
        function obj = FmcwProcessor(folder_path, Fs, Nfft)
            obj.Fs = Fs;
            obj.Nfft = Nfft;
            obj.cfg = jsondecode(fileread( ...
                           fullfile(folder_path, 'metadata.json')));
            obj.f0 = obj.cfg.start_freq_ghz * 1e9;
            obj.slope = obj.cfg.freq_slope_mhz_per_usec * 1e12;
            obj.N = obj.cfg.num_adc_samples;

            bins = dir(fullfile(folder_path, '*_Raw_0.bin'));
            if isempty(bins)
                error('FmcwProcessor:noFiles', ...
                      'No *_Raw_0.bin in %s', folder_path);
            end

            numRX = obj.cfg.num_rx;
            filename = fullfile(folder_path, bins(1).name);
            [raw, obj.num_chirps, N_actual] = rawDataParser( ...
                filename, obj.N, numRX, obj.cfg.is_real);
            obj.N = N_actual;
            Xrx = reshape(raw.', obj.N, obj.num_chirps, numRX);
            obj.X = sum(Xrx, 3) / numRX;   % RX averaging only; DC removed downstream

            obj.BW = obj.slope * obj.N / obj.Fs;
            obj.Rres = phys_const.C / (2 * obj.BW);
            obj.Rmax = phys_const.C * (obj.Fs / 2) / (2 * obj.slope);   % range axis uses bins 0..Fs/2

            f_beat = (0:obj.Nfft/2-1).' * obj.Fs / obj.Nfft;
            obj.R = f_beat * phys_const.C / (2 * obj.slope);
            obj.Rcm = obj.R * 100;
        end

        function obj = range_profile(obj, window_fn)
            w = window_fn(obj.N);
            Xw = (obj.X - mean(obj.X, 1)) .* w;
            RPf = fft(Xw, obj.Nfft, 1);
            obj.RP = RPf(1:obj.Nfft/2, :);
            obj.Y = mean(obj.RP, 2);
        end

        function P_dB = power_rel_noise(obj, noise_cm)
            P_dB = range_profile_power_rel_noise(abs(obj.Y), obj.Rcm, noise_cm);
        end

        function print_info(obj)
            fprintf('Chirps: %d | N: %d | RX avg (%d ch)\n', ...
                    obj.num_chirps, obj.N, obj.cfg.num_rx);
            fprintf('f0=%.2f GHz  slope=%.2f MHz/us\n', ...
                    obj.f0/1e9, obj.slope/1e12);
            fprintf('BW=%.2f MHz  dR=%.2f cm  Rmax=%.2f m\n', ...
                    obj.BW/1e6, obj.Rres*100, obj.Rmax);
        end
    end
end
