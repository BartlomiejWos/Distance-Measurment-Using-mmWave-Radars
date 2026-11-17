classdef SfcwProcessor
    properties
        cfg           % struct from metadata.json + derived constants
        raw           % [numRX x total_samples] raw IQ data
        num_chirps    % [num_frames x 1] chirps per frame
        V             % [N x 1] complex - one IQ per tone
        N             % total number of tones
        N_parts       % [num_frames x 1] tones per frame
        Nfft          % IFFT size
        Y             % [Nfft x 1] complex - range profile
        R             % [Nfft x 1] range axis [m]
        Rcm           % [Nfft x 1] range axis [cm]
        f             % [N x 1] frequency axis [Hz]
        B             % total bandwidth [Hz]
        Rres          % range resolution [m]
    end

    methods
        function obj = SfcwProcessor(folder_path, Nfft)
            if nargin < 2
                Nfft = 8192;
            end
            obj.Nfft = Nfft;
            obj.cfg = load_metadata(folder_path);
            [obj.raw, obj.num_chirps] = load_measurement(folder_path, obj.cfg);
        end

        function obj = average_tones(obj)
            total = sum(obj.num_chirps);
            rx_avg = sum(obj.raw, 1) / obj.cfg.num_rx;
            rx_mat = reshape(rx_avg, [obj.cfg.num_adc_samples, total]);
            obj.V = mean(rx_mat, 1).';
            obj.N = numel(obj.V);
            obj.N_parts = obj.num_chirps;
            obj.B = obj.N * obj.cfg.df_hz;
            obj.Rres = obj.cfg.C / (2 * obj.B);
            if isfield(obj.cfg, 'start_freq_ghz')
                f0 = obj.cfg.start_freq_ghz * 1e9;
            else
                f0 = obj.cfg.start_freq_hz;
            end
            obj.f = generate_frequency_vector( ...
                f0, obj.N, obj.N_parts, obj.cfg.df_hz);
        end

        function obj = remove_dc(obj)
            obj.V = obj.V - mean(obj.V);
        end

        function obj = range_profile(obj, window_fn)
            if nargin < 2
                window_fn = @hamming;
            end
            w = window_fn(obj.N);
            obj.Y = ifft(w .* obj.V, obj.Nfft);
            obj.R = (0:obj.Nfft-1).' * (obj.cfg.Rmax / obj.Nfft);
            obj.Rcm = obj.R * 100;
        end

        function phi_ideal = ideal_phase(obj, R0)
            phi_ideal = -(4*pi*R0/obj.cfg.C) * (obj.f - obj.f(1));
        end

        function print_info(obj)
            fprintf('Frames: %d | Tones: %d\n', numel(obj.num_chirps), obj.N);
            fprintf('Rmax  = %.3f m\n', obj.cfg.Rmax);
            fprintf('Rres  = %.2f cm\n', obj.Rres * 100);
        end
    end
end
