classdef SfcwProcessor
    properties
        cfg           % struct from metadata.json
        num_chirps    % [num_frames x 1] chirps (= tones) per frame
        V             % [N x 1] complex - one IQ per tone
        N             % total number of tones
        Nfft          % IFFT size
        Y             % [Nfft x 1] complex - range profile
        R             % [Nfft x 1] range axis [m]
        Rcm           % [Nfft x 1] range axis [cm]
        f             % [N x 1] frequency axis [Hz]
        BW            % total bandwidth [Hz]
        Rres          % range resolution [m]
        Rmax          % max unambiguous range [m]
    end

    methods
        function obj = SfcwProcessor(folder_path, Nfft)
            obj.Nfft = Nfft;
            obj.cfg = load_metadata(folder_path);
            [raw, obj.num_chirps] = load_measurement(folder_path, obj.cfg);

            % one IQ value per tone: average over RX channels and ADC samples
            rx_avg = sum(raw, 1) / obj.cfg.num_rx;
            rx_mat = reshape(rx_avg, [obj.cfg.num_adc_samples, sum(obj.num_chirps)]);
            obj.V = mean(rx_mat, 1).';
            obj.N = numel(obj.V);
            obj.f = obj.cfg.start_freq_ghz * 1e9 + (0:obj.N-1).' * obj.cfg.df_hz;

            obj.BW = obj.N * obj.cfg.df_hz;
            obj.Rres = phys_const.C / (2 * obj.BW);
            obj.Rmax = phys_const.C / (2 * obj.cfg.df_hz);
            obj.R = (0:obj.Nfft-1).' * (obj.Rmax / obj.Nfft);
            obj.Rcm = obj.R * 100;
        end

        function obj = remove_dc(obj)
            obj.V = obj.V - mean(obj.V);
        end

        function obj = range_profile(obj, window_fn)
            w = window_fn(obj.N);
            obj.Y = ifft(w .* obj.V, obj.Nfft);
        end

        function phi_ideal = ideal_phase(obj, R0)
            phi_ideal = -(4*pi*R0/phys_const.C) * (obj.f - obj.f(1));
        end
    end
end
