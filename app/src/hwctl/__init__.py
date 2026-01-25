class HwCtl:
    def __init__(self, uart, radar_api, logger):
        self.uart = uart
        self.radar_api = radar_api
        self.logger = logger