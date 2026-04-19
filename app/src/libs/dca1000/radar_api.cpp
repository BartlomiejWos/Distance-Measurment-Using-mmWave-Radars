#include <pybind11/pybind11.h>
#include "ti-external/rf_api.h"
#include "ti-external/dca_types.h"
#include "event_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SUCCESS_STATUS 0
#define FAILURE_STATUS -1

#define MAX_NAME_LEN 255
#define FPGA_CONFIG_DEFAULT_TIMER 30


static strEthConfigMode gsEthConfigMode = {
    .au8MacId = {12, 34, 56, 78, 90, 12},          // DCA1000 MAC address
    .au8PcIpAddr = {192, 168, 33, 30},             // systemIPAddress (PC) {192, 168, 33, 30}, 
    .au8Dca1000IpAddr = {192, 168, 33, 180},       // DCA1000 IP {192, 168, 33, 180},  
    .u32RecordPortNo = 4098,                       // Data port
    .u32ConfigPortNo = 4096                        // Config port
};


static strFpgaConfigMode gsFpgaConfigMode = {
    .eLogMode         = RAW_MODE,
    .eLvdsMode        = TWO_LANE,
    .eDataXferMode    = CAPTURE,
    .eDataCaptureMode = ETH_STREAM,
    .eDataFormatMode  = BIT16,
    .u8Timer          = FPGA_CONFIG_DEFAULT_TIMER
};


static strRecConfigMode gsRecConfigMode = {
    .u16RecDelay = 25
};


static strStartRecConfigMode gsStartRecConfigMode = {
    .s8FileBasePath = "C:\\Users\\Asus\\Desktop\\REPOZYTORIUM\\Distance-Measurment-Using-mmWave-Radars\\postproc\\data",
    .s8FilePrefix = "Capture",
    .bSequenceNumberEnable = true,                  
    .bMsbToggleEnable = false,                     
    .bReorderEnable = false,                        
    .u16MaxRecFileSize = 1024,                      
    .eRecordStopMode = FRAMES,                      
    .u32BytesToCapture = 0,                         
    .u32DurationToCapture = 0,                     
    .u32FramesToCapture = 1,
    .eConfigLogMode = RAW_MODE,
    .eLvdsMode = TWO_LANE,
};

int dca_set_file_base_path(const std::string& path) {
    size_t max_path_size = sizeof(gsStartRecConfigMode.s8FileBasePath);
    if (path.size() >= max_path_size) {
        printf("[ERROR] Path too long for s8FileBasePath buffer\n");
        return FAILURE_STATUS;
    }
    std::memset(gsStartRecConfigMode.s8FileBasePath, 0, max_path_size);
    std::memcpy(gsStartRecConfigMode.s8FileBasePath, path.c_str(), path.size());
    return SUCCESS_STATUS;
}

int dca_set_file_prefix(const std::string& prefix) {
    size_t max_path_size = sizeof(gsStartRecConfigMode.s8FilePrefix);
    if (prefix.size() >= max_path_size) {
        printf("[ERROR] Prefix too long for s8FilePrefix buffer\n");
        return FAILURE_STATUS;
    }
    for (char c : prefix) {
        switch (c) {
            case '<': case '>': case ':': case '"':
            case '/': case '\\': case '|': case '?': case '*':
                printf("[ERROR] Prefix contains invalid filename characters\n");
                return FAILURE_STATUS;
        }
    }
    std::memset(gsStartRecConfigMode.s8FilePrefix, 0, max_path_size);
    std::memcpy(gsStartRecConfigMode.s8FilePrefix, prefix.c_str(), prefix.size());
    return SUCCESS_STATUS;
}


SINT32 DisconnectRFDCCard(UINT16 u16CmdCode)
{
    if (u16CmdCode == CMD_CODE_CLI_ASYNC_RECORD_STOP)
    {
        return DisconnectRFDCCard_AsyncCommandMode();
    }
    else
    {
        return DisconnectRFDCCard_ConfigMode();
    }
}


int dca1000_config_init()
{
    py::gil_scoped_release release;
    UINT16 u16CmdCode = 0;
    SINT8 version[MAX_NAME_LEN] = {0};
    SINT32 s32CliStatus, cnt = 0;
    memset(version, '\0', MAX_NAME_LEN);

    int status = ReadRFDCCard_DllVersion(version);

    if (status != SUCCESS_STATUS)
    {
        printf("[ERROR]: Failed to read DLL version\n");
        return FAILURE_STATUS;
    }
    printf("[INFO]: DLL Version: %s\n", version);

    s32CliStatus = StatusRFDCCard_EventRegister(EthernetEventHandlercallback);
    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR]: Ethernet event handler callback failed. [error %d]", s32CliStatus);
        return FAILURE_STATUS;
    }
    if (u16CmdCode == CMD_CODE_CLI_ASYNC_RECORD_STOP)
    {
        s32CliStatus = ConnectRFDCCard_AsyncCommandMode(gsEthConfigMode);
    }
    else
    {
        s32CliStatus = ConnectRFDCCard_ConfigMode(gsEthConfigMode);
    }

    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR]: Ethernet connection failed. [error %d]\n", s32CliStatus);
        DisconnectRFDCCard(u16CmdCode);
        return FAILURE_STATUS;
    }
    /** Reset the FPGA, to avoid any error for last execution */
    s32CliStatus = ResetRFDCCard_FPGA();
    DecodeCommandStatus(s32CliStatus, "FPGA Reset");
    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR] FPGA RFDC card reset failed. [error %d]\n", s32CliStatus);
        DisconnectRFDCCard(u16CmdCode);
        return FAILURE_STATUS;
    }
    printf("[SUCCESS]: Ethernet connection successful. [status code: %d]\n", s32CliStatus);
    /** API Call - Configure FPGA    */
    s32CliStatus = ConfigureRFDCCard_Fpga(gsFpgaConfigMode);
    DecodeCommandStatus(s32CliStatus, "FPGA Configuration");
    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR] FPGA configuration failed. [error %d]\n", s32CliStatus);
        DisconnectRFDCCard(u16CmdCode);
        return FAILURE_STATUS;
    }
    /** API Call - Configure Record  */
    s32CliStatus = ConfigureRFDCCard_Record(gsRecConfigMode);
    DecodeCommandStatus(s32CliStatus, "Configure Record");
    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR] Record configuration failed. [error %d]\n", s32CliStatus);
        DisconnectRFDCCard(u16CmdCode);
        return FAILURE_STATUS;
    }

    /** API Call - System aliveness  */
    s32CliStatus = HandshakeRFDCCard();
    DecodeCommandStatus(s32CliStatus, "Handshake FPGA");
    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR] FPGA handshake failed. [error %d]\n", s32CliStatus);
        DisconnectRFDCCard(u16CmdCode);
        return FAILURE_STATUS;
    }
    /* disconnect for config mode */
    s32CliStatus = DisconnectRFDCCard_ConfigMode();
    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR] Config mode disconnect failed. [error %d]\n", s32CliStatus);
        return FAILURE_STATUS;
    }
    /* Required: DLL's UDP receive thread dereferences inline-stats callback
     * on each packet; without registration that pointer is uninitialized →
     * 0xC0000005 on first packet burst. Stub handler is enough. */
    s32CliStatus = RecInlineProcStats_EventRegister(
        (INLINE_PROC_HANDLER)inlineStatsCallback);
    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR] Inline stats callback register failed. [error %d]\n", s32CliStatus);
        return FAILURE_STATUS;
    }
    /** API Call - Ethernet connection                                       */
    s32CliStatus = ConnectRFDCCard_RecordMode(gsEthConfigMode);
    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR] Ethernet record mode connection failed. [error %d]", s32CliStatus);
        DisconnectRFDCCard_RecordMode();
        return FAILURE_STATUS;
    }
    return SUCCESS_STATUS;
}


int dca_trigger_record(void)
{
    py::gil_scoped_release release;
    SINT32 s32CliStatus;
    /** API Call - Start Record     */
    s32CliStatus = StartRecordData(gsStartRecConfigMode);
    /** Handling command response   */
    DecodeCommandStatus(s32CliStatus, "Start Record");
    return s32CliStatus;
}


int dca_stop_record(void)
{
    py::gil_scoped_release release;
    SINT32 s32CliStatus;
    UINT16 u16CmdCode = 0;
    /* if already stopped */
    if (dcaCaptureMode == DCA1000_CAPTURE_STOP)
    {
        printf("[ERROR] Capture already stopped\n");
        return FAILURE_STATUS;
    }
    /* API Call - Stop Record */
    if (u16CmdCode == CMD_CODE_CLI_ASYNC_RECORD_STOP)
    {
        s32CliStatus = StopRecordAsyncCmd();
    }
    else
    {
        s32CliStatus = StopRecordData();
    }

    if (s32CliStatus != SUCCESS_STATUS)
    {
        DecodeCommandStatus(s32CliStatus, "Stop Record Command");
        return FAILURE_STATUS;
    }
    s32CliStatus = DisconnectRFDCCard_RecordMode();
    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR] Record mode disconnect failed. [error %d]\n", s32CliStatus);
        return FAILURE_STATUS;
    }
    return SUCCESS_STATUS;
}


void register_event_callback(py::function cb)
{
    py_event_callback = cb;
}


void dca1000_deinit(void)
{
    {
        py::gil_scoped_release release;
        /* Stop record if capture is still running */
        if (dcaCaptureMode == DCA1000_CAPTURE_ONGOING)
        {
            SINT32 s32CliStatus = StopRecordData();
            DecodeCommandStatus(s32CliStatus, "Deinit Stop Record");
        }
        /* Disconnect both modes — safe to call even if already disconnected */
        DisconnectRFDCCard_RecordMode();
        DisconnectRFDCCard_ConfigMode();
        /* Reset global state for next init cycle */
        dcaCaptureMode = 0;
    }
    /* GIL re-acquired here — safe to touch py::function */
    py_event_callback = py::function();
    printf("[INFO] DCA1000 deinitialized\n");
}

#ifdef __cplusplus
}
#endif


PYBIND11_MODULE(radar_api, module)
{
    module.doc() = "Python bindings for RFDC FPGA Card DCA1000 API";
    module.def("register_event_callback", &register_event_callback);
    module.def("dca_set_file_base_path", &dca_set_file_base_path);
    module.def("dca_set_file_prefix", &dca_set_file_prefix);
    module.def("dca1000_config_init", &dca1000_config_init);
    module.def("dca1000_deinit", &dca1000_deinit);
    module.def("dca_trigger_record", &dca_trigger_record);
    module.def("dca_stop_record", &dca_stop_record);
}
