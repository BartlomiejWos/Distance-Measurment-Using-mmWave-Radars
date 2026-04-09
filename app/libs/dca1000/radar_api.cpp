#include <pybind11/pybind11.h>
#include "ti-external/rf_api.h"
#include "ti-external/dca_types.h"
#include "event_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NAME_LEN 255
#define SUCCESS_STATUS 0
#define FAILURE_STATUS 1
#define FPGA_CONFIG_DEFAULT_TIMER 30

static strEthConfigMode gsEthConfigMode = {
    .au8MacId = {12, 34, 56, 78, 90, 12},                   // DCA1000 MAC address
    .au8PcIpAddr = {192, 168, 33, 30},                      // systemIPAddress (PC) {192, 168, 33, 30}, 
    .au8Dca1000IpAddr = {192, 168, 33, 180},                // DCA1000 IP {192, 168, 33, 180},  
    .u32RecordPortNo = 4098,                                // Data port
    .u32ConfigPortNo = 4096                                 // Config port
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
    .s8FilePrefix = "capture_",                    
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


void dca_set_file_base_path(const std::string& path) {
    constexpr size_t CAP = sizeof(gsStartRecConfigMode.s8FileBasePath);

    if (path.size() >= CAP) {
        throw std::runtime_error("Path too long for s8FileBasePath buffer");
    }

    std::memset(gsStartRecConfigMode.s8FileBasePath, 0, CAP);
    std::memcpy(gsStartRecConfigMode.s8FileBasePath, path.c_str(), path.size());
}


// Bezpieczne ustawienie prefixu (zero-terminated, bez przepełnienia)
void dca_set_file_prefix(const std::string& prefix) {
    // UWAGA: dopasuj do rzeczywistego typu/rozmiaru s8FilePrefix
    constexpr size_t CAP = sizeof(gsStartRecConfigMode.s8FilePrefix);

    if (prefix.size() >= CAP) {
        throw std::runtime_error("Prefix too long for s8FilePrefix buffer");
    }

    // (Opcjonalnie) prosta sanityzacja znaków dla Windows
    for (char c : prefix) {
        switch (c) {
            case '<': case '>': case ':': case '"':
            case '/': case '\\': case '|': case '?': case '*':
                throw std::runtime_error("Prefix contains invalid filename characters");
        }
    }

    std::memset(gsStartRecConfigMode.s8FilePrefix, 0, CAP);
    std::memcpy(gsStartRecConfigMode.s8FilePrefix, prefix.c_str(), prefix.size());
}



/** @fn void cli_DisconnectRFDCCard(UINT16 u16CmdCode)
 * @brief This function is to disconnect from DCA1000EVM sysytem
 * @param [in] u16CmdCode [UINT16] - Command code
 */
void DisconnectRFDCCard(UINT16 u16CmdCode)
{
    if (u16CmdCode == CMD_CODE_CLI_ASYNC_RECORD_STOP)
    {
        DisconnectRFDCCard_AsyncCommandMode();
    }
    else
    {
        DisconnectRFDCCard_ConfigMode();
    }
}


/** @fn void inlineStatsCallback(strRFDCCard_InlineProcStats inlineStats, bool bOutOfSeqSetFlag)
 * @brief This function is to handle the update of inline processing summary using callbacks
 * @param [in] inlineStats [strRFDCCard_InlineProcStats] - Status code
 * @param [in] bOutOfSeqSetFlag [bool] - If OutOfSeq occured then set flag
 */
void inlineStatsCallback(strRFDCCard_InlineProcStats inlineStats,
                         bool                        bOutOfSeqSetFlag,
                         UINT8                       u8DataIndex)
{
#if 0 /* copied from DCA application code but disabled for this tool.*/
	if (bReady)
	{
		/** Copying global variables and trigger the event to update    */
		memcpy(&gInlineStats, &inlineStats, sizeof(strRFDCCard_InlineProcStats));
		gbOutOfSeqSetFlag = bOutOfSeqSetFlag;
		gu8DataIndex = u8DataIndex;

		osalObj_Rec.SignalEvent(&sgnInlineStsUpdateWaitEvent);
	}
#endif
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
        printf("Error: Failed to read DLL version\n");
        return -1;
    }
    printf("DLL Version: %s\n", version);

    s32CliStatus = StatusRFDCCard_EventRegister(EthernetEventHandlercallback);

     /* API Call - Ethernet connection */
    if (u16CmdCode == CMD_CODE_CLI_ASYNC_RECORD_STOP)
    {
        s32CliStatus = ConnectRFDCCard_AsyncCommandMode(gsEthConfigMode);
    }
    else
    {
        /* this needs to be called at first time */
        s32CliStatus = ConnectRFDCCard_ConfigMode(gsEthConfigMode);
    }

    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR] Ethernet connection failed. [error %d]", s32CliStatus);
        DisconnectRFDCCard(u16CmdCode);
        return -1;
    }
    printf("[SUCCESS] Ethernet connection successful. [status code: %d]\n", s32CliStatus);

    /** Reset the FPGA, to avoid any error for last execution */
    s32CliStatus = ResetRFDCCard_FPGA();
    DecodeCommandStatus(s32CliStatus, "FPGA Reset");

    /** API Call - Configure FPGA    */
    s32CliStatus = ConfigureRFDCCard_Fpga(gsFpgaConfigMode);
    DecodeCommandStatus(s32CliStatus, "FPGA Configuration");

    /** API Call - Configure Record  */
    s32CliStatus = ConfigureRFDCCard_Record(gsRecConfigMode);
    DecodeCommandStatus(s32CliStatus, "Configure Record");

    /** API Call - System aliveness  */
    s32CliStatus = HandshakeRFDCCard();
    DecodeCommandStatus(s32CliStatus, "Handshake FPGA");

    /* diconnect for config mode */
    DisconnectRFDCCard_ConfigMode();

    s32CliStatus = RecInlineProcStats_EventRegister(
        (INLINE_PROC_HANDLER)inlineStatsCallback);
    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR] Registering Callback function failed (Inline status). error[%d]", s32CliStatus);
        return -1;
    }

    /** API Call - Ethernet connection                                       */
    s32CliStatus = ConnectRFDCCard_RecordMode(gsEthConfigMode);
    if (s32CliStatus != SUCCESS_STATUS)
    {
        printf("[ERROR] Ethernet connection failed. [error %d]", s32CliStatus);
        DisconnectRFDCCard_RecordMode();
        return -1;
    }
    return 0;
}


void read_rfdc_card_fpga_version()
{
    SINT8 version[MAX_NAME_LEN] = {0};
    memset(version, '\0', MAX_NAME_LEN);

    int status = ReadRFDCCard_FpgaVersion(version);

    // if (status != SUCCESS_STATUS) {
    //     printf("Error: Failed to read FPGA version\n");
    //     return;
    // }

    printf("FPGA Version: %s\n", version);
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


/** @fn int dcaStopRecording(void)
 * @brief This function stops the DCA1000 capture.
 * @param [in] None
 */
int dca_stop_record(void)
{
    py::gil_scoped_release release;
    SINT32 s32CliStatus;
    UINT16 u16CmdCode = 0;

    /* if already stopped */
    if (dcaCaptureMode == DCA1000_CAPTURE_STOP)
    {
        return -1;
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
    }
    else
    {
        DisconnectRFDCCard_RecordMode();
    }
    return 0;
}

void register_event_callback(py::function cb)
{
    py_event_callback = cb;
}


/** @fn int dca1000_deinit(void)
 * @brief Tear down DCA1000 connections and reset module state so that
 *        dca1000_config_init() can be called again in the same process.
 */
int dca1000_deinit(void)
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

    printf("[DEINIT] DCA1000 deinitialized\n");
    return 0;
}

#ifdef __cplusplus
}
#endif


PYBIND11_MODULE(radar_api, module)
{
    module.doc() = "Python bindings for RF FPGA version API via pybind11";
    module.def("register_event_callback", &register_event_callback);
    module.def("dca_set_file_base_path", &dca_set_file_base_path);
    module.def("dca_set_file_prefix", &dca_set_file_prefix);
    module.def("dca1000_config_init", &dca1000_config_init);
    module.def("dca1000_deinit", &dca1000_deinit);
    module.def("read_rfdc_card_fpga_version", &read_rfdc_card_fpga_version);
    module.def("dca_trigger_record", &dca_trigger_record);
    module.def("dca_stop_record", &dca_stop_record);
}
