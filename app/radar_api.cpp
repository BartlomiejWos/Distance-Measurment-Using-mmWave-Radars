#include <pybind11/pybind11.h>
#include "ti-external/rf_api.h"
#include "ti-external/dca_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_NAME_LEN 255
#define SUCCESS_STATUS 0 
#define FAILURE_STATUS 1


strEthConfigMode gsEthConfigMode = {
    .au8MacId = {12, 34, 56, 78, 90, 12},       // DCA1000 MAC address
    .au8PcIpAddr = {192, 168, 33, 30},                      // systemIPAddress (PC)
    .au8Dca1000IpAddr = {192, 168, 33, 180},                // DCA1000 IP
    .u32RecordPortNo = 4098,                                // Data port
    .u32ConfigPortNo = 4096                                 // Config port
};

/** @fn void EthernetEventHandlercallback(UINT16 u16CmdCode, UINT16 u16Status)
 * @brief RF data card capture event handler
 * @param [in] cmd code
 * @param [in] status
 */
void EthernetEventHandlercallback(UINT16 u16CmdCode, UINT16 u16Status)
{
    printf("Call back called :  Async event recieved %d, %d \n", u16CmdCode, u16Status);

    // Syste Error cmd code
    if (u16CmdCode == 0x0A)
    {
        switch (u16Status)
        {
            case 0x0:
                printf("STS_NO_LVDS_DATA Async event recieved [%d] \n", u16Status);
                break;
            case 0x1:
                printf("STS_NO_HEADER Async event recieved [%d] \n", u16Status);
                break;
            case 0x2:
                printf("STS_EEPROM_FAILURE Async event recieved [%d] \n", u16Status);
                break;
            case 0x3:
                printf("STS_SD_CARD_DETECTED Async event recieved [%d] \n", u16Status);
                break;
            case 0x4:
                printf("STS_SD_CARD_REMOVED Async event recieved [%d] \n", u16Status);
                break;
            case 0x5:
                printf("STS_SD_CARD_FULL Async event recieved [%d] \n", u16Status);
                break;
            case 0x6:
                printf("STS_MODE_CONFIG_FAILURE Async event recieved [%d] \n", u16Status);
                break;
            case 0x7:
                printf("STS_DDR_FULL Async event recieved [%d] \n", u16Status);
                break;
            case 0x8:
                // dcaCaptureMode = DCA1000_CAPTURE_DONE;
                // /* for fixed set of frame count or on sensorStop, dca will generate this
                //  * event, so stop the dca capture which will update the adc*.bin file */
                // mmw_CaptureDcaDone();
                printf("STS_RECORD_COMPLETED Async event recieved [%d] \n", u16Status);
                break;
            case 0x9:
                printf("STS_LVDS_BUFFER_FULL Async event recieved [%d] \n", u16Status);
                break;
            case 0xA:
                printf("STS_PLAYBACK_COMPLETED Async event recieved [%d] \n", u16Status);
                break;
            case 0xB:
                printf("STS_PLAYBACK_OUT_OF_SEQUENCE Async event recieved [%d] \n", u16Status);
                break;
            default:
                printf("Syste Error cmd code :  Async event recieved [%d] \n", u16Status);
                break;
        }
    }
    else
    {
        switch (u16CmdCode)
        {
            // Event register status
            case 0x01:
                printf("RESET_FPGA_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x02:
                printf("RESET_AR_DEV_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x03:
                printf("CONFIG_FPGA_GEN_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x04:
                printf("CONFIG_EEPROM_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x05:
                // dcaCaptureMode = DCA1000_CAPTURE_ONGOING;
                printf("RECORD_START_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x06:
                // dcaCaptureMode = DCA1000_CAPTURE_STOP;
                printf("RECORD_STOP_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x07:
                printf("PLAYBACK_START_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x08:
                printf("PLAYBACK_STOP_CMD_CODE event recieved [%d] \n", u16CmdCode);
                break;
            case 0x09:
                printf("SYSTEM_CONNECT_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x0A:
                printf("SYSTEM_ERROR_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x0B:
                printf("CONFIG_PACKET_DATA_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x0C:
                printf("CONFIG_DATA_MODE_AR_DEV_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x0D:
                printf("INIT_FPGA_PLAYBACK_CMD_CODE event recieved [%d] \n", u16CmdCode);
                break;
            case 0x0E:
                break;
            case 0xC1:
                printf("INVALID_RESP_PKT_ERROR_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0xC2:
                printf("RECORD_FILE_CREATION_ERROR_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0xC3:
                printf("RECORD_PKT_OUT_OF_SEQ_ERROR_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0xC4:
                printf("RECORD_IS_IN_PROGRESS_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0xC5:
                printf("GUI_PLAYBACK_COMPLETED_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0xC6:
                printf("PLAYBACK_FILE_OPEN_ERROR_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0xC7:
                printf("PLAYBACK_UDP_WRITE_ERR Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0xC8:
                printf("PLAYBACK_IS_IN_PROGRESS_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            default:
                printf("RF data capture related Async event recieved [%d] \n", u16CmdCode);
                break;
        }
    }
}

/** @fn void DecodeCommandStatus(SINT32 s32Status, const SINT8 * strCommand)
 * @brief This function is to decode command response status as success or failure
 * @param [in] s32Status [SINT32] - Command status
 * @param [in] strCommand [const SINT8 *] - Command name
 */
void DecodeCommandStatus(SINT32 s32Status, const SINT8 *strCommand)
{
    SINT8 msgData[MAX_NAME_LEN];

    if (s32Status == SUCCESS_STATUS)
    {
        sprintf(msgData, "%s command : Success", strCommand);
    }
    else if (s32Status == FAILURE_STATUS)
    {
        sprintf(msgData, "%s command : Failed", strCommand);
    }
    else if (s32Status == STS_RFDCCARD_INVALID_INPUT_PARAMS)
    {
        sprintf(msgData, "%s : \nVerify the input parameters - %d", strCommand, s32Status);
    }
    else if (s32Status == STS_RFDCCARD_OS_ERR)
    {
        sprintf(msgData, "%s : \nOS error - %d", strCommand, s32Status);
    }
    else if (s32Status == STS_RFDCCARD_UDP_WRITE_ERR)
    {
        sprintf(msgData, "%s : \nSending command failed - %d", strCommand, s32Status);
    }
    else if (s32Status == STS_RFDCCARD_TIMEOUT_ERR)
    {
        sprintf(msgData, "%s : \nTimeout Error! System disconnected", strCommand);
    }
    else if (s32Status == STS_INVALID_RESP_PKT_ERR)
    {
        sprintf(msgData, "%s : \nInvalid response packet error code", strCommand);
    }
    else
    {
        sprintf(msgData, "%s Cmd", strCommand);
    }

    printf("%s", msgData);
    printf(" Return status : %d\n", s32Status);
}


void dca1000_config_init()
{   
    UINT16 u16CmdCode = 0;
    SINT8 version[MAX_NAME_LEN] = {0};
    SINT32 s32CliStatus, cnt = 0;
    memset(version, '\0', MAX_NAME_LEN);

    int status = ReadRFDCCard_DllVersion(version);

    if (status != SUCCESS_STATUS) {
        printf("Error: Failed to read DLL version\n");
        return;
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
    }
    printf("[SUCCESS] Ethernet connection successful. [status code: %d]\n", s32CliStatus);
    /* Reset the FPGA, to avoid any error for last execution */
    s32CliStatus = ResetRFDCCard_FPGA();

    /** Handling command response    */
    DecodeCommandStatus(s32CliStatus, "FPGA Reset");
}



void read_rfdc_card_fpga_version()
{
    SINT8 version[MAX_NAME_LEN] = {0};
    memset(version, '\0', MAX_NAME_LEN);

    int status = ReadRFDCCard_FpgaVersion(version);

    if (status != SUCCESS_STATUS) {
        printf("Error: Failed to read FPGA version\n");
        return;
    }

    printf("FPGA Version: %s\n", version);
}

#ifdef __cplusplus
}
#endif

namespace py = pybind11;

PYBIND11_MODULE(radar_api, module)
{
    module.doc() = "Python bindings for RF FPGA version API via pybind11";
    module.def("dca1000_config_init", &dca1000_config_init);
    module.def("read_rfdc_card_fpga_version", &read_rfdc_card_fpga_version);
   
}
