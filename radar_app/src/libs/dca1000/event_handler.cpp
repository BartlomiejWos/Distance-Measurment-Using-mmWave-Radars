#include "event_handler.h"
#include <cstdio>

#define MAX_NAME_LEN 255
#define SUCCESS_STATUS 0
#define FAILURE_STATUS 1

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

volatile char dcaCaptureMode = 0;
py::function py_event_callback;

static void invoke_py_callback(UINT16 u16CmdCode, UINT16 u16Status)
{
    if (!py_event_callback)
        return;
    PyGILState_STATE gstate = PyGILState_Ensure();
    try
    {
        py_event_callback(u16CmdCode, u16Status);
    }
    catch (const std::exception &e)
    {
        fprintf(stderr, "Python callback error: %s\n", e.what());
    }
    PyGILState_Release(gstate);
}

static void handle_system_error(UINT16 u16CmdCode, UINT16 u16Status)
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
                dcaCaptureMode = DCA1000_CAPTURE_DONE;
                printf("STS_RECORD_COMPLETED Async event recieved [%d] \n", u16Status);
                invoke_py_callback(u16CmdCode, u16Status);
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

static void handle_cmd_code(UINT16 u16CmdCode, UINT16 u16Status)
{
    switch (u16CmdCode)
    {
            case 0x01:
                printf("RESET_FPGA_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                invoke_py_callback(u16CmdCode, u16Status);
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
                dcaCaptureMode = DCA1000_CAPTURE_ONGOING;
                printf("RECORD_START_CMD_CODE Async event recieved [%d] \n", u16CmdCode);
                break;
            case 0x06:
                dcaCaptureMode = DCA1000_CAPTURE_STOP;
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

void EthernetEventHandlercallback(UINT16 u16CmdCode, UINT16 u16Status)
{
    printf("Call back called :  Async event recieved %d, %d \n", u16CmdCode, u16Status);
    if (u16CmdCode == 0x0A)
        handle_system_error(u16CmdCode, u16Status);
    else
        handle_cmd_code(u16CmdCode, u16Status);
}


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
