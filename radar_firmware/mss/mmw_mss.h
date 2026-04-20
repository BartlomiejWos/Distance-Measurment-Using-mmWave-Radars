/**
 *   @file  mmw_mss.h
 *
 *   @brief
 *      This is the main header file for the Millimeter Wave Demo
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2016 Texas Instruments, Inc.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef MMW_MSS_H
#define MMW_MSS_H
#define LVDS_STREAM
#ifdef __cplusplus
extern "C" {
#endif

#if defined (SOC_AWR294X)
#include <mmwavelink.h>
#include <drivers/uart.h>
#include <kernel/dpl/SemaphoreP.h>
#include "FreeRTOS.h"
#include "task.h"
//include <ti/common/mmwave_error.h>
#include <ti/common/syscommon.h>
//#include <ti/control/mmwave/mmwave.h>


#elif defined (SOC_AWR2544)
#include <mmwavelink.h>
#include <drivers/uart.h>
#include <kernel/dpl/SemaphoreP.h>
#include "FreeRTOS.h"
#include "task.h"
//include <ti/common/mmwave_error.h>
#include <ti/common/syscommon.h>
//#include <ti/control/mmwave/mmwave.h>
#else

#include <ti/common/sys_common.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Task.h>

#include <ti/common/mmwave_error.h>
#include <ti/drivers/osal/DebugP.h>
#include <ti/drivers/soc/soc.h>
#include <ti/drivers/uart/UART.h>
#include <ti/drivers/gpio/gpio.h>
#include <ti/drivers/mailbox/mailbox.h>
#endif
#include "common/mmw_config.h"
#include "mss/mmw_lvds_stream.h"
#include "mss/mmw_cli.h"

#define MMW_MMWAVELINK_DEVICE_MAP   RL_AR_DEVICETYPE_18XX /* AWR294x as well */

/* definition for xWR1843 device variant */
#ifdef SOC_XWR18XX
#define MMW_SOC_PIN_UART1_TX        SOC_XWR18XX_PINN5_PADBE
#define MMW_SOC_PIN_UART1_TX_FUNC   SOC_XWR18XX_PINN5_PADBE_MSS_UARTA_TX
#define MMW_SOC_PIN_UART1_RX        SOC_XWR18XX_PINN4_PADBD
#define MMW_SOC_PIN_UART1_RX_FUNC   SOC_XWR18XX_PINN4_PADBD_MSS_UARTA_RX
#define MMW_SOC_PIN_UART3_TX        SOC_XWR18XX_PINF14_PADAJ
#define MMW_SOC_PIN_UART3_TX_FUNC   SOC_XWR18XX_PINF14_PADAJ_MSS_UARTB_TX
#define MMW_SOC_PIN_GPIO2           SOC_XWR18XX_PINK13_PADAZ
#define MMW_SOC_PIN_GPIO2_FUNC      SOC_XWR18XX_PINK13_PADAZ_GPIO_2
#define MMW_SOC_PIN_GPIO2_OUT       SOC_XWR18XX_GPIO_2
#define MMW_MMWAVELINK_DEVICE_MAP   RL_AR_DEVICETYPE_18XX
#define MMW_MSS_FRAME_START_INT     SOC_XWR18XX_MSS_FRAME_START_INT
#endif
/* definition for xWR1642 device variant */
#ifdef SOC_XWR16XX
#define MMW_SOC_PIN_UART1_TX        SOC_XWR16XX_PINN5_PADBE
#define MMW_SOC_PIN_UART1_TX_FUNC   SOC_XWR16XX_PINN5_PADBE_MSS_UARTA_TX
#define MMW_SOC_PIN_UART1_RX        SOC_XWR16XX_PINN4_PADBD
#define MMW_SOC_PIN_UART1_RX_FUNC   SOC_XWR16XX_PINN4_PADBD_MSS_UARTA_RX
#define MMW_SOC_PIN_UART3_TX        SOC_XWR16XX_PINF14_PADAJ
#define MMW_SOC_PIN_UART3_TX_FUNC   SOC_XWR16XX_PINF14_PADAJ_MSS_UARTB_TX
#define MMW_SOC_PIN_GPIO2           SOC_XWR16XX_PINK13_PADAZ
#define MMW_SOC_PIN_GPIO2_FUNC      SOC_XWR16XX_PINK13_PADAZ_GPIO_2
#define MMW_SOC_PIN_GPIO2_OUT       SOC_XWR16XX_GPIO_2
#define MMW_MMWAVELINK_DEVICE_MAP   RL_AR_DEVICETYPE_16XX
#define MMW_MSS_FRAME_START_INT     SOC_XWR16XX_MSS_FRAME_START_INT
#endif
/* definition for xWR1443 device variant */
#ifdef SOC_XWR14XX
#define MMW_SOC_PIN_UART1_TX        SOC_XWR14XX_PINN6_PADBE
#define MMW_SOC_PIN_UART1_TX_FUNC   SOC_XWR14XX_PINN6_PADBE_MSS_UARTA_TX
#define MMW_SOC_PIN_UART1_RX        SOC_XWR14XX_PINN5_PADBD
#define MMW_SOC_PIN_UART1_RX_FUNC   SOC_XWR14XX_PINN5_PADBD_MSS_UARTA_RX
#define MMW_SOC_PIN_UART3_TX        SOC_XWR14XX_PINN6_PADBE_MSS_UARTB_TX
#define MMW_SOC_PIN_UART3_TX_FUNC   SOC_XWR14XX_PINN6_PADBE_MSS_UARTB_TX
#define MMW_SOC_PIN_GPIO2           SOC_XWR14XX_PINN13_PADAZ
#define MMW_SOC_PIN_GPIO2_FUNC      SOC_XWR14XX_PINN13_PADAZ_GPIO_2
#define MMW_SOC_PIN_GPIO2_OUT       SOC_XWR14XX_GPIO_2
#define MMW_MMWAVELINK_DEVICE_MAP   RL_AR_DEVICETYPE_14XX
#define MMW_MSS_FRAME_START_INT     SOC_XWR14XX_BSS_FRAME_START_INT
#endif
/* definition for xWR684x device variant */
#ifdef SOC_XWR68XX
#define MMW_SOC_PIN_UART1_TX        SOC_XWR68XX_PINN5_PADBE
#define MMW_SOC_PIN_UART1_TX_FUNC   SOC_XWR68XX_PINN5_PADBE_MSS_UARTA_TX
#define MMW_SOC_PIN_UART1_RX        SOC_XWR68XX_PINN4_PADBD
#define MMW_SOC_PIN_UART1_RX_FUNC   SOC_XWR68XX_PINN4_PADBD_MSS_UARTA_RX
#define MMW_SOC_PIN_UART3_TX        SOC_XWR68XX_PINF14_PADAJ
#define MMW_SOC_PIN_UART3_TX_FUNC   SOC_XWR68XX_PINF14_PADAJ_MSS_UARTB_TX
#define MMW_SOC_PIN_GPIO2           SOC_XWR68XX_PINK13_PADAZ
#define MMW_SOC_PIN_GPIO2_FUNC      SOC_XWR68XX_PINK13_PADAZ_GPIO_2
#define MMW_SOC_PIN_GPIO2_OUT       SOC_XWR68XX_GPIO_2
#define MMW_MMWAVELINK_DEVICE_MAP   RL_AR_DEVICETYPE_16XX
#define MMW_MSS_FRAME_START_INT     SOC_XWR68XX_MSS_FRAME_START_INT
#endif
/* definition for xWR64xx device variant */
#ifdef SOC_XWR64XX
#define MMW_SOC_PIN_UART1_TX        SOC_XWR64XX_PINN5_PADBE
#define MMW_SOC_PIN_UART1_TX_FUNC   SOC_XWR64XX_PINN5_PADBE_MSS_UARTA_TX
#define MMW_SOC_PIN_UART1_RX        SOC_XWR64XX_PINN4_PADBD
#define MMW_SOC_PIN_UART1_RX_FUNC   SOC_XWR64XX_PINN4_PADBD_MSS_UARTA_RX
#define MMW_SOC_PIN_UART3_TX        SOC_XWR64XX_PINF14_PADAJ
#define MMW_SOC_PIN_UART3_TX_FUNC   SOC_XWR64XX_PINF14_PADAJ_MSS_UARTB_TX
#define MMW_SOC_PIN_GPIO2           SOC_XWR64XX_PINK13_PADAZ
#define MMW_SOC_PIN_GPIO2_FUNC      SOC_XWR64XX_PINK13_PADAZ_GPIO_2
#define MMW_SOC_PIN_GPIO2_OUT       SOC_XWR64XX_GPIO_2
#define MMW_MMWAVELINK_DEVICE_MAP   RL_AR_DEVICETYPE_16XX
#define MMW_MSS_FRAME_START_INT     SOC_XWR64XX_MSS_FRAME_START_INT
#endif

/*! @brief For advanced frame config, below define means the configuration given is
 * global at frame level and therefore it is broadcast to all sub-frames.
 */
#define MMWDEMO_SUBFRAME_NUM_FRAME_LEVEL_CONFIG (-1)

/*! @brief CFAR threshold encoding factor
 */
#define MMWDEMO_CFAR_THRESHOLD_ENCODING_FACTOR (100.0)


/*! @brief For the ADCBufData.dataProperty.adcBits field
 */
#define ADCBUF_DATA_PROPERTY_ADCBITS_16BIT (2)

/**
 * @defgroup configStoreOffsets     Offsets for storing CLI configuration
 * @brief    Offsets of config fields within the parent structures, note these offsets will be
 *           unique and hence can be used to differentiate the commands for processing purposes.
 * @{
 */
#define MMWDEMO_GUIMONSEL_OFFSET                 (offsetof(MmwDemo_SubFrameCfg, guiMonSel))
#define MMWDEMO_ADCBUFCFG_OFFSET                 (offsetof(MmwDemo_SubFrameCfg, adcBufCfg))
#define MMWDEMO_LVDSSTREAMCFG_OFFSET             (offsetof(MmwDemo_SubFrameCfg, lvdsStreamCfg))

#define MMWDEMO_SUBFRAME_DYNCFG_OFFSET           (offsetof(MmwDemo_SubFrameCfg, objDetDynCfg) + \
                                                  offsetof(MmwDemo_DPC_ObjDet_DynCfg, dynCfg))

#define MMWDEMO_CFARCFGRANGE_OFFSET              (MMWDEMO_SUBFRAME_DYNCFG_OFFSET + \
                                                  offsetof(DPC_ObjectDetection_DynCfg, cfarCfgRange))

#define MMWDEMO_CFARCFGDOPPLER_OFFSET            (MMWDEMO_SUBFRAME_DYNCFG_OFFSET + \
                                                  offsetof(DPC_ObjectDetection_DynCfg, cfarCfgDoppler))

#define MMWDEMO_FOVRANGE_OFFSET                  (MMWDEMO_SUBFRAME_DYNCFG_OFFSET + \
                                                  offsetof(DPC_ObjectDetection_DynCfg, fovRange))

#define MMWDEMO_FOVDOPPLER_OFFSET                (MMWDEMO_SUBFRAME_DYNCFG_OFFSET + \
                                                  offsetof(DPC_ObjectDetection_DynCfg, fovDoppler))

#define MMWDEMO_FOVAOA_OFFSET                    (MMWDEMO_SUBFRAME_DYNCFG_OFFSET + \
                                                  offsetof(DPC_ObjectDetection_DynCfg, fovAoaCfg))

#define MMWDEMO_EXTMAXVEL_OFFSET                 (MMWDEMO_SUBFRAME_DYNCFG_OFFSET + \
                                                  offsetof(DPC_ObjectDetection_DynCfg, extMaxVelCfg))

#define MMWDEMO_MULTIOBJBEAMFORMING_OFFSET       (MMWDEMO_SUBFRAME_DYNCFG_OFFSET + \
                                                  offsetof(DPC_ObjectDetection_DynCfg, multiObjBeamFormingCfg))

#define MMWDEMO_CALIBDCRANGESIG_OFFSET           (MMWDEMO_SUBFRAME_DYNCFG_OFFSET + \
                                                  offsetof(DPC_ObjectDetection_DynCfg, calibDcRangeSigCfg))

#define MMWDEMO_STATICCLUTTERREMOFVAL_OFFSET     (MMWDEMO_SUBFRAME_DYNCFG_OFFSET + \
                                                  offsetof(DPC_ObjectDetection_DynCfg, staticClutterRemovalCfg))

#if defined (SOC_AWR2544)
#define RANGEPROCHWA_L3REUSE_MAX_ITERATIONS                                 3U
#define MMWDEMO_COMPRESSIONCFG_OFFSET MMWDEMO_SUBFRAME_STATICCFG_OFFSET + \
                                     (offsetof(MmwDemo_datapathCfg, \
                                      compressionCfg))
#define MMWDEMO_INTFMITIGCFG_OFFSET MMWDEMO_SUBFRAME_STATICCFG_OFFSET + \
                                    (offsetof(MmwDemo_datapathCfg, \
                                     intfStatsdBCfg))
/** @}*/ /* configStoreOffsets */

/*! \brief Debug Function */
#define MmwDemo_debugAssert(expression) \
                    {_MmwDemo_debugAssert(expression, __FILE__, __LINE__); \
                                                    DebugP_assert(expression);}

/* This value is written to specific register to gate the clock */
#define MMWDEMO_CLOCK_GATE 0x7

/* Bitmap for GPADC read */
#define CONFIG_GPADC_EXT_VOLT_CHANNEL_BITMAP 0x33

/* HWA Power Saving */
#define MMWDEMO_HWA_POWER_GATE 1U

typedef void* DPC_Handle;
/**
 * @brief
 *  Millimeter Wave Demo Sensor State
 *
 * @details
 *  The enumeration is used to define the sensor states used in mmwDemo
 */
typedef enum MmwDemo_SensorState_e
{
    /*!  @brief Inital state after sensor is initialized.
     */
    MmwDemo_SensorState_INIT = 0,

    /*!  @brief Inital state after sensor is post RF init.
     */
    MmwDemo_SensorState_OPENED,

    /*!  @brief Indicates sensor is started */
    MmwDemo_SensorState_STARTED,

    /*!  @brief  State after sensor has completely stopped */
    MmwDemo_SensorState_STOPPED
}MmwDemo_SensorState;

/**
 * @brief
 *  Millimeter Wave Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_MSS_Stats_t
{
    /*! @brief  Counter to track the number of frame trigger events from BSS */
    uint64_t     frameTriggerReady;

    /*! @brief   Counter which tracks the number of failed calibration reports
     *           The event is triggered by an asynchronous event from the BSS.*/
    uint32_t     failedTimingReports;

    /*! @brief   Counter which tracks the number of calibration reports
     *           received. The event is triggered by an asynchronous event
     *           from the BSS.
     */
    uint32_t     calibrationReports;

     /*! @brief  Counter which tracks the number of sensor stop events
      *          received. The event is triggered by an asynchronous event
      *          from the BSS.
      */
    uint32_t     sensorStopped;

}MmwDemo_MSS_Stats;

/**
 * @brief
 *  Millimeter Wave Demo Gui Monitor Selection
 *
 * @details
 *  The structure contains the selection for what information is placed to
 *  the output packet, and sent out to GUI. Unless otherwise specified,
 *  if the flag is set to 1, information
 *  is sent out. If the flag is set to 0, information is not sent out.
 *
 */
typedef struct MmwDemo_GuiMonSel_t
{
    /*! @brief   if 1: send the compressed FFT output
     *           if 0: Don't send anything */
    uint8_t        radarCube;
} MmwDemo_GuiMonSel;

/**
 * @brief
 *  Millimeter Wave Demo configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used to execute the Millimeter Wave Demo.
 */
typedef struct MmwDemo_Cfg_t
{
    /*! @brief   mmWave Control Configuration. */
    MMWave_CtrlCfg      ctrlCfg;

    /*! @brief   mmWave Open Configuration. */
    MMWave_OpenCfg      openCfg;

    /*! @brief   Datapath output loggerSetting
                 0 (default): MSS UART logger
                 1: DSS UART logger
     */
    uint8_t              dataLogger;
} MmwDemo_Cfg;

/**
 * @brief
 *  Millimeter Wave Demo Datapath configuration
 *
 * @details
 *  The structure is used to hold datapath related configuration.
 */
typedef struct MmwDemo_datapathCfg_t
{
    /*! @brief     Compression Cfg */
    DPU_RangeProcHWA_CompressionCfg compressionCfg;

    /*! @brief Shift/Scale config for Interf Stats Mag Diff in range DPU */
    DPU_RangeProcHWA_intfStatsdBCfg  intfStatsdBCfg;

} MmwDemo_datapathCfg;

/**
 * @brief Common Configuration independent of sub frame.
 */
typedef struct MmwDemo_PreStartCommonCfg_t
{
    /*! @brief   Number of sub-frames */
    uint8_t numSubFrames;

} MmwDemo_PreStartCommonCfg;


/**
 * @brief Common Configuration independent of sub frame.
 */
typedef struct MmwDemo_DPC_ObjDet_CommonCfg_t
{
    /*! @brief  Processing Chain 1-DDM 0: TDM */
    bool procChain;

    /*! @brief Is 2x mode enabled 1- enabled */
    bool is2xMode;

    /*! @brief Number of Chirps after which CPSW is to be triggered */
    uint16_t ethPktRdyCnt;

    /*! @brief Delay (us) to be added before each packet transfer */
    uint16_t ethPerPktDly;

    /*! @brief pre start common config */
    MmwDemo_PreStartCommonCfg   preStartCommonCfg;
} MmwDemo_DPC_ObjDet_CommonCfg;

/**
 * @brief
 *  Millimeter Wave Demo Data Path Information.
 *
 * @details
 *  The structure is used to hold all the relevant information for
 *  the data path.
 */
typedef struct MmwDemo_SubFrameCfg_t
{
    /*! @brief ADC buffer configuration storage */
    MmwDemo_ADCBufCfg adcBufCfg;

#ifdef LVDS_STREAM
    /*! @brief  LVDS stream configuration */
    MmwDemo_LvdsStreamCfg lvdsStreamCfg;
#endif

    /*! @brief GUI Monitor selection configuration storage from CLI */
    MmwDemo_GuiMonSel guiMonSel;

    /*! @brief  Number of range FFT bins, this is at a minimum the next power of 2 of
                numAdcSamples. If range zoom is supported, this can be bigger than
                the minimum. */
    uint16_t    numRangeBins;

    /*! @brief  Number of Doppler FFT bins, this is at a minimum the next power of 2 of
                numDopplerChirps. If Doppler zoom is supported, this can be bigger
                than the minimum. */
    uint16_t    numDopplerBins;

    /*! @brief  ADCBUF will generate chirp interrupt event every this many chirps - chirpthreshold */
    uint8_t     numChirpsPerChirpEvent;

    /*! @brief  Number of bytes per RX channel, it is aligned to 16 bytes as required by ADCBuf driver  */
    uint32_t    adcBufChanDataSize;

    /*! @brief CQ signal & image band monitor buffer size */
    uint32_t    sigImgMonTotalSize;

    /*! @brief CQ RX Saturation monitor buffer size */
    uint32_t    satMonTotalSize;

    /*! @brief  Number of ADC samples */
    uint16_t    numAdcSamples;

    /*! @brief  Number of chirps per sub-frame */
    uint16_t    numChirpsPerSubFrame;

    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas;

    /*! @brief   Datapath static configuration */
    MmwDemo_datapathCfg     datapathStaticCfg;

} MmwDemo_SubFrameCfg;

/*!
 * @brief
 * Structure holds message stats information from data path.
 *
 * @details
 *  The structure holds stats information. This is a payload of the TLV message item
 *  that holds stats information.
 */
typedef struct MmwDemo_SubFrameStats_t
{
    /*! @brief   Frame processing stats */
//    MmwDemo_output_message_stats    outputStats;

    /*! @brief   Dynamic CLI configuration time in usec */
    uint32_t                        pendingConfigProcTime;

    /*! @brief   SubFrame Preparation time on MSS in usec */
    uint32_t                        subFramePreparationTime;
} MmwDemo_SubFrameStats;

/**
 * @brief Task handles storage structure
 */
typedef struct MmwDemo_TaskHandles_t
{
    /*! @brief   MMWAVE Control Task Handle */
#if (defined SOC_AWR294X) || (defined SOC_AWR2544)
    TaskHandle_t mmwaveCtrl;
    StaticTask_t mmwCtrlTaskObj;

    /*! @brief   ObjectDetection DPC related dpmTask */
    TaskHandle_t objDetDpmTask;

    /*! @brief   Demo init task */
    TaskHandle_t initTask;
#else
    Task_Handle mmwaveCtrl;

    /*! @brief   ObjectDetection DPC related dpmTask */
    Task_Handle objDetDpmTask;

    /*! @brief   Demo init task */
    Task_Handle initTask;
#endif
} MmwDemo_taskHandles;

/*!
 * @brief
 * Structure holds temperature information from Radar front end.
 *
 * @details
 *  The structure holds temperature stats information.
 */
typedef struct MmwDemo_temperatureStats_t
{

    /*! @brief   retVal from API rlRfTempData_t - can be used to know
                 if values in temperatureReport are valid */
    int32_t        tempReportValid;

    /*! @brief   detailed temperature report - snapshot taken just
                 before shipping data over UART */
    rlRfTempData_t temperatureReport;

} MmwDemo_temperatureStats;

/*!
 * @brief
 * Structure holds calibration save configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration save configuration.
 */
typedef struct MmwDemo_calibDataHeader_t
{
    /*! @brief      Magic word for calibration data header */
    uint32_t    magic;

    /*! @brief      Header length */
    uint32_t    hdrLen;

    /*! @brief      mmwLink version */
    rlSwVersionParam_t  linkVer;

    /*! @brief      RadarSS version */
    rlFwVersionParam_t  radarSSVer;

    /*! @brief      Data length */
    uint32_t    dataLen;

    /*! @brief      data padding to make sure calib data is 8 bytes aligned */
    uint32_t      padding;
} MmwDemo_calibDataHeader;

/*!
 * @brief
 * Structure holds calibration save configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration save configuration.
 */
typedef struct MmwDemo_calibCfg_t
{
    /*! @brief      Calibration data header for validation read from flash */
    MmwDemo_calibDataHeader    calibDataHdr;

    /*! @brief      Size of Calibraton data size includng header */
    uint32_t        sizeOfCalibDataStorage;

    /*! @brief      Enable/Disable calibration save process  */
    uint32_t        saveEnable;

    /*! @brief      Enable/Disable calibration restore process  */
    uint32_t        restoreEnable;

    /*! @brief      Flash Offset to restore the data from */
    uint32_t        flashOffset;
} MmwDemo_calibCfg;

/*!
 * @brief
 * Structure holds calibration restore configuration used during sensor open.
 *
 * @details
 *  The structure holds calibration restore configuration.
 */
typedef struct MmwDemo_calibData_t
{
    /*! @brief      Calibration data header for validation read from flash */
    MmwDemo_calibDataHeader    calibDataHdr;

    /*! @brief      Calibration data */
    rlCalibrationData_t               calibData;

    /*! @brief      Phase shift Calibration data */
    rlPhShiftCalibrationData_t     phaseShiftCalibData;

    /* Future: If more fields are added to this structure or RL definitions
        are changed, please add dummy padding bytes here if size of
        MmwDemo_calibData is not multiple of 8 bytes. */
} MmwDemo_calibData;

/*!
 * @brief
 * Structure holds Spread Spectrum configuration.
 *
 * @details
 *  The structure holds Spread Spectrum configuration.
 */
typedef struct MmwDemo_spreadSpectrumConfig_t
{
    /*! @brief      Moduldation depth in percentage */
    float           modDepth;

    /*! @brief      flag to check the valid Config */
    bool            isEnable;

    /*! @brief      Moduldation rate in KHz */
    uint8_t         modRate;

    /*! @brief      downSpread */
    uint8_t         downSpread;
} MmwDemo_spreadSpectrumConfig;
typedef struct MmwDemo_adcDataDithDelayCfg_t
{
    uint8_t     isDelayEn;
    uint8_t     isDitherEn;
    uint16_t    minDelay;
    uint16_t    ditherVal;
}MmwDemo_adcDataDithDelayCfg;

/*!
 * @brief
 * Structure holds power optimization configuration.
 *
 * @details
 *  The structure holds power optimization configuration given through CLI commands.
 */

typedef struct MmwDemo_powerMeas_t
{
    /*! @brief Flag to Gate HWA clock source. */
    uint32_t                 isHwaDynamicClockGating;

    /*! @brief Flag to Gate HWA clock after frame Processing. */
    uint32_t                 isHwaGateAfterFrameProc;

    /*! @brief MSS Core loading Percentage. */
    uint32_t                 mssLoadingPercent;

    /*! @brief Flag to load MSS core loading. */
    uint32_t                 mssLoading;

    /*! @brief Flag to enable run time calibration. */
    uint32_t                 runCalEn;

    /*! @brief Periodic calibration enable flag. */
    uint32_t                 runCalPeriodEn;

    /*! @brief Periodicity in number of frames. */
    uint32_t                 periodicTimeInFrames;

    /*! @brief Semaphore to trigger MSS loading task. */
    SemaphoreP_Object        frameStartSemaphore;

    /*! @brief FrameStart Time Stamp for triggering MSS Loadig task. */
    volatile uint64_t        frameStartTimeStamp;
} MmwDemo_powerMeas;

/**
 * @brief
 *  Millimeter Wave Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo.
 */
typedef struct MmwDemo_MSS_MCB_t
{
    /*! @brief      Configuration which is used to execute the demo */
    MmwDemo_Cfg                 cfg;

    /*! @brief      UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;

    /*! @brief      This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*! @brief      DPC Handle, used in datapath processing */
    DPC_Handle                  dpcHandle;

    /*! @brief      ADCBuf driver handle */
    ADCBuf_Handle               adcBufHandle;

    /*! @brief   Handle of the EDMA driver, used for CBUFF */
    EDMA_Handle                 edmaHandle;

    /*! @brief      Object Detection DPC common configuration */
    MmwDemo_DPC_ObjDet_CommonCfg objDetCommonCfg;

    /*! @brief      Object Detection DPC subFrame configuration */
    MmwDemo_SubFrameCfg         subFrameCfg[RL_MAX_SUBFRAMES];

    /*! @brief      sub-frame stats */
    MmwDemo_SubFrameStats       subFrameStats[RL_MAX_SUBFRAMES];

    /*! @brief      Demo Stats */
    MmwDemo_MSS_Stats           stats;

    /*! @brief      Task handle storage */
    MmwDemo_taskHandles         taskHandles;

    /*! @brief   Semaphore Object to pend main task */
    SemaphoreP_Object           demoInitTaskCompleteSemHandle;

    /*! @brief    Sensor state */
    MmwDemo_SensorState         sensorState;

    /*! @brief   Tracks the number of sensor start */
    uint32_t                    sensorStartCount;

    /*! @brief   Tracks the number of sensor sop */
    uint32_t                    sensorStopCount;

    /*! @brief   CQ monitor configuration - Signal Image band data */
    rlSigImgMonConf_t           cqSigImgMonCfg[RL_MAX_PROFILES_CNT];

    /*! @brief   CQ monitor configuration - Saturation data */
    rlRxSatMonConf_t            cqSatMonCfg[RL_MAX_PROFILES_CNT];

    /*! @brief   Analog monitor bit mask */
    MmwDemo_AnaMonitorCfg       anaMonCfg;

#ifdef LVDS_STREAM
    /*! @brief   this structure is used to hold all the relevant information
         for the mmw demo LVDS stream*/
    MmwDemo_LVDSStream_MCB_t    lvdsStream;
#endif

    float framePeriod;

    MmwDemo_powerMeas powerMeas;

    /*! @brief   this structure is used to hold all the relevant information
     for the temperature report*/
    MmwDemo_temperatureStats  temperatureStats;

    /*! @brief   Calibration cofiguration for save/restore */
    MmwDemo_calibCfg                calibCfg;

    /*! @brief   this structure is used to hold all the relevant information
     for the Core ADPLL SSC Configuration*/
    MmwDemo_spreadSpectrumConfig     coreAdpllSscCfg;

    /*! @brief   this structure is used to hold all the relevant information
     for the PER ADPLL SSC Configuration*/
    MmwDemo_spreadSpectrumConfig     perAdpllSscCfg;

    /*! @brief   ADC data dithering configuration */
    MmwDemo_adcDataDithDelayCfg  adcDataDithDelayCfg;
} MmwDemo_MSS_MCB;

/*
 * @brief Memory Configuration used during init API
 */
typedef struct MmwDemo_MemCfg_t
{
    /*! @brief   Start address of memory provided by the application
     *           from which DPC will allocate.
     */
    void *addr;

    /*! @brief   Size limit of memory allowed to be consumed by the DPC */
    uint32_t size;
} MmwDemo_MemCfg;


/*
 * @brief Configuration for DPC initialization.
 */
typedef struct MmwDemo_initDPCParams_t
{
   /*! @brief   Handle to the hardware accelerator */
   HWA_Handle hwaHandle;

   /*! @brief   Handle to the EDMA driver. */
   EDMA_Handle edmaHandle;

   /*! @brief L3 RAM configuration. DPC will allocate memory from this
    *         as needed and report the amount of memory consumed */
   MmwDemo_MemCfg L3RamCfg;

   /*! @brief Core Local RAM configuration (e.g L2 for R5F).
    *         DPC will allocate memory from this as needed and report the
    *         amount of memory consumed */
   MmwDemo_MemCfg CoreLocalRamCfg;

}MmwDemo_initDPCParams;

/*
 * @brief Static Configuration that is part of the pre-start configuration.
 */
typedef struct MmwDemo_staticDPCCfg_t
{
    /*! @brief      ADCBuf buffer interface */
    DPIF_ADCBufData  ADCBufData;

    /*! @brief  Number of transmit antennas */
    uint8_t     numTxAntennas;

    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas;

    /*! @brief  Number of range FFT bins, this is at a minimum the next power
     * of 2 of @ref DPIF_ADCBufProperty_t::numAdcSamples, in case of complex
     * ADC data,and half that value in case of real only data. If range zoom
     * is supported, this can be bigger than the minimum.
     */
    uint16_t    numRangeBins;

    /*! @brief  Number of bins used in Range FFT Calculation. In case of real
     * only samples, this is twice the number of range bins. Else it is equal * to the number of range bins. */
    uint16_t    numRangeFFTBins;

    /*! @brief  Number of chirps per frame */
    uint16_t    numChirpsPerFrame;

    /*! @brief Number of chirps for Doppler computation purposes. */
    uint16_t    numChirps;

    /*! @brief 1 if 2X Mode is enabled for real data */
    uint16_t    isMode2x;

    /*! @brief  Number of Chirps to be Stored in L3RAM in ecah iteration */
    uint16_t    numChirpsEachIter[RANGEPROCHWA_L3REUSE_MAX_ITERATIONS];

    /*! @brief  Application header size per packet*/
    uint32_t    appHeaderSize;

    /*! @brief  Application footer size per packet*/
    uint32_t    appFooterSize;

    /*! @brief  Number of Payloads per chirp*/
    uint32_t    numPayloads;

    /*! @brief  Radar Cube Data Size including headear and footer payload*/
    uint32_t    radarCubeDataSize;

    /*! @brief  Compression Cfg */
    DPU_RangeProcHWA_CompressionCfg compressionCfg;

    /*! @brief Shift/Scale config for Interf Stats Mag Diff in range DPU */
    DPU_RangeProcHWA_intfStatsdBCfg  intfStatsdBCfg;

} MmwDemo_staticDPCCfg;

/**
 * @brief  Structure related to memory usage as part of
 * \ref @MmwDemo_preStartDPCCfg.
 */
typedef struct MmwDemo_memUsageDPC_t
{
    /*! @brief   number of bytes of L3 memory allocated to be used by DPC */
    uint32_t L3RamTotal;

    /*! @brief  number of bytes of L3 memory used by DPC from the allocated
     *           amount indicated through @ref MmwDemo_initDPCParams */
    uint32_t L3RamUsage;

    /*! @brief  number of bytes of Local memory allocated to be used by DPC */
    uint32_t CoreLocalRamTotal;

    /*! @brief  number of bytes of Local memory used by DPC from the allocated
     *           amount indicated through @ref MmwDemo_initDPCParams */
    uint32_t CoreLocalRamUsage;

    /*! @brief Indicates number of bytes of system heap allocated */
    uint32_t SystemHeapTotal;

    /*! @brief number of bytes of heap used at the end of PreStartCfg */
    uint32_t SystemHeapUsed;

    /*! @brief number of bytes of heap used by DPC at the end of PreStartCfg */
    uint32_t SystemHeapDPCUsed;
} MmwDemo_memUsageDPC;

/*
 * @brief Configuration done before starting execution. Unique to each
 * subrframe.
 */
typedef struct MmwDemo_preStartDPCCfg_t
{
    /*! @brief   Subframe number for which this message is applicable. When
     *           advanced frame is not used, this should be set to
     *           0 (the 1st and only sub-frame) */
    uint8_t subFrameNum;

    /*! Static configuration */
    MmwDemo_staticDPCCfg staticCfg;

    /*! Memory usage after the preStartCfg is applied */
    MmwDemo_memUsageDPC memUsage;
} MmwDemo_preStartDPCCfg;

/* ========================================================================== */
/*                            Global Variables                                */
/* ========================================================================== */

extern MmwDemo_MSS_MCB gMmwMssMCB;

/* CPSW Toggle register Values */
extern const uint32_t gToggleValPing;
extern const uint32_t gToggleValPong;

/**************************************************************************
 ************************* Function Declarations **************************
 **************************************************************************/

/* Function to initialize the CLI. */
extern void MmwDemo_CLIInit(uint8_t taskPriority);

/* Functions to handle the actions need to move the sensor state */
extern int32_t MmwDemo_openSensor(bool isFirstTimeOpen);
extern int32_t MmwDemo_configSensor(void);
//extern int32_t MmwDemo_startSensor(void);
extern void MmwDemo_stopSensor(void);

/*! \brief  Utility function to apply configuration to specified sub-frame */
extern void MmwDemo_CfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum);
/*! \brief Spread Spectrum Configuration Function*/
extern void MmwDemo_configSSC(void);

/*! Debug Functions */
extern void _MmwDemo_debugAssert(int32_t expression, const char *file,\
                             int32_t line);

/*! \brief Functions related to Datapath Chain (DPC) */
extern void MmwDPC_configAssistEDMAChannel(EDMA_Handle handle, uint32_t chNum,
                                uint32_t shadowParam, uint32_t eventQueue,
                                DPEDMA_ChanCfg *chanCfg);
extern void MmwDPC_configAssistEDMAChannelThreeLinks(EDMA_Handle handle,
                                uint32_t chNum, uint32_t shadowParam1,
                                uint32_t shadowParam2, uint32_t shadowParam3,
                                uint32_t eventQueue,
                                DPEDMA_3LinkChanCfg *chanCfg);
extern void MmwDPC_frameStartISR(void* arg);
extern DPC_Handle MmwDPC_init(MmwDemo_initDPCParams *ptrInitCfg, \
                                int32_t *errCode);
extern void MmwDPC_start(DPC_Handle handle, int32_t *errCode);
extern void MmwDPC_preStartCommonConfig(DPC_Handle handle,\
                        MmwDemo_PreStartCommonCfg *ptrCommonCfg,\
                        int32_t *errCode);
extern void MmwDPC_preStartConfig(DPC_Handle handle, \
                        MmwDemo_preStartDPCCfg *ptrPreStartCfg, \
                        int32_t *errCode);
extern void MmwDPC_execute(DPC_Handle handle, int32_t *errCode);
extern void MmwDPC_deinit(DPC_Handle handle, int32_t *errCode);
extern void MmwDPC_stop(DPC_Handle handle, int32_t *errCode);

#ifdef POWER_MEAS
/*! \brief
 * Function that executes Matrix multiplication for user
 * provided loading percentage
 */
void MmwPm_MssLoadingTask(void * args);
#endif

/**
 *  @b Description
 *  @n
 *     Initializes CPSW interface and and open CPDMA handle for
 *     Tx and Rx channels. Also etablishes DP83867 ethernet PHY.
 *
 *  @retval   None
 */
void MmwEnet_init(void);

/**
 *  @b Description
 *  @n
 *     Configures Tx Buffer descriptors, initializes NW packet buffer
 *     descriptors and enables FHost CPDMA hardware controlled packet
 *     transmission feature
 *
 *  @param[in]  ptrStaticCfg   Pointer to MmwDemo_staticDPCCfg to
 *                             compute Payload size
 *
 *  @retval   None
 */
void MmwEnet_config(MmwDemo_staticDPCCfg *ptrStaticCfg);

/**
 *  @b Description
 *  @n
 *     Free and dequeue Tx buffer descriptors.
 *
 *  @retval   None
 */
void MmwEnet_freeEthTxPkt(void);

/**
 *  @b Description
 *  @n
 *     Initializes RTI Timer B for HWA wakeup.
 *
 *  @retval   None
 */
extern void MmwDPC_hwaPGTimerInit(void);

/**
 *  @b Description
 *  @n
 *     prints CPSW Host and MAC port stats.
 *
 *  @param[in]  None
 *
 *  @retval   None
 */
void MmwEnet_showCpswStats(void);

#ifdef __cplusplus
}
#endif



#else

/** @}*/ /* configStoreOffsets */

/**
 * @brief
 *  Millimeter Wave Demo Sensor State
 *
 * @details
 *  The enumeration is used to define the sensor states used in mmwDemo
 */
typedef enum MmwDemo_SensorState_e
{
    /*!  @brief Inital state after sensor is initialized.
     */
    MmwDemo_SensorState_INIT = 0,

    /*!  @brief Inital state after sensor is post RF init.
     */
    MmwDemo_SensorState_OPENED,

    /*!  @brief Indicates sensor is started */
    MmwDemo_SensorState_STARTED,

    /*!  @brief  State after sensor has completely stopped */
    MmwDemo_SensorState_STOPPED
}MmwDemo_SensorState;

/**
 * @brief
 *  Millimeter Wave Demo statistics
 *
 * @details
 *  The structure is used to hold the statistics information for the
 *  Millimeter Wave demo
 */
typedef struct MmwDemo_MSS_Stats_t
{
    /*! @brief   Counter which tracks the number of frame trigger events from BSS */
    uint64_t     frameTriggerReady;
    
    /*! @brief   Counter which tracks the number of failed calibration reports
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     failedTimingReports;

    /*! @brief   Counter which tracks the number of calibration reports received
     *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     calibrationReports;

     /*! @brief   Counter which tracks the number of sensor stop events received
      *           The event is triggered by an asynchronous event from the BSS */
    uint32_t     sensorStopped;
}MmwDemo_MSS_Stats;

/**
 * @brief
 *  Millimeter Wave Demo Data Path Information.
 *
 * @details
 *  The structure is used to hold all the relevant information for
 *  the data path.
 */
typedef struct MmwDemo_SubFrameCfg_t
{
    /*! @brief ADC buffer configuration storage */
    Mmw_ADCBufCfg adcBufCfg;

    /*! @brief Flag indicating if @ref adcBufCfg is pending processing. */
    uint8_t isAdcBufCfgPending : 1;

    /*! @brief  LVDS stream configuration */
    MmwDemo_LvdsStreamCfg lvdsStreamCfg;

    /*! @brief Flag indicating if @ref lvdsStreamCfg is pending processing. */
    uint8_t isLvdsStreamCfgPending : 1;

    /*! @brief GUI Monitor selection configuration storage from CLI */
    MmwDemo_GuiMonSel guiMonSel;

    /*! @brief  Number of range FFT bins, this is at a minimum the next power of 2 of
                numAdcSamples. If range zoom is supported, this can be bigger than
                the minimum. */
    uint16_t    numRangeBins;

    /*! @brief  Number of Doppler FFT bins, this is at a minimum the next power of 2 of
                numDopplerChirps. If Doppler zoom is supported, this can be bigger
                than the minimum. */
    uint16_t    numDopplerBins;

    /*! @brief  ADCBUF will generate chirp interrupt event every this many chirps - chirpthreshold */
    uint8_t     numChirpsPerChirpEvent;

    /*! @brief  Number of bytes per RX channel, it is aligned to 16 bytes as required by ADCBuf driver  */
    uint32_t    adcBufChanDataSize;

    /*! @brief CQ signal & image band monitor buffer size */
    uint32_t    sigImgMonTotalSize;

    /*! @brief CQ RX Saturation monitor buffer size */
    uint32_t    satMonTotalSize;

    /*! @brief  Number of ADC samples */
    uint16_t    numAdcSamples;

    /*! @brief  Number of chirps per sub-frame */
    uint16_t    numChirpsPerSubFrame;
    
    /*! @brief  Number of virtual antennas */
    uint8_t     numVirtualAntennas; 
} MmwDemo_SubFrameCfg;
#if 0
/*!
 * @brief
 * Structure holds message stats information from data path.
 *
 * @details
 *  The structure holds stats information. This is a payload of the TLV message item
 *  that holds stats information.
 */
typedef struct MmwDemo_SubFrameStats_t
{
    /*! @brief   Frame processing stats */
    MmwDemo_output_message_stats    outputStats;

    /*! @brief   Dynamic CLI configuration time in usec */
    uint32_t                        pendingConfigProcTime;

    /*! @brief   SubFrame Preparation time on MSS in usec */
    uint32_t                        subFramePreparationTime;
} MmwDemo_SubFrameStats;
#endif
/**
 * @brief Task handles storage structure
 */
typedef struct MmwDemo_TaskHandles_t
{
    /*! @brief   MMWAVE Control Task Handle */
#if defined(SOC_AWR294X)
    TaskHandle_t mmwaveCtrl;
    StaticTask_t mmwCtrlTaskObj;

    /*! @brief   ObjectDetection DPC related dpmTask */
    TaskHandle_t objDetDpmTask;

    /*! @brief   Demo init task */
    TaskHandle_t initTask;
#else
    Task_Handle mmwaveCtrl;

    /*! @brief   ObjectDetection DPC related dpmTask */
    Task_Handle objDetDpmTask;

    /*! @brief   Demo init task */
    Task_Handle initTask;
#endif
} MmwDemo_taskHandles;

/**
 * @brief
 *  Millimeter Wave Demo MCB
 *
 * @details
 *  The structure is used to hold all the relevant information for the
 *  Millimeter Wave demo.
 */
typedef struct MmwDemo_MSS_MCB_t
{
    /*! @brief      Configuration which is used to execute the demo */
    MmwDemo_Cfg                 cfg;

#if !defined(SOC_AWR294X)
    /*! * @brief    Handle to the SOC Module */
    SOC_Handle                  socHandle;

    /*! @brief   EDMA error Information when there are errors like missing events */
    EDMA_errorInfo_t            EDMA_errorInfo;

    /*! @brief EDMA transfer controller error information. */
    EDMA_transferControllerErrorInfo_t EDMA_transferControllerErrorInfo;
#endif
    /*! @brief      UART Logging Handle */
    UART_Handle                 loggingUartHandle;

    /*! @brief      UART Command Rx/Tx Handle */
    UART_Handle                 commandUartHandle;

    /*! @brief      This is the mmWave control handle which is used
     * to configure the BSS. */
    MMWave_Handle               ctrlHandle;

    /*! @brief      ADCBuf driver handle */
    ADCBuf_Handle               adcBufHandle;

    /*! @brief   Handle of the EDMA driver, used for CBUFF */
    EDMA_Handle                 edmaHandle;

    /*! @brief   Number of EDMA event Queues (tc) */
    uint8_t                     numEdmaEventQueues;

    /*! @brief   True if need to poll for edma error (because error interrupt is not
     *           connected to the CPU on the device */
    bool                        isPollEdmaError;

    /*! @brief   True if need to poll for edma transfer controller error
     *           because at least one of the transfer controller error interrupts
     *           are not connected to the CPU on the device */
    bool                        isPollEdmaTransferControllerError;

    /*! @brief      DPM Handle */
    DPM_Handle                  objDetDpmHandle;

    /*! @brief      Object Detection DPC subFrame configuration */
    MmwDemo_SubFrameCfg         subFrameCfg[RL_MAX_SUBFRAMES];

    /*! @brief      sub-frame stats */
//    MmwDemo_SubFrameStats       subFrameStats[RL_MAX_SUBFRAMES];

    /*! @brief      Demo Stats */
    MmwDemo_MSS_Stats           stats;

    /*! @brief      Task handle storage */
    MmwDemo_taskHandles         taskHandles;

    /*! @brief   Rf frequency scale factor, = 2.7 for 60GHz device, = 3.6 for 76GHz device */
    double                      rfFreqScaleFactor;

#if defined(SOC_AWR294X)
    /*! @brief   Semaphore Object to signal DPM started from DPM report function */
    SemaphoreP_Object            DPMstartSemHandle;

    /*! @brief   Semaphore Object to signal DPM stopped from DPM report function. */
    SemaphoreP_Object            DPMstopSemHandle;

    /*! @brief   Semaphore Object to signal DPM ioctl from DPM report function. */
    SemaphoreP_Object            DPMioctlSemHandle;
#else
    /*! @brief   Semaphore handle to signal DPM started from DPM report function */
    Semaphore_Handle            DPMstartSemHandle;

    /*! @brief   Semaphore handle to signal DPM stopped from DPM report function. */
    Semaphore_Handle            DPMstopSemHandle;

    /*! @brief   Semaphore handle to signal DPM ioctl from DPM report function. */
    Semaphore_Handle            DPMioctlSemHandle;
#endif
    /*! @brief    Sensor state */
    MmwDemo_SensorState         sensorState;

    /*! @brief   Tracks the number of sensor start */
    uint32_t                    sensorStartCount;

    /*! @brief   Tracks the number of sensor sop */
    uint32_t                    sensorStopCount;

    /*! @brief   CQ monitor configuration - Signal Image band data */
    rlSigImgMonConf_t           cqSigImgMonCfg[RL_MAX_PROFILES_CNT];

    /*! @brief   CQ monitor configuration - Saturation data */
    rlRxSatMonConf_t            cqSatMonCfg[RL_MAX_PROFILES_CNT];

    /*! @brief   Analog monitor bit mask */
   // MmwDemo_AnaMonitorCfg       anaMonCfg;

    /*! @brief   this structure is used to hold all the relevant information
         for the mmw demo LVDS stream*/
    MmwDemo_LVDSStream_MCB_t    lvdsStream;
    /*! @brief   calibration time unit configuration. */
    uint16_t            calibMonTimeUnit;

    /*! @brief   calibration period configuration. */
    uint32_t            calibPeriodicity;
    
     /*! @brief   Monitors Enabled. */
    uint8_t            monitorEnable;

    /*! @brief Flag indicating if @ref anaMonCfg is pending processing. */
    uint8_t isAnaMonCfgPending : 1;
    
} MmwDemo_MSS_MCB;

/**************************************************************************
 *************************** Extern Definitions ***************************
 **************************************************************************/

/* Functions to handle the actions need to move the sensor state */
extern int32_t MmwDemo_openSensor(bool isFirstTimeOpen);
extern int32_t MmwDemo_configSensor(void);
extern int32_t MmwDemo_startSensor(bool isFirstTime);
extern void MmwDemo_stopSensor(void);

/* functions to manage the dynamic configuration */
extern uint8_t MmwDemo_isAllCfgInPendingState(void);
extern uint8_t MmwDemo_isAllCfgInNonPendingState(void);
extern void MmwDemo_resetStaticCfgPendingState(void);
extern void MmwDemo_CfgUpdate(void *srcPtr, uint32_t offset, uint32_t size, int8_t subFrameNum);
extern void MmwDemo_gpioStateCtrl(uint32_t ledState);
extern void deactivateLVDSSession();

/* Debug Functions */
extern void _MmwDemo_debugAssert(int32_t expression, const char *file, int32_t line);
#define MmwDemo_debugAssert(expression) {                                      \
                                         _MmwDemo_debugAssert(expression,      \
                                                  __FILE__, __LINE__);         \
                                         DebugP_assert(expression);             \
                                        }

#ifdef __cplusplus
}
#endif
#endif
#endif /* MMW_MSS_H */
