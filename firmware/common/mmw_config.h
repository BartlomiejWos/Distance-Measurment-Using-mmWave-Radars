/**
 *   @file  mmw_config.h
 *
 *   @brief
 *      This is the header file that describes configurations for the Millimeter
 *   Wave Demo.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2018 Texas Instruments, Inc.
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
#ifndef MMW_CONFIG_H
#define MMW_CONFIG_H

/* MMWAVE library Include Files */
#include <ti/control/mmwave/mmwave.h>
#if (defined SOC_AWR294X) || (defined SOC_AWR2544) 
#include <ti/common/syscommon.h>
#else
#include <ti/common/sys_common.h>
#endif

#include <ti/datapath/dpc/objectdetection/objdethwa/objectdetection.h>

#ifdef __cplusplus
extern "C" {
#endif

#if !((defined SOC_AWR294X) || (defined SOC_AWR2544))
#define CRC_TYPE_16BIT        0
#define CRC_TYPE_32BIT        1
#define CRC_TYPE_64BIT        2
#endif


/* Change these configuration as required */
#if defined(SOC_XWR18XX)
#define MMW_FREQUENCY_LIMIT_LOW            77U
#define MMW_FREQUENCY_LIMIT_HIGH           81U

#elif defined(SOC_XWR68XX)
#define MMW_FREQUENCY_LIMIT_LOW            60U
#define MMW_FREQUENCY_LIMIT_HIGH           64U
#endif

/* Set the CRC Type for mmWaveLink communication with RadarSS */
#define MMWAVELINK_CRC_TYPE                 CRC_TYPE_32BIT

/* Monitoring Related configurations */
#define MMW_CALIB_MON_TIME_UNIT               1U

/* disable frame start async-event */
#define MMW_FRAME_START_ASYNC_EVENT_DISABLE   1U
/* disable frame Stop async-event */
#define MMW_FRAME_STOP_ASYNC_EVENT_DISABLE    1U


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
#if !defined (SOC_AWR2544)
typedef struct MmwDemo_GuiMonSel_t
{
    /*! @brief   if 1: Send list of detected objects (see @ref DPIF_PointCloudCartesian) and
     *                 side info (@ref DPIF_PointCloudSideInfo).\n
     *           if 2: Send list of detected objects only (no side info)\n
     *           if 0: Don't send anything */
    uint8_t        detectedObjects;

    /*! @brief   Send log magnitude range array  */
    uint8_t        logMagRange;

    /*! @brief   Send noise floor profile */
    uint8_t        noiseProfile;

    /*! @brief   Send complex range bins at zero doppler, all antenna symbols for range-azimuth heat map */
    uint8_t        rangeAzimuthHeatMap;

    /*! @brief   Send complex range bins at zero doppler, (all antenna symbols), for range-azimuth heat map */
    uint8_t        rangeDopplerHeatMap;

    /*! @brief   Send stats */
    uint8_t        statsInfo;
} MmwDemo_GuiMonSel;
#endif
/**
 * @brief
 *  LVDS streaming configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  for the LVDS streaming.
 */
#if !defined (SOC_AWR2544)
typedef struct MmwDemo_LvdsStreamCfg_t
{
    /**
     * @brief  HSI Header enabled/disabled flag. Only applicable for HW streaming.
     *         Will be ignored for SW streaming which will always have HSI header.
     */
    bool        isHeaderEnabled;

    /*! HW STREAMING DISABLED */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED   0

    /*! ADC */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC        1

    /*! CP_ADC_CQ */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ  4

    /*! HW streaming data format:
        0-HW STREAMING DISABLED
        1-ADC
        2-Reserved
        3-Reserved
        4-CP_ADC_CQ
    */
    uint8_t     dataFmt;

    /**
     * @brief  SW enabled/disabled flag
     */
    bool        isSwEnabled;
}MmwDemo_LvdsStreamCfg;
#endif

#if defined (SOC_AWR2544)
/**
 * @brief
 *  ADCBUF configuration (meant for CLI configuration)
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used to configure ADCBUF.
 *
 *  \ingroup MMWDEMO_ADCCONFIG_EXTERNAL_DATA_STRUCTURE
 */
typedef struct MmwDemo_ADCBufCfg_t
{
    /*! ADCBUF out format:
        0-Complex,
        1-Real */
    uint8_t     adcFmt;

    /*! ADCBUF IQ swap selection:
        0-I in LSB, Q in MSB,
        1-Q in LSB, I in MSB */
    uint8_t     iqSwapSel;

    /*! ADCBUF channel interleave configuration:
        0-interleaved(not supported on XWR16xx),
        1- non-interleaved */
    uint8_t     chInterleave;

    /**
     * @brief   Chirp Threshold configuration used for ADCBUF buffer
     */
    uint8_t     chirpThreshold;
}MmwDemo_ADCBufCfg;

typedef struct MmwDemo_AnaMonitorCfg_t
{
    /*! @brief   Setting for Rx Saturation monitor */
    uint8_t        rxSatMonEn;

    /*! @brief   Setting for signal & image band monitor  */
    uint8_t        sigImgMonEn;
} MmwDemo_AnaMonitorCfg;

/**
 * @brief
 *  LVDS streaming configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  for the LVDS streaming.
 */
typedef struct MmwDemo_LvdsStreamCfg_t
{
    /**
     * @brief  HSI Header enabled/disabled flag. Only applicable for HW
     * streaming. Will be ignored for SW streaming which will always have HSI
     * header.
     */
    bool        isHeaderEnabled;

    /*! HW STREAMING DISABLED */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_DISABLED   0

    /*! ADC */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_ADC        1

    /*! CP_ADC_CQ */
#define MMW_DEMO_LVDS_STREAM_CFG_DATAFMT_CP_ADC_CQ  4

    /*! HW streaming data format:
        0-HW STREAMING DISABLED
        1-ADC
        2-Reserved
        3-Reserved
        4-CP_ADC_CQ
    */
    uint8_t     dataFmt;

    /**
     * @brief  SW enabled/disabled flag
     */
    bool        isSwEnabled;
}MmwDemo_LvdsStreamCfg;

/**
 * @brief
 *  RangeProcHWA Compression hardware resources
 *
 * @details
 *  The structure is used to hold the hardware resources needed for compression of Range FFT
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_CompressionCfg
{
    /*! @brief Flag that indicates if compression is enabled */
    bool  isEnabled;

    /*! @brief Compression Method, 0 indicates EGE and 1 indicates BFP */
    uint8_t  compressionMethod;

    /*! @brief Compression ration, a value between 0 and 1 */
    float  compressionRatio;

    /*! @brief Indicates the number of range bins to be compressed in a single compression block */
    uint16_t  rangeBinsPerBlock;

    /*! @brief Can be greater than 1 only for DPIF_RADARCUBE_FORMAT_2.
               For DPIF_RADARCUBE_FORMAT_1 this should be set to 1 */
    uint16_t  numRxAntennaPerBlock;

}DPU_RangeProcHWA_CompressionCfg;

/**
 * @brief
 *  RangeProcHWA Interference mitigation configuration
 *
 * @details
 *  The structure is used to hold the hardware resources needed for interference mitigation in Range DPU
 *
 *  \ingroup DPU_RANGEPROC_EXTERNAL_DATA_STRUCTURE
 */
typedef struct DPU_RangeProcHWA_intfStatsdBCfg_t
{

    /*! @brief Interference mitigation magnitude SNR in dB */
    uint32_t intfMitgMagSNRdB;

    /*! @brief Interference mitigation magdiff SNR in dB */
    uint32_t intfMitgMagDiffSNRdB;

}DPU_RangeProcHWA_intfStatsdBCfg;
#endif
/**
 * @brief
 *  Millimeter Wave Demo Data Path Information.
 *
 * @details
 *  The structure is used to hold all the relevant information for
 *  the data path.
 */
typedef struct MmwDemo_platformCfg_t
{
    /*! @brief   GPIO index for sensor status */
    uint32_t            SensorStatusGPIO;
    
    /*! @brief   CPU Clock Frequency. */
    uint32_t            sysClockFrequency;

    /*! @brief   UART Logging Baud Rate. */
    uint32_t            loggingBaudRate;

    /*! @brief   UART Command Baud Rate. */
    uint32_t            commandBaudRate;
} MmwDemo_platformCfg;

/**
 * @brief
 *  Millimeter Wave Demo configuration
 *
 * @details
 *  The structure is used to hold all the relevant configuration
 *  which is used to execute the Millimeter Wave Demo.
 */
#if !defined (SOC_AWR2544)
typedef struct MmwDemo_Cfg_t
{
    /*! @brief   mmWave Control Configuration. */
    MMWave_CtrlCfg      ctrlCfg;

    /*! @brief   mmWave Open Configuration. */
    MMWave_OpenCfg      openCfg;

    /*! @brief   Platform specific configuration. */
    MmwDemo_platformCfg platformCfg;


    /*! @brief   Datapath output loggerSetting
                 0 (default): MSS UART logger
                 1: DSS UART logger
     */
    uint8_t              dataLogger;
} MmwDemo_Cfg;
#endif

#ifdef __cplusplus
}
#endif

#endif /* MMW_CONFIG_H */
