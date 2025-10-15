/**
 *   @file  mmwdemo_rfparser.c
 *
 *   @brief
 *      Implements rf parser.
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/

/* Standard Include Files. */
#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <math.h>

#ifdef MMWDEMO_RFPARSER_DBG
/* enable float extended format in BIOS cfg file using System.extendedFormats
   to able to print %f values */
#include <xdc/runtime/System.h>
#endif

/* mmWave SDK Include files */
#if defined (SOC_AWR294X)
#include <ti/common/syscommon.h>


#elif defined (SOC_AWR2544) 
#include <ti/common/syscommon.h>
#else

#include <ti/common/sys_common.h>
#endif
/* MATH utils library Include files */
#include <ti/utils/mathutils/mathutils.h>
#include "common/mmw_rfparser.h"
#include "mss/mmw_cli.h"

/** @defgroup MMWDEMO_RFPARSER_INTERNAL       Mmwdemo RFparser Internal
 */

/**
@defgroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION            RF Parser Internal Functions
@ingroup MMWDEMO_RFPARSER_INTERNAL
@brief
*   The section has a list of all internal API which are not exposed to the external
*   applications.
*/
/**
@defgroup MMWDEMO_RFPARSER_INTERNAL_DATA_STRUCTURE      RF Parser Internal Data Structures
@ingroup MMWDEMO_RFPARSER_INTERNAL
@brief
*   The section has a list of all internal data structures which are used internally
*   by the RF Parser module.
*/
/**
@defgroup MMWDEMO_RFPARSER_INTERNAL_DEFINITION          RF Parser Internal Definitions
@ingroup MMWDEMO_RFPARSER_INTERNAL
@brief
*   The section has a list of all internal definitions which are used internally
*   by the RF Parser.
*/


/** @addtogroup MMWDEMO_RFPARSER_INTERNAL_DEFINITION
 *
 @{ */

/*! Speed of light in m/s expressed in float */
#define MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC (3.0e8)

/** @} */

/**
 * @brief
 *  Data Source Hardware configuration definition
 *
 * @details
 *  The structure describes the hardware related configuration for device and evm
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_DATA_STRUCTURE
 */
typedef struct MmwDemo_RFParserHwAttr_t
{
    /**
     * @brief   ADC buffer size
     */
    uint32_t      adcBufSize;

    /**
     * @brief   Tx Antenna mask for elevation, antenna pattern specific
     */
    uint8_t       elevTxAntMask;

    /**
     * @brief   Tx Antenna mask for azimuth, antenna pattern specific
     */
    uint8_t       azimTxAntMask;
} MmwDemo_RFParserHwAttr;

/*================================================================
               RF Parser platform dependent params
 ================================================================*/
#ifdef SOC_XWR14XX
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_XWR14XX_MSS_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#ifdef SOC_XWR16XX
#ifdef SUBSYS_DSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0,
    0x3
};
#endif

#ifdef SUBSYS_MSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0,
    0x3
};
#endif

#endif

#ifdef SOC_XWR18XX

#ifdef SUBSYS_MSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#ifdef SUBSYS_DSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg = 
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#endif

#ifdef SOC_XWR68XX

#ifdef SUBSYS_MSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#ifdef SUBSYS_DSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0x5
};
#endif

#endif
/* For awr294x */

#if defined (SOC_AWR294X)

#ifdef SUBSYS_MSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0xD
};
#endif

#ifdef SUBSYS_DSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg = 
{
    SOC_ADCBUF_SIZE,
    0x2,
    0xD
};
#endif

#endif

#if (defined SOC_AWR2544)

#ifdef SUBSYS_MSS
MmwDemo_RFParserHwAttr MmwDemo_RFParserHwCfg =
{
    SOC_ADCBUF_SIZE,
    0x2,
    0xD
};
#endif

#endif

/*================================================================
               RF Parser internal APIs
 ================================================================*/

/**
 *  @b Description
 *  @n
 *      Help Function to get frame period
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Frame period
 */
static float MmwDemo_RFParser_getFramePeriod_ms(MMW_CLI_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].subFramePeriodicity * 0.000005f);
    }
    else
    {
        return((float)ctrlCfg->u.frameCfg.frameCfg.framePeriodicity * 0.000005f);
    }
}


/**
 *  @b Description
 *  @n
 *      Help Function to get chirp start Index
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Chirp start index
 */
static uint16_t MmwDemo_RFParser_getChirpStartIdx(MMW_CLI_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].chirpStartIdx);
    }
    else if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_CONTINUOUS)
    {
        return 0; /* in case of continuous mode no number of chirps */
    }
    else
    {
        return(ctrlCfg->u.frameCfg.frameCfg.chirpStartIdx);
    }
}

/**
 *  @b Description
 *  @n
 *      Help Function to get chirp stop Index
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Chirp stop index
 */
static uint16_t MmwDemo_RFParser_getChirpEndIdx(MMW_CLI_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].chirpStartIdx +
              (ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numOfChirps - 1));
    }
    else if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_CONTINUOUS)
    {
        return 0; /* in case of continuous mode no number of chirps */
    }
    else
    {
        return(ctrlCfg->u.frameCfg.frameCfg.chirpEndIdx);
    }
}

/**
 *  @b Description
 *  @n
 *      Help Function to get number of loops
 *
 *  @param[in] ctrlCfg       Handle to MMWave control configuration
 *  @param[in] subFrameIndx  Sub frame index
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Number of loops
 */
static uint16_t MmwDemo_RFParser_getNumLoops(MMW_CLI_CtrlCfg *ctrlCfg, uint8_t subFrameIndx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIndx].numLoops);
    }
    else if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_CONTINUOUS)
    {
        return 1; /* in case of continuous mode NumOfLoop : 1 */
    }
    else
    {
        return(ctrlCfg->u.frameCfg.frameCfg.numLoops);
    }
}

/**
 *  @b Description
 *  @n
 *      Help Function to get profile handle
 *
 *  @param[in] ctrlCfg          Handle to MMWave control configuration
 *  @param[in] profileLoopIdx   Profile index
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Profile handle
 */
MMWave_ProfileHandle MmwDemo_RFParser_getProfileHandle(MMW_CLI_CtrlCfg *ctrlCfg, uint32_t profileLoopIdx)
{
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return(ctrlCfg->u.advancedFrameCfg.profileHandle[profileLoopIdx]);
    }
    else
    {
        return(ctrlCfg->u.frameCfg.profileHandle[profileLoopIdx]);
    }
}

/**
 *  @b Description
 *  @n
 *      Helper function that parses chirp Tx antenna configuration and extracts parameters
 *      needed for AoA configuration depending on the AoA DPU needs
 *
 *  @param[inout]   outParams               Pointer to parameters generated after parsing configuration
 *  @param[inout]   pFoundValidProfile      flag to indicate if this is valid profile; if not, advance to the next one
 *  @param[in]      frameTotalChirps        Total chirps in the frame (used for looping through validChirpTxEnBits)
 *  @param[in]      validChirpTxEnBits      Array of chirp's txEn bits. Dimension is provided by frameTotalChirps
 *  @param[in]      bpmEnabled              BPM flag, 0 -disabled, 1-enabled
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *      Fail        <0, one of error codes @ref MMWDEMO_RFPARSER_ERROR_CODE
 */
int32_t MmwDemo_RFParser_setAoAParams
(
    MmwDemo_RFParserOutParams  *outParams,
    bool                       *pFoundValidProfile,
    int16_t                     frameTotalChirps,
    uint16_t                   *validChirpTxEnBits,
    bool                        bpmEnabled
)
{
    
    uint32_t    chirpLoopIdx;
    bool        validProfileHasOneTxPerChirp = false;
    uint16_t    validProfileTxEn = 0;
    int32_t     retVal = 0;
    bool        validProfileHasElevation = false;
    int32_t     i;
        

    /* now loop through unique chirps and check if we found all of the ones
       needed for the frame and then determine the azimuth/elevation antenna
       configuration
     */
    if (*pFoundValidProfile==true) {
        int16_t nonElevFirstChirpIdx = -1;
        for (chirpLoopIdx = 0; chirpLoopIdx < frameTotalChirps; chirpLoopIdx++)
        {
            bool validChirpHasElevation = false;
            bool validChirpHasOneTxPerChirp = false;
            uint16_t chirpTxEn = validChirpTxEnBits[chirpLoopIdx];
            if (chirpTxEn == 0) {
                /* this profile doesn't have all the needed chirps */
                *pFoundValidProfile = false;
                break;
            }

            /* check if this is an elevation TX chirp - if yes, it is assumed to be 4Rx, 3Tx case */
            validChirpHasElevation = (chirpTxEn == MmwDemo_RFParserHwCfg.elevTxAntMask);
            validProfileHasElevation |= validChirpHasElevation;

            /* if not, then check the MIMO config */
            if (!validChirpHasElevation)
            {
                uint8_t     log2TxEn;

                if(bpmEnabled)
                {   /* In case of BPM, check if both TX antennas are enabled*/
                    if(chirpTxEn != MmwDemo_RFParserHwCfg.azimTxAntMask)
                    {
                        /* The frame is configured as BPM but this chirp does not enable both TX antennas*/
                        *pFoundValidProfile = false;
#ifdef MMWDEMO_RFPARSER_DBG
                        System_printf("Bad BPM configuration. chirpTxEn=%d for chirp %d \n",chirpTxEn,chirpLoopIdx);
#endif
                        break;
                    }
                }
                else
                {
                    /* If only TX antenna is enabled, then its 0x1<< log2TxEn should equal chirpTxEn */
                    log2TxEn = mathUtils_floorLog2(chirpTxEn);
                    validChirpHasOneTxPerChirp = (chirpTxEn == (1 << log2TxEn));
                }

                /* if this is the first chirp without elevation, record the chirp's
                   MIMO config as profile's MIMO config. We dont handle intermix
                   at this point */
                if (nonElevFirstChirpIdx == -1) {
                    validProfileHasOneTxPerChirp = validChirpHasOneTxPerChirp;
                    nonElevFirstChirpIdx = chirpLoopIdx;
                }

                /* if this is the first chirp, record the chirp's
                   MIMO config as profile's MIMO config. We dont handle intermix
                   at this point */
                if (chirpLoopIdx == 0)
                {
                    validProfileHasOneTxPerChirp = validChirpHasOneTxPerChirp;
                }
                /* check the chirp's MIMO config against Profile's MIMO config */
                if (validChirpHasOneTxPerChirp != validProfileHasOneTxPerChirp)
                {
                    /* this profile doesnt have all chirps with same MIMO config */
                    *pFoundValidProfile = false;
                    break;
                }
            }

            /* save the antennas actually enabled in this profile */
            validProfileTxEn |= chirpTxEn;
        }
    }

    /* found valid chirps for the frame; mark this profile valid */
    if (*pFoundValidProfile == true) {
        uint32_t        numTxAntAzim = 0;
        uint32_t        numTxAntElev = 0;

        outParams->numTxAntennas = 0;
        if (validProfileHasElevation)
        {
            numTxAntElev = 1;
        }

        if (bpmEnabled == true)
        {
            /* BPM eanbled */
            numTxAntAzim = 2;
        }
        else if (validProfileHasOneTxPerChirp == true)
        {
            uint8_t log2TxEn;

            while(validProfileTxEn)
            {
                log2TxEn = mathUtils_floorLog2(validProfileTxEn);
                if ((0x1 << log2TxEn) & MmwDemo_RFParserHwCfg.azimTxAntMask)
                {
                    numTxAntAzim++;
                }
                validProfileTxEn &= ~(0x1<< log2TxEn);
            }
        }
        else /* i.e. SIMO case, all chirps have same one Tx antenna. */
        {
            /* in this case, for AOA, Tx Azimuth antenna is only 1. */
            if ((validProfileTxEn & MmwDemo_RFParserHwCfg.azimTxAntMask) != 0)
            {
                numTxAntAzim = 1;
            }
            else
            {
                numTxAntAzim = 0;
            }
        }

#ifdef MMWDEMO_RFPARSER_DBG
        System_printf("Azimuth Tx: %d (MIMO:%d), Elev Tx:%d\n",
                       numTxAntAzim,validProfileHasOneTxPerChirp,numTxAntElev);
#endif

        outParams->numTxAntennas = numTxAntAzim + numTxAntElev;
        if (outParams->numTxAntennas > SYS_COMMON_NUM_TX_ANTENNAS)
        {
            retVal = MMWDEMO_RFPARSER_EINVAL_NUM_TX_ANTENNAS;
            goto exit;
        }

        outParams->numVirtualAntAzim = numTxAntAzim * outParams->numRxAntennas;
        outParams->numVirtualAntElev = numTxAntElev * outParams->numRxAntennas;
        outParams->numVirtualAntennas = outParams->numVirtualAntAzim +
                                              outParams->numVirtualAntElev;

        /* Sanity Check: Ensure that the number of antennas is within system limits */
        if ((outParams->numVirtualAntennas <= 0) ||
           (outParams->numVirtualAntennas > (SYS_COMMON_NUM_TX_ANTENNAS * SYS_COMMON_NUM_RX_CHANNEL)))
        {
            retVal = MMWDEMO_RFPARSER_EINVAL__NUM_VIRTUAL_ANTENNAS;
        }

        /* txAntOrder[] will be needed for Rx Channel Phase Measurement/Compensation routines */
        if (validProfileHasOneTxPerChirp)
        {
            for (i = 0; i < outParams->numTxAntennas; i++)
            {
                outParams->txAntOrder[i] = mathUtils_floorLog2(validChirpTxEnBits[i]);
            }
        }
        else
        {
            for (i = 0; i < outParams->numTxAntennas; i++)
            {
                outParams->txAntOrder[i] = i;
            }
        }

        outParams->validProfileHasOneTxPerChirp = validProfileHasOneTxPerChirp;
        
#ifdef MMWDEMO_RFPARSER_DBG
        System_printf("Ant setting virtualAzim: %d , virtual Elev :%d\n",
                        outParams->numVirtualAntAzim, outParams->numVirtualAntElev);
#endif
    }
    
exit:
    return (retVal);        
}

/**
 *  @b Description
 *  @n
 *      Helper function that parses Profile, Chirp and Frame config and extracts parameters
 *      needed for processing chain configuration
 *
 *  @param[out] outParams      Pointer to parameters generated after parsing configuration
 *  @param[in]  subFrameIdx    Sub-frame index
 *  @param[in]  openCfg              Pointer to the MMWave Open configuration
 *  @param[in]  ctrlCfg              Pointer to the MMWave Control configuration
 *  @param[in]  rfFreqScaleFactor RF frequency scale factor, see SOC_getDeviceRFFreqScaleFactor API
 *                                in SOC driver
 *  @param[in]  bpmEnabled     BPM flag, 0 -disabled, 1-enabled
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *      Fail        <0, one of error codes @ref MMWDEMO_RFPARSER_ERROR_CODE
 */
static int32_t MmwDemo_RFParser_parseCtrlConfig
(
    MmwDemo_RFParserOutParams  *outParams,
    uint8_t              subFrameIdx,
    MMW_CLI_OpenCfg      *openCfg,
    MMW_CLI_CtrlCfg      *ctrlCfg,
    MMW_CLI_ProfChirpCfg  *profChirpCfg,
    float               rfFreqScaleFactor,
    bool                bpmEnabled
)
{
    uint16_t    frameChirpStartIdx;
    uint16_t    frameChirpEndIdx;
    int16_t     frameTotalChirps;
    uint32_t    profileLoopIdx;
    bool        foundValidProfile = false;
#if (defined SOC_AWR294X) || (defined SOC_AWR2544)
    uint16_t    channelTxEn = openCfg->chCfg.txChannelEn;
    uint16_t    channelRxEn = openCfg->chCfg.rxChannelEn;
#else
    uint16_t    channelTxEn = openCfg->chCfg.txChannelEn;
    uint16_t    channelRxEn = openCfg->chCfg.rxChannelEn;
#endif
    uint8_t     channel;
    uint8_t     numRxAntennas;
    uint16_t    numLoops;
    int32_t     retVal = 0;
    float       bandwidth, centerFreq, adcStart, slope, startFreq;
    rlUInt32_t startFreqConst;
    rlUInt16_t digOutSampleRate;

    /***********************************************
     * Sanity Check on ADC configuration
     ***********************************************/
    /* Only support 16 Bits ADC out bits */
    if(openCfg->adcOutCfg.fmt.b2AdcBits != 2U)
    {
        retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__NON_16BITS_ADC;
        goto exit;
    }

#if (defined SOC_AWR294X) || (defined SOC_AWR2544)
    /* Only support Real */
    if (openCfg->adcOutCfg.fmt.b2AdcOutFmt != 0U)
    {
        retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__ADC_FORMAT;
        goto exit;
    }
#else
    /* Only support complex ADC data format */
    if((openCfg->adcOutCfg.fmt.b2AdcOutFmt != 1U) &&
       (openCfg->adcOutCfg.fmt.b2AdcOutFmt != 2U))
    {
        retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__NON_COMPLEX_ADC_FORMAT;
        goto exit;
    }
#endif

#if (defined SOC_AWR294X) || (defined SOC_AWR2544)
    if(openCfg->adcOutCfg.fmt.b2AdcOutFmt == 0){
        outParams->adcDataFmtIsReal = 1; /* Data format is real */
    }
    else{
        outParams->adcDataFmtIsReal = 0; /* Data format is complex */
    }
#endif

    /***********************************************
     * Parse openCfg
     ***********************************************/
    /* Find number of enabled channels */
    numRxAntennas = 0U;
    for (channel = 0U; channel < SYS_COMMON_NUM_RX_CHANNEL; channel++)
    {
        if(channelRxEn & (0x1U << channel))
        {
            outParams->rxAntOrder[numRxAntennas] = channel;
            /* Track the number of receive channels: */
            numRxAntennas++;
        }
        else
        {
            outParams->rxAntOrder[channel] = 0U;
        }

        //add check to make sure lambda/2
        //check for rxAnt -= 1, 2, 4 and their limitations
    }
    outParams->numRxAntennas = numRxAntennas;



    /***********************************************
     * Parse frameCfg
     ***********************************************/
    /* Read frameCfg chirp start/stop index */
    frameChirpStartIdx  = MmwDemo_RFParser_getChirpStartIdx(ctrlCfg, subFrameIdx);
    frameChirpEndIdx    = MmwDemo_RFParser_getChirpEndIdx(ctrlCfg, subFrameIdx);
    numLoops            = MmwDemo_RFParser_getNumLoops(ctrlCfg, subFrameIdx);    
    outParams->framePeriod  =  MmwDemo_RFParser_getFramePeriod_ms(ctrlCfg, subFrameIdx);
    frameTotalChirps    = frameChirpEndIdx - frameChirpStartIdx + 1;

    /* TODO::Advance Frame only support one burst - chain limitation,  */
    if (ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        if(ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.subFrameCfg[subFrameIdx].numOfBurst != 1)
        {
            retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__NON_ONE_NUMOFBURST_FOR_ADVANCED_FRAME;
            goto exit;
        }
    }

    /* since validChirpTxEnBits is static array of 32 -> Changed to 512 */
    if(frameTotalChirps > 512U)
    {
        retVal = MMWDEMO_RFPARSER_ENOIMPL__NUM_UNIQUE_CHIRPS_MORE_THAN_32;
    }


    /* loop for profiles and find if it has valid chirps */
    /* we support only one profile in this processing chain */
    profileLoopIdx = 0;
    do
    {
        /* in this application chirp validity is not checked,
         * user needs to make sure the CLI parameters are correct
         */
        foundValidProfile = true;
        profileLoopIdx = 0;

        if(ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_CONTINUOUS)
        {
            startFreqConst = ctrlCfg->u.continuousModeCfg.contModecfg.startFreqConst;
            digOutSampleRate = ctrlCfg->u.continuousModeCfg.contModecfg.digOutSampleRate;
            outParams->numAdcSamples = ctrlCfg->u.continuousModeCfg.dataTransSize;
        }
        else
        {
            outParams->numAdcSamples = profChirpCfg->profileCfg[profileLoopIdx].numAdcSamples;
            startFreqConst = profChirpCfg->profileCfg[profileLoopIdx].startFreqConst;
            digOutSampleRate = profChirpCfg->profileCfg[profileLoopIdx].digOutSampleRate;
        }

        /* found valid chirps for the frame; set remaining parameters */
        if (foundValidProfile == true)
        {
            rlProfileCfg_t  profileCfg;
            
            outParams->validProfileIdx = profileLoopIdx;

            /* Get the profile configuration: */
            profileCfg = profChirpCfg->profileCfg[profileLoopIdx];
            
#ifndef MMW_ENABLE_NEGATIVE_FREQ_SLOPE
            /* Check frequency slope */
            if (profileCfg.freqSlopeConst < 0)
            {
                retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__NEGATIVE_FREQ_SLOPE;
                goto exit;
            }
#endif

#if (defined SOC_AWR294X) || (defined SOC_AWR2544)
            /* set other parameters */
            if (outParams->adcDataFmtIsReal){
                outParams->numRangeBins = mathUtils_pow2roundup(outParams->numAdcSamples)/2;
            }
            else{
                outParams->numRangeBins = mathUtils_pow2roundup(outParams->numAdcSamples);
            }
#endif
            outParams->numRangeBins = mathUtils_pow2roundup(outParams->numAdcSamples);

            outParams->numChirpsPerFrame = frameTotalChirps * numLoops;

            outParams->numDopplerChirps = outParams->numChirpsPerFrame/outParams->numTxAntennas;
            outParams->numDopplerBins = mathUtils_pow2roundup(outParams->numDopplerChirps);
            
            adcStart                        =   ((float)profileCfg.adcStartTimeConst * 10.f * 1.e-9);
            startFreq                       =   (float)startFreqConst/(float)(1U << 26)*rfFreqScaleFactor*(float)1e9;
            slope                           =   (float)profileCfg.freqSlopeConst * ((rfFreqScaleFactor*1e3*900.f)/(float)(1U << 26)) * 1e12;
            bandwidth                       =   (slope * outParams->numAdcSamples)/(digOutSampleRate * 1e3);
            centerFreq                      =   startFreq + bandwidth * 0.5f + adcStart * slope;            
            outParams->chirpInterval        =   (float)(profileCfg.idleTimeConst + profileCfg.rampEndTime)/1000.*10*1.e-6;

            outParams->rangeStep            =   (MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC * digOutSampleRate * 1e3) /
                                                (2.f * slope * outParams->numRangeBins);
            outParams->rangeResolution      =   (MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC * digOutSampleRate * 1e3) /
                                                (2.f * slope * outParams->numAdcSamples);
            
            outParams->dopplerStep          =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                                (2.f * outParams->numDopplerBins * outParams->numTxAntennas * 
                                                centerFreq * outParams->chirpInterval);            
            outParams->dopplerResolution    =   MMWDEMO_RFPARSER_SPEED_OF_LIGHT_IN_METERS_PER_SEC /
                                                (2.f * outParams->numChirpsPerFrame * centerFreq * outParams->chirpInterval);            
            
#ifdef MMWDEMO_RFPARSER_DBG        
            System_printf("Ant setting virtualAzim: %d , virtual Elev :%d\n",
                            outParams->numVirtualAntAzim, outParams->numVirtualAntElev);
            System_printf("startFreqConst: %d\n", profileCfg.startFreqConst);
            System_printf("rfFreqScaleFactor: %f\n", rfFreqScaleFactor);
            System_printf("bandwidth : %f GHz\n", bandwidth*1.e-9);
            System_printf("adcStartTimeConst: %d\n", profileCfg.adcStartTimeConst);
            System_printf("freqSlopeConst: %d\n", profileCfg.freqSlopeConst);
            System_printf("centerFreq: %f GHz\n", centerFreq*1.e-9);
            System_printf("dopplerStep: %f\n", outParams->dopplerStep);
            System_printf("adcStart: %f us\n", adcStart * 1e6);
            System_printf("slope: %f GHz\n", slope*1.e-9);
            System_printf("startFreq: %f GHz\n", startFreq*1.e-9);
#endif
            

#ifndef LIMITED_FEATURES

            break; //single profile ONLY
#endif
        }
        profileLoopIdx++;
    }while((profileLoopIdx < profChirpCfg->rcvProfileCfgCnt /*MMWAVE_MAX_PROFILE*/) && \
           (foundValidProfile == false));

    if (foundValidProfile == false)
    {
        retVal = MMWDEMO_RFPARSER_EINVAL__VALID_PROFILECFG_NOT_FOUND;
        goto exit;
    }

exit:
    return (retVal);
}

/**
 *  @b Description
 *  @n
 *      Helper function that parses ADCBuf configuration to be used to configure ADCBuf driver
 *
 *  @param[out] outParams      Pointer to parameters generated after parsing configuration
 *  @param[in]  adcBufCfg            Pointer to ADCBuf configuration
 *
 *  \ingroup MMWDEMO_RFPARSER_INTERNAL_FUNCTION
 *
 *  @retval
 *      Success -   0
 *      Fail        < 0, one of @ref MMWDEMO_RFPARSER_ERROR_CODE
 */
static int32_t MmwDemo_RFParser_parseADCBufCfg
(
    MmwDemo_RFParserOutParams   *outParams,
    Mmw_ADCBufCfg     *adcBufCfg
)
{
    uint16_t            numBytePerSample = 0;
    uint32_t            chirpThreshold;
    uint32_t            maxChirpThreshold;
    uint32_t            bytesPerChirp;
    int32_t             retVal = 0;

#if (defined SOC_AWR294X) || (defined SOC_AWR2544)
    /* Only support Real */
    if (adcBufCfg->adcFmt != 1)
    {
        retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__ADC_FORMAT;
        goto exit;
    }
#else
    /* Only support Complex */
    if (adcBufCfg->adcFmt != 0)
    {
        retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__NONCOMPLEX_ADC_FORMAT;
        goto exit;
    }
#endif

    if (adcBufCfg->adcFmt == 1){
        numBytePerSample = 2U; /* Real Data */
    }
    else if (adcBufCfg->adcFmt == 0){
        numBytePerSample = 4U; /* Complex Data */
    }

    /* Calculate RX channel data size and make it align to 16 bytes */
    outParams->adcBufChanDataSize =  outParams->numAdcSamples * numBytePerSample;
    outParams->adcBufChanDataSize = (outParams->adcBufChanDataSize + 15U) / 16U * 16U;

    /* Calculate max possible chirp threshold */
    bytesPerChirp = outParams->adcBufChanDataSize * outParams->numRxAntennas;

    /* find maximum number of full chirps that can fit in the ADCBUF memory, while
       also being able to divide numChirpsPerFrame, we do not want remainder processing */
    maxChirpThreshold = MmwDemo_RFParserHwCfg.adcBufSize/ bytesPerChirp;
    if (maxChirpThreshold >= outParams->numChirpsPerFrame)
    {
        maxChirpThreshold = outParams->numChirpsPerFrame;
        if (maxChirpThreshold > SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD)
        {
            /* If CQ monitor is enabled, then check max chirpthreshold */
            maxChirpThreshold = SYS_COMMON_CQ_MAX_CHIRP_THRESHOLD;
        }
    }
    else
    {
        /* Find largest divisor of numChirpsPerFrame no bigger than maxChirpThreshold */
        while (outParams->numChirpsPerFrame % maxChirpThreshold)
        {
            maxChirpThreshold--;
        }
    }

    /* ADCBuf control function requires argument alignment at 4 bytes boundary */
    chirpThreshold = adcBufCfg->chirpThreshold;

    /* if automatic, set to the calculated max */
    if (chirpThreshold == 0)
    {
        chirpThreshold = maxChirpThreshold;
    }
    else
    {
        if (chirpThreshold > maxChirpThreshold)
        {
#ifdef MMWDEMO_RFPARSER_DBG
            System_printf("Desired chirpThreshold %d higher than max possible of %d, setting to max\n",
                chirpThreshold, maxChirpThreshold);
#endif
            retVal = MMWDEMO_RFPARSER_EINVAL__CHIRP_THRESH_GREATER_THAN_MAX_ALLOWED;
            goto exit;
        }
        else
        {
            /* check for divisibility of the user provided threshold */
            if((outParams->numChirpsPerFrame % chirpThreshold) != 0)
            {
                retVal = MMWDEMO_RFPARSER_ENOTSUPPORT__NON_DIVISIBILITY_OF_CHIRP_THRESH;
                goto exit;
            }
            else
            {
                /* Chirp threshold has a valid configuration */
            }
        }
    }

#ifdef MMWDEMO_RFPARSER_DBG
    System_printf(" chirpThreshold %d max = %d, \n",
                  chirpThreshold, maxChirpThreshold);
#endif

    /* Save chirpThreshold */
    outParams->numChirpsPerChirpEvent = chirpThreshold;

exit:
    return (retVal);
}

/**************************************************************
 * Exposed APIs
 **************************************************************/

/**
 *  @b Description
 *  @n
 *      The function is used to parse chirp/profile configurations and 
 *  save configuration and derived parameter in datapath parameter structure.
 *
 *  @param[out] outParams      Pointer to parameters generated after parsing configuration
 *  @param[in]  subFrameIdx    Sub-frame index
 *  @param[in]  openCfg        Pointer to the MMWave Open configuration
 *  @param[in]  ctrlCfg        Pointer to the MMWave Control configuration
 *  @param[in]  adcBufCfg      Pointer to ADCBuf configuration
 *  @param[in]  rfFreqScaleFactor RF frequency scale factor, see SOC_getDeviceRFFreqScaleFactor API
 *  @param[in]  bpmEnable      BPM flag, 0 -disabled, 1-enabled
 *
 *  \ingroup MMWDEMO_RFPARSER_EXTERNAL_FUNCTION
 *
 *  @retval
 *      Success     - = 0
 *  @retval
 *      Error       - < 0, one of @ref MMWDEMO_RFPARSER_ERROR_CODE
 */
int32_t Mmw_RFParser_parseConfig
(
    MmwDemo_RFParserOutParams    *outParams,
    uint8_t              subFrameIdx,
    MMW_CLI_OpenCfg       *openCfg,
    MMW_CLI_CtrlCfg       *ctrlCfg,
    MMW_CLI_ProfChirpCfg  *profChirpCfg,
    Mmw_ADCBufCfg         *adcBufCfg,
    float                rfFreqScaleFactor,
    bool                 bpmEnable
)
{
    int32_t retVal;

    if(subFrameIdx < RL_MAX_SUBFRAMES)
    {
        /* Parse the profile and chirp configs and get the valid number of TX Antennas */
        retVal = MmwDemo_RFParser_parseCtrlConfig(outParams, subFrameIdx,
                                                  openCfg, ctrlCfg, profChirpCfg,
                                                  rfFreqScaleFactor, bpmEnable);

        if (retVal != 0)
        {
            goto exit;
        }
        retVal = MmwDemo_RFParser_parseADCBufCfg(outParams, adcBufCfg);
        if (retVal != 0)
        {
            goto exit;
        }
    }
    else
    {
        retVal = MMWDEMO_RFPARSER_EINVAL__NUM_SUBFRAMES;
    }

exit:
    return retVal;
}

/**
 *  @b Description
 *  @n
 *      The function is used to get number of sub-frames from the input
 *      chirp/profile configuration.
 *
 *  @param[in]  ctrlCfg        Pointer to the MMWave Control configuration
 *
 *  \ingroup MMWDEMO_RFPARSER_EXTERNAL_FUNCTION
 *
 *  @retval  Number of Sub-frames
 */
uint8_t MmwDemo_RFParser_getNumSubFrames(MMW_CLI_CtrlCfg *ctrlCfg)
{
    if(ctrlCfg->dfeDataOutputMode == MMWave_DFEDataOutputMode_ADVANCED_FRAME)
    {
        return (ctrlCfg->u.advancedFrameCfg.frameCfg.frameSeq.numOfSubFrames);
    }
    else
    {
        return (1);
    }
}
