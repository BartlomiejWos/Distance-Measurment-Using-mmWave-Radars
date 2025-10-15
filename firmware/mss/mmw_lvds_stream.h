/**
 *   @file  mmw_lvds_stream.h
 *
 *   @brief
 *      LVDS stream header file.
 *
 *  \par
 *  NOTE:
 *      (C) Copyright 2020 - 2021 Texas Instruments, Inc.
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
#ifndef MSS_LVDS_STREAM_H
#define MSS_LVDS_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

#if defined (SOC_AWR294X)
#include <kernel/dpl/SemaphoreP.h>


#elif defined (SOC_AWR2544)
#include <kernel/dpl/SemaphoreP.h>
#include <ti/demo/awr2544/mmw/mmw_lvds/mmw_lvds_stream.h>
#else

#include <ti/drivers/cbuff/cbuff.h>
#endif
#include <ti/utils/hsiheader/hsiheader.h>

/**
 * @brief   This is the maximum number of EDMA Channels which is used by
 * the HW Session
 */
#define MMWDEMO_LVDS_STREAM_HW_SESSION_MAX_EDMA_CHANNEL             11U

/**
 * @brief   This is the maximum number of EDMA Channels which is used by
 * the SW Session
 */
#define MMWDEMO_LVDS_STREAM_SW_SESSION_MAX_EDMA_CHANNEL             3U

/**
 * @brief
 *  LVDS streaming user data header
 *
 * @details
 *  The LVDS SW streaming user data header.
 */
typedef struct MmwDemo_LVDSUserDataHeader
{
    /**
     * @brief   Frame number.
     */
    uint32_t     frameNum;

    /**
     * @brief   Sub-Frame number. Always 0 when advanced frame is not enabled.
     *          Note although the subFrameNum does not need to be 16-bits (it needs to be
     *          only 8-bits), we keep it 16-bit for parsing convenience as compiler
     *          does not insert holes in this case
     */
    uint16_t     subFrameNum;

    /**
     * @brief   Number of detected objects.
     */
    uint16_t     detObjNum;
} MmwDemo_LVDSUserDataHeader_t;


/**
 * @brief
 *  LVDS streaming MCB
 *
 * @details
 *  The LVDS streaming MCB.
 */
typedef struct MmwDemo_LVDSStream_MCB
{
    /*! @brief   Handle of the EDMA driver, used for CBUFF */
    EDMA_Handle              edmaHandle;

    /**
    * @brief   Handle to the CBUFF Driver
    */
    CBUFF_Handle             cbuffHandle;

    /**
     * @brief   EDMA Channel Allocator Index for the HW Session
     */
    uint8_t                  hwSessionEDMAChannelAllocatorIndex;

    /**
     * @brief   EDMA Channel Resource Table: This is used for creating the CBUFF Session.
     */
    CBUFF_EDMAChannelCfg     hwSessionEDMAChannelTable[MMWDEMO_LVDS_STREAM_HW_SESSION_MAX_EDMA_CHANNEL];

    /**
     * @brief   EDMA Channel Allocator Index for the SW Session
     */
    uint8_t                  swSessionEDMAChannelAllocatorIndex;

    /**
     * @brief   EDMA Channel Resource Table: This is used for creating the CBUFF Session.
     */
    CBUFF_EDMAChannelCfg     swSessionEDMAChannelTable[MMWDEMO_LVDS_STREAM_SW_SESSION_MAX_EDMA_CHANNEL];

    /**
     * @brief   HW session HSI header.
     */
#if (defined SOC_AWR294X) || (defined SOC_AWR2544)
    HSIHeader*               ptrHwSessionHSIHeader;
#else
    HSIHeader                hwSessionHSIHeader;
#endif
    /**
     * @brief   True if hw session HSI header is allocated, false otherwise
     */
    bool                     isHwSessionHSIHeaderAllocated;
    
    /**
     * @brief   SW session HSI header.
     */
    HSIHeader                swSessionHSIHeader;
    
    /**
     * @brief   Handle to the HW CBUFF Session Handle.
     */
    CBUFF_SessionHandle      hwSessionHandle;

    /**
     * @brief   Handle to the SW CBUFF Session Handle.
     */
    CBUFF_SessionHandle      swSessionHandle;
    
    /**
     * @brief   Number of HW frame done interrupt received.
     */
    uint16_t                 hwFrameDoneCount;
    
    /**
     * @brief   Number of SW frame done interrupt received.
     */
    uint16_t                 swFrameDoneCount;
    
    /**
     * @brief   Semaphore handle to signal hw session done.
     */
#if (defined SOC_AWR294X) || (defined SOC_AWR2544)
    SemaphoreP_Object         hwFrameDoneSemHandle;
#else
    Semaphore_Handle         hwFrameDoneSemHandle;
#endif
    /**
     * @brief   Semaphore handle to signal sw session done.
     */
#if (defined SOC_AWR294X) || (defined SOC_AWR2544)
    SemaphoreP_Object         swFrameDoneSemHandle;
#else
    Semaphore_Handle         swFrameDoneSemHandle;
#endif
    /**
     * @brief   Pointer to user data header.
     */
    MmwDemo_LVDSUserDataHeader_t  *userDataHeader;
} MmwDemo_LVDSStream_MCB_t;

int32_t MmwDemo_LVDSStreamInit (void);
int32_t MmwDemo_LVDSStreamHwConfig (uint8_t subFrameIndx, CBUFF_OperationalMode operationMode);
// int32_t MmwDemo_LVDSStreamSwConfig (uint32_t numObjOut,
                                    // DPIF_PointCloudCartesian *objOut,
                                    // DPIF_PointCloudSideInfo *objOutSideInfo);
void MmwDemo_configLVDSHwData(uint8_t subFrameIndx, CBUFF_OperationalMode operationMode);
void MmwDemo_LVDSStreamDeleteHwSession (void);
void MmwDemo_LVDSStreamDeleteSwSession (void);

#ifdef __cplusplus
}
#endif

#endif
