/*
 *   @file  osi_tirtos.c
 *
 *   @brief
 *      OSAL Porting Interface which is required by the mmWave Link
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

/**************************************************************************
 *************************** Include Files ********************************
 **************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* OS Porting driver files */
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/SemaphoreP.h>
#if (defined SOC_AWR294X) || (defined SOC_AWR2544) 
#include <ti/common/syscommon.h>
#elif
#include <ti/common/sys_common.h>
#endif
/* mmWave Link Include File */
#include <mmwavelink.h>
#include "FreeRTOS.h"
#include "task.h"

typedef struct MMWave_osalSem_t
{
    /**
     * @brief   Semahore object
     */
    SemaphoreP_Object           object;

    /**
     * @brief   Flag to check if semaphore is alread allocated.
     */
    uint32_t                    isUsed;
}MMWave_osalSem;


/**
 * @brief   Mutex object structure.
 */
MMWave_osalSem              osalMutex;

/**
 * @brief   Semaphore object structure.
 */
MMWave_osalSem              osalSem;

/**
 *  @b Description
 *  @n
 *      Registered OSAL Function with the Radar Link to create a Mutex
 *
 *  @retval
 *      Not Applicable.
 */
rlInt32_t Osal_mutexCreate(rlOsiMutexHdl_t* mutexHandle, rlInt8_t* name)
{
    int32_t  retVal = MINUS_ONE;

    retVal = SemaphoreP_constructMutex(&osalMutex.object);

    if(retVal == SystemP_FAILURE)
    {
       /* Error: Unable to create the semaphore */
       retVal       = MINUS_ONE;
       *mutexHandle = NULL;
    }
    else
    {
       /* Successfully created the semaphore */
       retVal    = 0;
       *mutexHandle = (rlOsiMutexHdl_t*)&osalMutex.object;
    }

    return (rlInt32_t)retVal;
}

/**
 *  @b Description
 *  @n
 *      Registered OSAL Function with the Radar Link to lock the Mutex
 *
 *  @retval
 *      Not Applicable.
 */
rlInt32_t Osal_mutexLock(rlOsiMutexHdl_t* mutexHandle, rlOsiTime_t timeout)
{
    uint32_t   semTimeout;

    /* Translate the timeout from mmWave link format to the semaphore OSAL module */
    if (timeout == RL_OSI_WAIT_FOREVER)
    {
        /* Semaphore timeout is set to wait forever */
        semTimeout = SystemP_WAIT_FOREVER;
    }
    else
    {
        /* Set the semaphore timeout. */
        semTimeout = timeout;
    }

    /* Pend on the semaphore: */
    SemaphoreP_pend((SemaphoreP_Object*)(*mutexHandle), semTimeout);
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Registered OSAL Function with the Radar Link to unlock the Mutex
 *
 *  @retval
 *      Not Applicable.
 */
rlInt32_t Osal_mutexUnlock(rlOsiMutexHdl_t* mutexHandle)
{
    /* Post the semaphore */
    SemaphoreP_post((SemaphoreP_Object *)(*mutexHandle));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Registered OSAL Function with the Radar Link to delete the Mutex
 *
 *  @retval
 *      Not Applicable.
 */
rlInt32_t Osal_mutexDelete(rlOsiMutexHdl_t* mutexHandle)
{
    // rlInt32_t retVal = MINUS_ONE;

    /* Delete the semaphore: */
    SemaphoreP_destruct((SemaphoreP_Object*)(*mutexHandle));

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Registered OSAL Function with the Radar Link to create a semaphore
 *
 *  @retval
 *      Not Applicable.
 */
rlInt32_t Osal_semCreate(rlOsiSemHdl_t* semHandle, rlInt8_t* name)
{
    // SemaphoreP_Object           object;
    int32_t              retVal = MINUS_ONE;

    retVal = SemaphoreP_constructBinary(&osalSem.object, 0);

    if(retVal == SystemP_FAILURE)
    {
        /* Error: Unable to create the semaphore */
        retVal       = MINUS_ONE;
        *semHandle = NULL;
    }
    else
    {
        /* Successfully created the semaphore */
        retVal    = 0;
        *semHandle = (rlOsiSemHdl_t*)&osalSem.object;
    }

    return (rlInt32_t)retVal;
}

/**
 *  @b Description
 *  @n
 *      Registered OSAL Function with the Radar Link to wait on a semaphore
 *
 *  @retval
 *      Not Applicable.
 */
rlInt32_t Osal_semWait(rlOsiSemHdl_t* semHandle, rlOsiTime_t timeout)
{
    uint32_t   semTimeout;

    /* Translate the timeout from mmWave link format to the semaphore OSAL module */
    if (timeout == RL_OSI_WAIT_FOREVER)
    {
        /* Semaphore timeout is set to wait forever */
        semTimeout = SystemP_WAIT_FOREVER;
    }
    else
    {
        /* Set the semaphore timeout. */
        semTimeout = timeout;
    }

    /* Pend on the semaphore: */
    SemaphoreP_pend((SemaphoreP_Object*)(*semHandle), semTimeout);

    return 0;
}

/**
 *  @b Description
 *  @n
 *      Registered OSAL Function with the Radar Link to release the semaphore
 *
 *  @retval
 *      Not Applicable.
 */
rlInt32_t Osal_semSignal(rlOsiSemHdl_t* semHandle)
{
    /* Post the semaphore */
    SemaphoreP_post((SemaphoreP_Object*)(*semHandle));
    return 0;
}

/**
 *  @b Description
 *  @n
 *      Registered OSAL Function with the Radar Link to delete the semaphore
 *
 *  @retval
 *      Not Applicable.
 */
rlInt32_t Osal_semDelete(rlOsiSemHdl_t* semHandle)
{
    // rlInt32_t retVal = MINUS_ONE;

    /* Delete the semaphore: */
    SemaphoreP_destruct((SemaphoreP_Object*)(*semHandle));

    return 0;
}
#if 0
/**
 *  @b Description
 *  @n
 *      Registered OSAL Function with the Radar Link to suspend the task for
 *      a specific delay period
 *
 *  @retval
 *      Not Applicable.
 */
rlInt32_t Osal_delay(rlUInt32_t delay)
{
    Task_sleep(delay);
    return 0;
}
#endif
