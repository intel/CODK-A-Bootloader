/*
 * Copyright (c) 2016, Intel Corporation
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * \file balloc.c
 *
 * memory allocation services
 *
 * Functions are exported by os.h
 *
 */

#include <stdint.h>

#define NULL 0
/******* Framework headers : */
//#include "os/os.h"          /* framework export definitions */
//#include "os/os_types.h"    /* framework-specific types */
//#include "zephyr/common.h"
//#include "zephyr/os_config.h"
//#include "infra/log.h"

#ifdef MALLOC_ENABLE_STATISTICS
#include "zephyr/os_specific.h"   /* need _log function */
#endif


/**********************************************************
 ************** Extern variables   ************************
 **********************************************************/




/**********************************************************
 ************** Local definitions  ************************
 **********************************************************/
#define BITS_PER_UINT32 (sizeof(uint32_t) * 8)

/** Descriptor for a memory pool */
typedef struct {
    uint32_t* track;    /** block allocation tracker */
    uint32_t  start;    /** start address of the pool */
    uint32_t  end;      /** end address of the pool */
    uint16_t  count;    /** total number of blocks within the pool */
    uint16_t  size;     /** size of each memory block within the pool */
#ifdef MALLOC_ENABLE_STATISTICS
    uint32_t  max;     /** maximum number of allocated blocks at the same time */
    uint32_t  cur;     /** current number of allocated blocks */
#endif
}T_POOL_DESC;


/**********************************************************
 ************** Private variables  ************************
 **********************************************************/

#ifdef MALLOC_ENABLE_STATISTICS

/** Allocate the memory blocks and tracking variables for each pool */
#define DECLARE_MEMORY_POOL(index,size,count) \
    uint8_t  g_MemBlock_##index[count][size] ; \
    uint32_t g_MemBlock_alloc_track_##index[count/BITS_PER_UINT32+1] = { 0 };

#include "memory_pool_list.def"


/** Pool descriptor definition */
T_POOL_DESC g_MemPool[] =
{
#define DECLARE_MEMORY_POOL(index,size,count) \
{ \
/* T_POOL_DESC.track */  g_MemBlock_alloc_track_##index,\
/* T_POOL_DESC.start */  (uint32_t) g_MemBlock_##index, \
/* T_POOL_DESC.end */    (uint32_t) g_MemBlock_##index + count * size, \
/* T_POOL_DESC.count */  count, \
/* T_POOL_DESC.size */   size, \
/* T_POOL_DESC.max */    0, \
/* T_POOL_DESC.cur */    0 \
},

#include "memory_pool_list.def"
};


#else

/** Allocate the memory blocks and tracking variables for each pool */
#define DECLARE_MEMORY_POOL(index,size,count) \
    uint8_t  g_MemBlock_##index[count][size] ; \
    uint32_t g_MemBlock_alloc_track_##index[count/BITS_PER_UINT32+1] = { 0 }; \

#include "memory_pool_list.def"



/** Pool descriptor definition */
T_POOL_DESC g_MemPool [] =
{
#define DECLARE_MEMORY_POOL(index,size,count) \
{ \
/* T_POOL_DESC.track */  g_MemBlock_alloc_track_##index,\
/* T_POOL_DESC.start */  (uint32_t) g_MemBlock_##index, \
/* T_POOL_DESC.end */    (uint32_t) g_MemBlock_##index + count * size, \
/* T_POOL_DESC.count */  count, \
/* T_POOL_DESC.size */   size \
},

#include "memory_pool_list.def"
};



#endif


/** Number of memory pools */
#define NB_MEMORY_POOLS   (sizeof(g_MemPool) / sizeof(T_POOL_DESC))

/**********************************************************
 ************** Private functions  ************************
 **********************************************************/

/**
 * \brief  Return the next free block of a pool and
 *   mark it as reserved/allocated
 *
 * \param pool index of the pool in g_MemPool
 *
 * \return allocated buffer or NULL if none is
 *   available
 */
static void* memblock_alloc (uint32_t pool)
{
    uint16_t block;

    for (block = 0; block < g_MemPool[pool].count; block++)
    {
        if (((g_MemPool[pool].track)[block/BITS_PER_UINT32] & 1 << (BITS_PER_UINT32 - 1 - (block%BITS_PER_UINT32))) == 0)
        {
            (g_MemPool[pool].track)[block/BITS_PER_UINT32] = (g_MemPool[pool].track)[block/BITS_PER_UINT32] | (1 << (BITS_PER_UINT32 - 1 - (block%BITS_PER_UINT32)));
#ifdef MALLOC_ENABLE_STATISTICS
            g_MemPool[pool].cur = g_MemPool[pool].cur +1;
            if (g_MemPool[pool].cur > g_MemPool[pool].max)
                g_MemPool[pool].max = g_MemPool[pool].cur;
#endif
            return (void*) ( g_MemPool[pool].start + g_MemPool[pool].size * block ) ;
        }
    }
    return NULL;
}



/**
 * \brief Free an allocated block from a pool
 *
 * \param pool index of the pool in g_MemPool
 *
 * \param ptr points to the start of the block
 *     to free
 *
 */
static void memblock_free (uint32_t pool, void* ptr)
{
    uint16_t block ;

    block = ((uint32_t)ptr - g_MemPool[pool].start) / g_MemPool[pool].size ;
    if (block < g_MemPool[pool].count)
    {
        (g_MemPool[pool].track)[block/BITS_PER_UINT32] &= ~(1 << (BITS_PER_UINT32-1-(block%BITS_PER_UINT32)));
#ifdef MALLOC_ENABLE_STATISTICS
        g_MemPool[pool].cur = g_MemPool[pool].cur - 1;
#endif
    }
#ifdef __DEBUG_OS_ABSTRACTION_BALLOC
    else
        _log ("ERR: memblock_free: ptr 0x%X is not within pool %d [0x%X , 0x%X]", ptr, pool, g_MemPool[pool].start, g_MemPool[pool].end);
#endif
}




/**
 * \brief Test if a block is allocated
 *
 * \param pool index of the pool in g_MemPool
 *
 * \param ptr points to the start of the block
 *
 * \return 1 if the block is allocated/reserved,
 *   0 if the block is free
 *
 */
static unsigned char memblock_used (uint32_t pool, void* ptr)
{
    uint16_t block ;
    block = ((uint32_t)ptr - g_MemPool[pool].start) / g_MemPool[pool].size ;
    if (block < g_MemPool[pool].count)
    {
        if ( ((g_MemPool[pool].track)[block/BITS_PER_UINT32] & (1 << (BITS_PER_UINT32 - 1 -(block%BITS_PER_UINT32)))) != 0 )
        {
            return 1;
        }
    }
    return 0;
}

/**********************************************************
 ************** Exported functions ************************
 **********************************************************/

/*----- Initialization  */

/**
 * Initialize the resources used by the framework's memory allocation services
 *
 * IMPORTANT : This function must be called during the initialization
 *             of the OS abstraction layer.
 *             This function shall only be called once after reset.
 */
void os_abstraction_init_malloc(void)
{
}

/**
 * \brief Reserves a block of memory
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * This function returns a pointer on the start of
 * a reserved memory block whose size is equal or
 * larger than the requested size.
 *
 * The returned pointer shall be null if the function
 * fails.
 *
 * This function may panic if err is null and
 *  - size is null or bigger than allowed, or
 *  - there is not enough available memory
 *
 * \param size number of bytes to reserve
 *
 *
 * \param err execution status:
 *    E_OS_OK : block was successfully reserved
 *    E_OS_ERR : size is null
 *    E_OS_ERR_NO_MEMORY: there is not enough available
 *              memory
 *    E_OS_ERR_NOT_ALLOWED : size is bigger than the
 *         biggest block size defined in os_config.h
 *
 * \return pointer to the reserved memory block
 *    or null if no block is available
 */
void* balloc (uint32_t size)
{
    void* buffer = NULL ;
    uint8_t poolIdx ;

    if (size > 0)
    {
        /* find the first block size greater or equal to requested size */
        poolIdx = 0;
        while (poolIdx < NB_MEMORY_POOLS &&
		size > g_MemPool[poolIdx].size)
        {
            poolIdx ++;
        }

        /* reserve the block */
        if ( poolIdx < NB_MEMORY_POOLS )
        {
#ifdef MALLOC_ALLOW_OUTCLASS
            /* loop until an available (maybe larger) block is found */
            do
            {
                if (size <= g_MemPool[poolIdx].size )
                { /* this condition may be false if pools are not sorted according to block size */
#endif
                buffer = memblock_alloc(poolIdx);

#ifdef MALLOC_ALLOW_OUTCLASS
                }

                if ( NULL == buffer )
                {
                    poolIdx ++;
                }
            }
            while ( (poolIdx < NB_MEMORY_POOLS ) && ( NULL == buffer ) );
#endif
            if ( NULL == buffer )
            {/* All blocks of relevant size are already reserved */
            }
        }
        else
        { /* Configuration does not define blocks large enough for the requested size */
        }
    }
    else
    { /* invalid function parameter */
    }

    /* set err or panic if err == NULL and localErr != E_OS_OK */
    return (buffer);
}

/**
 * \brief Frees a block of memory
 *
 * Authorized execution levels:  task, fiber, ISR
 *
 * This function frees a memory block that was
 * reserved by malloc.
 *
 * The "buffer" parameter must point to the
 * start of the reserved block (i.e. it shall
 * be a pointer returned by malloc).
 *
 * \param buffer pointer returned by malloc
 *
 * \return execution status:
 *    E_OS_OK : block was successfully freed
 *    E_OS_ERR : "buffer" param did not match
 *        any reserved block
 */
void bfree(void* buffer)
{
    uint8_t poolIdx ;

    /* find which pool the buffer was allocated from */
    poolIdx = 0;
    while (( NULL != buffer ) && ( poolIdx < NB_MEMORY_POOLS ))
    {
        /* check if buffer is within g_MemPool[poolIdx] */
        if ( ( (uint32_t) buffer >= g_MemPool[poolIdx].start ) &&
             ( (uint32_t) buffer < g_MemPool[poolIdx].end ) )
        {
            if ( 0 != memblock_used (poolIdx, buffer))
            {
                memblock_free (poolIdx, buffer);
            }
            /* else: buffer is not marked as used, keep err = E_OS_ERR */
            buffer = NULL; /* buffer was found in the pools, end the loop */
        }
        else
        {/* buffer does not belong to g_MemPool[poolIdx], go to the next one */
            poolIdx ++;
        }
    }
}



#ifdef MALLOC_ENABLE_STATISTICS
/**
 * \brief Log the current statistics on memory pool usage
 */
void print_memory_allocation_stats (void)
{
    uint32_t pool;

    _log ("\n\n------------------------------------\n");
    _log ("------ Memory allocation statistics:\n");

    for ( pool = 0; pool < NB_MEMORY_POOLS; pool ++ )
    {
        _log ("Pool %d Bytes -> current = %d, maximum = %d \n", g_MemPool[pool].size, g_MemPool[pool].cur, g_MemPool[pool].max);
    }
    _log ("\n\n------------------------------------\n");

}
#endif
