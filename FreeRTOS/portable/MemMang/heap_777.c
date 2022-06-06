/*
 * FreeRTOS Kernel V10.2.1
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/*
 * A sample implementation of pvPortMalloc() and vPortFree() that permits
 * allocated blocks to be freed, but does not combine adjacent free blocks
 * into a single larger block (and so will fragment memory).  See heap_4.c for
 * an equivalent that does combine adjacent blocks into single larger blocks.
 *
 * See heap_1.c, heap_3.c and heap_4.c for alternative implementations, and the
 * memory management pages of http://www.FreeRTOS.org for more information.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"

#undef MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#if( configSUPPORT_DYNAMIC_ALLOCATION == 0 )
	#error This file must not be used if configSUPPORT_DYNAMIC_ALLOCATION is 0
#endif

/* A few bytes might be lost to byte aligning the heap start address. */
#define configADJUSTED_HEAP_SIZE	( configTOTAL_HEAP_SIZE - portBYTE_ALIGNMENT )

/*
 * Initialises the heap structures before their first use.
 */
static void prvHeapInit( void );
static BaseType_t xPortPoolInit(size_t *);

/* Allocate the memory for the heap. */
#if( configAPPLICATION_ALLOCATED_HEAP == 1 )
	/* The application writer has already defined the array used for the RTOS
	heap - probably so it can be placed in a special segment or address. */
	extern uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#else
	static uint8_t ucHeap[ configTOTAL_HEAP_SIZE ];
#endif /* configAPPLICATION_ALLOCATED_HEAP */

/* Define the linked list structure. Block_t is used to link free blocks with
 * same size, and Pool_t is used to record the head of each list.
 */
typedef struct Pool Pool_t;
typedef struct Block Block_t;

struct Pool
{
    Block_t *pxFirstFree;   /* The first free block in this pool. */
    size_t xBlockSize;      /* The size of the free block in this pool. */
};

struct Block
{
    Pool_t *pxPool;         /* The pool this block belongs to. */
    Block_t *pxNext;        /* The next free block in the list. */
};

static const uint16_t heapSTRUCT_SIZE	= ( ( sizeof ( Block_t ) + ( portBYTE_ALIGNMENT - 1 ) ) & ~portBYTE_ALIGNMENT_MASK );
#define heapMINIMUM_BLOCK_SIZE	( ( size_t ) ( heapSTRUCT_SIZE * 2 ) )

#define heapMAXIMUM_POOL_NUM 10
const size_t xSizeList[heapMAXIMUM_POOL_NUM] = { 80, 160, 240, 320, 400, 480, 560, 640, 720, 1000 };

static BaseType_t xHeapHasBeenInitialised = pdFALSE;
static BaseType_t xPoolHasBeenInitialised = pdFALSE;

/* Create a couple of pools. */
static Pool_t xPool[heapMAXIMUM_POOL_NUM];

/* Keeps track of the unallocated heap's head. */
Block_t *pxFreeHeap = NULL;

/* Keeps track of the number of free bytes remaining, but says nothing about
fragmentation. */
static size_t xFreeBytesRemaining = configADJUSTED_HEAP_SIZE;

/* STATIC FUNCTIONS ARE DEFINED AS MACROS TO MINIMIZE THE FUNCTION CALL DEPTH. */

/*-----------------------------------------------------------*/

void *pvPortMalloc( size_t xWantedSize )
{
void *pvReturn = NULL;
int iter = 0;

	vTaskSuspendAll();
	{
		/* If this is the first call to malloc then the heap will require
		initialization. */
		if( xHeapHasBeenInitialised == pdFALSE )
		{
			prvHeapInit();
			xHeapHasBeenInitialised = pdTRUE;
            xPortPoolInit(xSizeList);
		}

		/* The wanted size is increased so it can contain a Block_t
		structure in addition to the requested amount of bytes. */
		if( xWantedSize > 0 )
		{
            /* Find the best-fit size from the pool size. */
            for (iter = 0; iter < heapMAXIMUM_POOL_NUM; ++iter) {
                if (xPool[iter].xBlockSize >= xWantedSize) {
                    xWantedSize = xPool[iter].xBlockSize;
                    break;
                }
            }

			xWantedSize += heapSTRUCT_SIZE;

			/* Ensure that blocks are always aligned to the required number of bytes. */
			if( ( xWantedSize & portBYTE_ALIGNMENT_MASK ) != 0 )
			{
				/* Byte alignment required. */
				xWantedSize += ( portBYTE_ALIGNMENT - ( xWantedSize & portBYTE_ALIGNMENT_MASK ) );
			}
		}

		if( ( xWantedSize > 0 ) && ( xWantedSize < configADJUSTED_HEAP_SIZE ) )
		{
            if (xPool[iter].pxFirstFree != NULL) {

                /* Return the memory space - jumping over the Block_t structure at its 
                start. */
                pvReturn = (void *)((uint8_t *)xPool[iter].pxFirstFree + heapSTRUCT_SIZE);

                /* This block is being returned for use so must be taken out of the
				list of free blocks. */
                xPool[iter].pxFirstFree = xPool[iter].pxFirstFree->pxNext;

            } else {
				
                /* The heap is to be split into two. Create a new block
                following the number of bytes requested. The void cast is
                used to prevent byte alignment warnings from the compiler. */
                pxFreeHeap->pxPool = &(xPool[iter]);
                pvReturn = (void *)((uint8_t *)pxFreeHeap + heapSTRUCT_SIZE);
                pxFreeHeap = (void *)((uint8_t *)pxFreeHeap + xWantedSize);

			}
		}
	}
	( void ) xTaskResumeAll();

	#if( configUSE_MALLOC_FAILED_HOOK == 1 )
	{
		if( pvReturn == NULL )
		{
			extern void vApplicationMallocFailedHook( void );
			vApplicationMallocFailedHook();
		}
	}
	#endif

	return pvReturn;
}
/*-----------------------------------------------------------*/

void vPortFree( void *pv )
{
uint8_t *puc = ( uint8_t * ) pv;
Block_t *pxLink;

	if( pv != NULL )
	{
		/* The memory being freed will have an Block_t structure immediately
		before it. */
		puc -= heapSTRUCT_SIZE;

		/* This unexpected casting is to keep some compilers from issuing
		byte alignment warnings. */
		pxLink = ( void * ) puc;

		vTaskSuspendAll();
		{
			/* Add this block to the list of free blocks. */
			pxLink->pxNext = pxLink->pxPool->pxFirstFree;
            pxLink->pxPool->pxFirstFree = pxLink;
		}
		( void ) xTaskResumeAll();
	}
}
/*-----------------------------------------------------------*/

size_t xPortGetFreeHeapSize( void )
{
	return xFreeBytesRemaining;
}
/*-----------------------------------------------------------*/

void vPortInitialiseBlocks( void )
{
	/* This just exists to keep the linker quiet. */
}
/*-----------------------------------------------------------*/

static void prvHeapInit( void )
{
uint8_t *pucAlignedHeap;

	/* Ensure the heap starts on a correctly aligned boundary. */
	pucAlignedHeap = ( uint8_t * ) ( ( ( portPOINTER_SIZE_TYPE ) &ucHeap[ portBYTE_ALIGNMENT ] ) & ( ~( ( portPOINTER_SIZE_TYPE ) portBYTE_ALIGNMENT_MASK ) ) );

	/* To start with there is a single free block that is sized to take up the
	entire heap space. */
	pxFreeHeap = ( void * ) pucAlignedHeap;
}
/*-----------------------------------------------------------*/

void vPrintFreeList(void)
{
    char data[100];
	Block_t *current;

	sprintf(data, "StartAddress heapSTRUCT_SIZE xBlockSize EndAddress\n\r");
	HAL_UART_Transmit(&huart2, (uint8_t *)data, strlen(data), 0xffff);

	for (int i = 0; i < heapMAXIMUM_POOL_NUM; ++i) {
        current = xPool[i].pxFirstFree;
        while (current) {
            sprintf(data, "%p         %d           %4d         %p\n\r", (void *)current, heapSTRUCT_SIZE, xPool[i].xBlockSize + heapSTRUCT_SIZE, (void *)current + xPool[i].xBlockSize + heapSTRUCT_SIZE);
            HAL_UART_Transmit(&huart2, (uint8_t *)data, strlen(data), 0xffff);
            current = current->pxNext;
        }
	}

	sprintf(data, "configADJUSTED_HEAP_SIZE: %0d xFreeBytesRemaining: %0d\n\r", configADJUSTED_HEAP_SIZE, xFreeBytesRemaining);
    HAL_UART_Transmit(&huart2, (uint8_t *)data, strlen(data), 0xffff);
}

static BaseType_t xPortPoolInit(size_t *pxSizeList)
{
    if (xPoolHasBeenInitialised == pdTRUE)
        return pdFALSE;
    
    vTaskSuspendAll();
	{
		if(xHeapHasBeenInitialised == pdFALSE)
		{
			prvHeapInit();
			xHeapHasBeenInitialised = pdTRUE;
		}
        // TODO: need to sort the size list?
        for (int i = 0; i < heapMAXIMUM_POOL_NUM; ++i) {
            xPool[i].pxFirstFree = NULL;
            xPool[i].xBlockSize = *(pxSizeList + i);
        }
        xPoolHasBeenInitialised = pdTRUE;
	}
	( void ) xTaskResumeAll();

    return pdTRUE;
}
