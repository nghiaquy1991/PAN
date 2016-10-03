/**
  ******************************************************************************
  * @file        ringBuffer.h
  * @author      OS Team
  * @version     V0.0.1
  * @date        4-September-2016
  * @brief       this file is header file of ringBuffer.c 
  * @revision
  ******************************************************************************
  */

#ifndef		_RINGBUFFER_H_
#define		_RINGBUFFER_H_
  
  
/******************************************************************************
**                      INCLUDE
*******************************************************************************/
#include "string.h"
#include "stdint.h"


/******************************************************************************
**                      DEFIINITIONS
*******************************************************************************/

#define MAX_INDEX_RING_BUFFER        20
#define MAX_RING_BUFFER              10


#define TEST_RING_BUFFER             0

uint8_t emptyString [20];





//ring buffer struct
typedef struct
{
    uint8_t buffer[MAX_RING_BUFFER][MAX_INDEX_RING_BUFFER];
    uint8_t head;
    uint8_t tail;
}ringBuffer;

//ringbuffer status
typedef enum
{
	RINGBUFF_OK = 0,
	RINGBUFF_FULL,
	RINGBUFF_EMPTY,
	RINGBUFF_HAS_DATA
}ringBufferStatus;


/******************************************************************************
**                      FUNCTIONS
*******************************************************************************/

uint8_t ringBufferPush(ringBuffer *c, uint8_t* data);
uint8_t ringBufferPop (ringBuffer *c, uint8_t *data);

#endif
