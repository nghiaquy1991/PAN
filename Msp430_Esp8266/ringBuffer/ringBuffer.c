/**
  ******************************************************************************
  * @file        ringBuffer.c
  * @author      OS Team
  * @version     V0.0.1
  * @date        4-September-2016
  * @brief       this file is ring buffer protocol for  asynchronous transmission 
  * @revision
  ******************************************************************************
  */
  
  
/******************************************************************************
**                      INCLUDE
*******************************************************************************/
#include "ringBuffer.h"

/******************************************************************************
**                      VARIABLE
*******************************************************************************/

/*
 *@functions: ringBufferPush
 *@brief    : push array into asyn buffer
 *@param    : ringBuffer, the pushed array
 *@return   : RINGBUFF_OK,RINGBUFF_FULL
 */

uint8_t ringBufferPush(ringBuffer *c, uint8_t* data)
{
    uint8_t next = c->head + 1;
    //check the maximum buffer
    if (next >= MAX_RING_BUFFER)
        next = 0;

    // check if buffer is full
    if (next == c->tail)
        return RINGBUFF_FULL;  // quit with full message

    //copy data
    strcpy((char *)c->buffer[c->head],(char*)data);

    //next string
    c->head = next;
    return RINGBUFF_OK;
}

/*
 *@functions: ringBufferPop
 *@brief    : pop array from asyn buffer
 *@param    : ringBuffer, the poped array
 *@return   : RINGBUFF_OK,RINGBUFF_EMPTY
 */

uint8_t ringBufferPop(ringBuffer *c, uint8_t *data)
{
	uint8_t next = c->tail + 1;

	//check the maximum buffer
    if (next >= MAX_RING_BUFFER)
        next = 0;

    // if the head isn't ahead of the tail, we don't have any
	// buffer is empty
    if (c->head == c->tail)
    {
        // copy empty string data
        strcpy((char*)data,(char*)emptyString);
        // quit with empty message
        return RINGBUFF_EMPTY;
    }

    // copy data
    strcpy((char*)data,(char*)c->buffer[c->tail]);
    //nexy tail
    c->tail = next;
/*    if (next < c->head)
    {
    	// post event until ringBuffer to empty
    	//Event_post(myEvent, Event_Id_01);
    	return RINGBUFF_HAS_DATA;
    }*/

    return RINGBUFF_OK;
}
